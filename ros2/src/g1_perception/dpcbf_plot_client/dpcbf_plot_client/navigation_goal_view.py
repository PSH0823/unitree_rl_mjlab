"""Interactive odom-frame navigation view and goal publisher.

Left press selects the goal position, dragging selects its heading, and
release publishes geometry_msgs/PoseStamped on /navigation/goal.  Right click
publishes /navigation/stop.  The view is intentionally an operator-side ROS
node; the MuJoCo window renders the same /navigation/markers independently.
"""

import math
import threading
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle, FancyArrowPatch
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker, MarkerArray

try:
    from obstacle_detector.msg import Obstacles
except ImportError:  # pragma: no cover
    Obstacles = None


def _best_effort():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


class NavigationGoalHub(Node):
    def __init__(self):
        super().__init__('navigation_goal_view')
        self.world_frame = self.declare_parameter('world_frame', 'odom').value
        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value
        self.obstacle_topic = self.declare_parameter(
            'obstacle_topic', '/obstacles_safe').value
        self.marker_topic = self.declare_parameter(
            'marker_topic', '/navigation/markers').value
        self.goal_topic = self.declare_parameter(
            'goal_topic', '/navigation/goal').value
        self.stop_topic = self.declare_parameter(
            'stop_topic', '/navigation/stop').value
        self.arena_width = float(self.declare_parameter(
            'arena_width', 10.0).value)
        self.arena_height = float(self.declare_parameter(
            'arena_height', 10.0).value)
        self.arena_center_x = float(self.declare_parameter(
            'arena_center_x', 0.0).value)
        self.arena_center_y = float(self.declare_parameter(
            'arena_center_y', 0.0).value)
        self.robot_radius = float(self.declare_parameter(
            'robot_radius', 0.3).value)
        self.stale_after = float(self.declare_parameter(
            'stale_after', 0.75).value)

        self._lock = threading.Lock()
        self._robot = None
        self._obstacles = []
        self._curves = []
        self._arrows = []
        self._goal = None
        self._rx = {'odom': None, 'obstacles': None, 'markers': None}

        self.create_subscription(Odometry, self.odom_topic,
                                 self._on_odom, _best_effort())
        if Obstacles is not None:
            self.create_subscription(Obstacles, self.obstacle_topic,
                                     self._on_obstacles, _best_effort())
        else:
            self.get_logger().warning(
                'obstacle_detector messages unavailable; obstacle layer disabled')
        self.create_subscription(MarkerArray, self.marker_topic,
                                 self._on_markers, _best_effort())
        # Goals are live operator commands, not latched configuration. Volatile
        # durability prevents an old drag from replaying when the controller or
        # Navigation state starts later.
        goal_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._goal_pub = self.create_publisher(
            PoseStamped, self.goal_topic, goal_qos)
        self._stop_pub = self.create_publisher(
            Empty, self.stop_topic, QoSProfile(depth=1))

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._robot = (msg.pose.pose.position.x,
                           msg.pose.pose.position.y, yaw)
            self._rx['odom'] = time.monotonic()

    def _on_obstacles(self, msg):
        circles = [(c.center.x, c.center.y, c.radius)
                   for c in msg.circles]
        with self._lock:
            self._obstacles = circles
            self._rx['obstacles'] = time.monotonic()

    def _on_markers(self, msg):
        curves, arrows, goal, goal_yaw = [], [], None, None
        for marker in msg.markers:
            if marker.action == Marker.DELETEALL:
                continue
            if marker.ns == 'dpcbf_boundary' and marker.type == Marker.LINE_STRIP:
                curves.append(([(p.x, p.y) for p in marker.points],
                               (marker.color.r, marker.color.g,
                                marker.color.b, marker.color.a)))
            elif marker.ns == 'relative_velocity_vector' \
                    and marker.type == Marker.LINE_STRIP \
                    and len(marker.points) >= 2:
                arrows.append(((marker.points[0].x, marker.points[0].y),
                               (marker.points[-1].x, marker.points[-1].y),
                               (marker.color.r, marker.color.g,
                                marker.color.b, marker.color.a)))
            elif marker.ns == 'navigation_command' \
                    and marker.type == Marker.ARROW and len(marker.points) >= 2:
                arrows.append(((marker.points[0].x, marker.points[0].y),
                               (marker.points[1].x, marker.points[1].y),
                               (marker.color.r, marker.color.g,
                                marker.color.b, marker.color.a)))
            elif marker.ns == 'goal' and marker.type == Marker.CYLINDER:
                goal = (marker.pose.position.x, marker.pose.position.y,
                        marker.scale.x * 0.5)
            elif marker.ns == 'goal_heading' and marker.type == Marker.CUBE:
                q = marker.pose.orientation
                goal_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._curves, self._arrows = curves, arrows
            if goal is not None:
                if goal_yaw is None and self._goal is not None:
                    goal_yaw = self._goal[3]
                self._goal = (*goal, 0.0 if goal_yaw is None else goal_yaw)
            self._rx['markers'] = time.monotonic()

    def publish_goal(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(0.5 * yaw)
        msg.pose.orientation.w = math.cos(0.5 * yaw)
        self._goal_pub.publish(msg)
        with self._lock:
            self._goal = (x, y, 0.3, yaw)

    def stop(self):
        self._stop_pub.publish(Empty())
        with self._lock:
            self._goal = None

    def snapshot(self):
        with self._lock:
            return (self._robot, list(self._obstacles), list(self._curves),
                    list(self._arrows), self._goal, dict(self._rx))


class NavigationGoalView:
    def __init__(self, hub):
        self.hub = hub
        self.fig, self.ax = plt.subplots(figsize=(8.5, 8.0))
        self._press = None
        self._drag = None
        self.fig.canvas.mpl_connect('button_press_event', self._on_press)
        self.fig.canvas.mpl_connect('motion_notify_event', self._on_motion)
        self.fig.canvas.mpl_connect('button_release_event', self._on_release)

    def _inside(self, event):
        return event.inaxes is self.ax and event.xdata is not None \
            and event.ydata is not None

    def _on_press(self, event):
        if not self._inside(event):
            return
        if event.button == 3:
            self.hub.stop()
            self._press = self._drag = None
        elif event.button == 1:
            self._press = (event.xdata, event.ydata)
            self._drag = self._press

    def _on_motion(self, event):
        if self._press is not None and self._inside(event):
            self._drag = (event.xdata, event.ydata)

    def _on_release(self, event):
        if event.button != 1 or self._press is None:
            return
        end = self._drag if self._drag is not None else self._press
        dx, dy = end[0] - self._press[0], end[1] - self._press[1]
        yaw = math.atan2(dy, dx) if math.hypot(dx, dy) >= 0.05 else 0.0
        self.hub.publish_goal(self._press[0], self._press[1], yaw)
        self._press = self._drag = None

    def draw(self, _frame):
        robot, obstacles, curves, arrows, goal, rx = self.hub.snapshot()
        ax = self.ax
        ax.clear()
        xmin = self.hub.arena_center_x - self.hub.arena_width * 0.5
        xmax = self.hub.arena_center_x + self.hub.arena_width * 0.5
        ymin = self.hub.arena_center_y - self.hub.arena_height * 0.5
        ymax = self.hub.arena_center_y + self.hub.arena_height * 0.5
        ax.set_xlim(xmin - 0.3, xmax + 0.3)
        ax.set_ylim(ymin - 0.3, ymax + 0.3)
        ax.set_aspect('equal', adjustable='box')
        ax.grid(True, alpha=0.2)
        ax.plot([xmin, xmax, xmax, xmin, xmin],
                [ymin, ymin, ymax, ymax, ymin], color='#777777', lw=2)
        for x, y, radius in obstacles:
            ax.add_patch(Circle((x, y), radius, facecolor='#315d8a',
                                edgecolor='#173451', alpha=0.72, lw=1.5))
        for points, color in curves:
            if len(points) >= 2:
                ax.plot([p[0] for p in points], [p[1] for p in points],
                        color=color, lw=2)
        for start, end, color in arrows:
            ax.add_patch(FancyArrowPatch(start, end, arrowstyle='-|>',
                                         mutation_scale=12, color=color,
                                         linewidth=2))
        if robot is not None:
            x, y, yaw = robot
            ax.add_patch(Circle((x, y), self.hub.robot_radius,
                                facecolor='#d8d8d8', edgecolor='#555555', lw=2))
            ax.arrow(x, y, 0.45 * math.cos(yaw), 0.45 * math.sin(yaw),
                     width=0.025, color='#444444', length_includes_head=True)
        if goal is not None:
            ax.add_patch(Circle((goal[0], goal[1]), goal[2],
                                facecolor='#94e0ad', edgecolor='#5e8069',
                                alpha=0.30, lw=2))
            ax.plot([goal[0], goal[0] + 0.92 * goal[2] * math.cos(goal[3])],
                    [goal[1], goal[1] + 0.92 * goal[2] * math.sin(goal[3])],
                    color='#d1e3d8', linewidth=5.0,
                    solid_capstyle='butt', zorder=5)
        if self._press is not None and self._drag is not None:
            dx = self._drag[0] - self._press[0]
            dy = self._drag[1] - self._press[1]
            yaw = math.atan2(dy, dx) if math.hypot(dx, dy) >= 0.05 else 0.0
            preview_length = 0.3 * 0.92
            ax.plot([self._press[0],
                     self._press[0] + preview_length * math.cos(yaw)],
                    [self._press[1],
                     self._press[1] + preview_length * math.sin(yaw)],
                    color='#d1e3d8', linewidth=5.0,
                    solid_capstyle='butt', zorder=6)
            ax.scatter([self._press[0]], [self._press[1]], s=60,
                       facecolor='none', edgecolor='#5e8069')
        now = time.monotonic()
        status = []
        for name in ('odom', 'obstacles', 'markers'):
            age = math.inf if rx[name] is None else now - rx[name]
            status.append(f'{name}: {"STALE" if age > self.hub.stale_after else f"{age:.2f}s"}')
        ax.set_title('Navigation goal: left-click + drag | stop: right-click\n'
                     + '   '.join(status))
        ax.set_xlabel('odom x [m]')
        ax.set_ylabel('odom y [m]')


def main(args=None):
    rclpy.init(args=args)
    hub = NavigationGoalHub()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(hub)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    view = NavigationGoalView(hub)
    animation = FuncAnimation(view.fig, view.draw, interval=50,
                              cache_frame_data=False)
    try:
        plt.show()
    finally:
        del animation
        executor.shutdown()
        hub.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
