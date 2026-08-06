"""Synthetic /dpcbf/plot + /odom (+ /obstacles_safe) source.

Verifies the whole visualization path — message package, QoS match, DDS
transport, client rendering, stale handling — with NO robot, NO simulator
and NO perception stack:

    ros2 run dpcbf_plot_client synthetic_dpcbf_publisher
    ros2 run dpcbf_plot_client synthetic_dpcbf_publisher --ros-args \
        -p rate_hz:=30.0 -p stop_after_s:=10.0

Scenario (deterministic, parameter-free): the robot walks a slow circle; one
obstacle is static ahead of the start pose, one crosses the robot's path
every ~12 s. While the crossing obstacle is near, the sample shows the
DPCBF "intervening": safe < scaled, intervention true, min_h dipping below
zero at the closest approach. Numbers are shaped to LOOK right on a plot;
nothing here runs the real QP, and nothing here may ever be wired toward a
robot — this node exists for bench verification of the plotting path only.

stop_after_s > 0 makes the node stop publishing (but stay alive) after that
many seconds — the client must flip to STALE; that transition is part of the
verification checklist.

QoS mirrors the real publishers: /dpcbf/plot BestEffort KeepLast(5),
/odom Reliable KeepLast(10), /obstacles_safe Reliable KeepLast(1).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from dpcbf_viz_msgs.msg import DpcbfPlotSample, PlotObstacle
from nav_msgs.msg import Odometry

try:
    from obstacle_detector.msg import CircleObstacle, Obstacles
except ImportError:  # minimal install: plot sample obstacles only
    Obstacles = None


class SyntheticDpcbfPublisher(Node):
    def __init__(self):
        super().__init__('synthetic_dpcbf_publisher')
        self.rate_hz = float(self.declare_parameter('rate_hz', 30.0).value)
        self.odom_rate_hz = float(
            self.declare_parameter('odom_rate_hz', 50.0).value)
        self.stop_after_s = float(
            self.declare_parameter('stop_after_s', 0.0).value)
        self.plot_topic = self.declare_parameter(
            'plot_topic', '/dpcbf/plot').value

        best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        self.pub_plot = self.create_publisher(
            DpcbfPlotSample, self.plot_topic, best_effort)
        self.pub_odom = self.create_publisher(
            Odometry, '/odom', QoSProfile(depth=10))
        self.pub_obs = None
        if Obstacles is not None:
            self.pub_obs = self.create_publisher(
                Obstacles, '/obstacles_safe',
                QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                           history=QoSHistoryPolicy.KEEP_LAST, depth=1))

        self.t0 = self.get_clock().now()
        self.tick = 0
        self.create_timer(1.0 / self.rate_hz, self._publish_plot)
        self.create_timer(1.0 / self.odom_rate_hz, self._publish_odom)
        if self.pub_obs is not None:
            self.create_timer(0.1, self._publish_obstacles)
        self.get_logger().info(
            f'synthetic source: {self.plot_topic} @ {self.rate_hz:.0f} Hz, '
            f'/odom @ {self.odom_rate_hz:.0f} Hz, /obstacles_safe '
            f"{'@ 10 Hz' if self.pub_obs else 'unavailable'}"
            + (f', stops after {self.stop_after_s:.0f}s'
               if self.stop_after_s > 0 else ''))

    # ---- scenario ---------------------------------------------------------

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self.t0).nanoseconds * 1e-9

    def _stopped(self) -> bool:
        return 0.0 < self.stop_after_s < self._elapsed()

    def _robot(self, t):
        # slow circle, 4 m radius, one lap every ~84 s
        w = 0.075
        x = 4.0 * math.cos(w * t) - 4.0
        y = 4.0 * math.sin(w * t)
        yaw = w * t + math.pi / 2.0
        return x, y, yaw

    def _obstacles(self, t):
        rx, ry, _ = self._robot(t)
        static = {'id': 1, 'x': 2.0, 'y': 1.5, 'r': 0.3, 'vx': 0.0,
                  'vy': 0.0}
        # crossing obstacle: shuttles through the robot's neighborhood
        phase = (t % 12.0) / 12.0
        cx = rx + 3.0 - 6.0 * phase
        cy = ry + 1.0
        crossing = {'id': 2, 'x': cx, 'y': cy, 'r': 0.35,
                    'vx': -0.5, 'vy': 0.0}
        return [static, crossing]

    # ---- publishers -------------------------------------------------------

    def _publish_plot(self):
        if self._stopped():
            return
        t = self._elapsed()
        self.tick += 1
        x, y, yaw = self._robot(t)
        obstacles = self._obstacles(t)

        msg = DpcbfPlotSample()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.tick = self.tick
        msg.t_ctrl = t
        msg.mode = DpcbfPlotSample.MODE_ESTIMATED
        msg.robot_x = x
        msg.robot_y = y
        msg.robot_phi = yaw
        msg.robot_sagittal_velocity = 0.3
        msg.robot_lateral_velocity = 0.0

        msg.nominal.sagittal = 0.5 + 0.1 * math.sin(0.4 * t)
        msg.nominal.lateral = 0.0
        msg.nominal.yaw_rate = 0.075
        msg.command_scale = 1.0
        msg.scaled = msg.nominal

        # "intervention" when the crossing obstacle is close
        dists = [math.hypot(o['x'] - x, o['y'] - y) - o['r']
                 for o in obstacles]
        min_clear = min(dists)
        intervening = min_clear < 1.5
        msg.safe.sagittal = (msg.scaled.sagittal * 0.35 if intervening
                             else msg.scaled.sagittal)
        msg.safe.lateral = -0.15 if intervening else 0.0
        msg.safe.yaw_rate = msg.scaled.yaw_rate
        msg.intervention = intervening
        msg.solved = True
        msg.active_constraints = 2 if intervening else 0
        msg.active_dpcbf_constraints = 1 if intervening else 0
        msg.active_ecbf_constraints = 1 if intervening else 0
        msg.acceleration = [0.0, 0.0, 0.0]

        msg.staleness_state = DpcbfPlotSample.STALENESS_FRESH
        msg.obstacle_age_s = 0.04 + 0.02 * math.sin(t)
        msg.obstacle_total = len(obstacles)
        for o, clear in zip(obstacles, dists):
            po = PlotObstacle()
            po.id = o['id']
            po.x = o['x']
            po.y = o['y']
            po.radius = o['r']
            po.velocity_x = o['vx']
            po.velocity_y = o['vy']
            po.distance = clear + o['r']
            po.h = clear - 0.8       # dips below 0 at closest approach
            msg.obstacles.append(po)
        msg.min_h_valid = True
        msg.min_h = min(o.h for o in msg.obstacles)
        msg.min_clearance = min_clear
        self.pub_plot.publish(msg)

    def _publish_odom(self):
        if self._stopped():
            return
        t = self._elapsed()
        x, y, yaw = self._robot(t)
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.twist.twist.linear.x = 0.3
        msg.twist.twist.angular.z = 0.075
        self.pub_odom.publish(msg)

    def _publish_obstacles(self):
        if self._stopped():
            return
        t = self._elapsed()
        msg = Obstacles()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        for o in self._obstacles(t):
            c = CircleObstacle()
            c.uid = o['id']
            c.center.x = o['x']
            c.center.y = o['y']
            c.radius = o['r'] + 0.1        # inflated, like the real stream
            c.true_radius = o['r']
            c.velocity.x = o['vx']
            c.velocity.y = o['vy']
            msg.circles.append(c)
        self.pub_obs.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticDpcbfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
