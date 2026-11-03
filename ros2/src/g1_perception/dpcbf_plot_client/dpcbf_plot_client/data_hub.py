"""DataHub: the rclpy side of the plotting client — subscriptions only.

Owns every ROS callback and a lock-guarded store of ring buffers + latest
states. GUI backends NEVER touch rclpy; they call snapshot() from their own
timer and draw whatever they get. Callbacks do bounded work (float appends
under the lock) — no drawing, no allocation-heavy processing.

Subscriptions (ALL BestEffort: a best-effort request is compatible with both
reliable and best-effort offers, so this matches every publisher in the
stack and can never make one of them wait for us):

  /dpcbf/plot        dpcbf_viz_msgs/DpcbfPlotSample   the control-seam sample
  /odom              nav_msgs/Odometry                DLIO / sim bridge pose
  /obstacles_safe    obstacle_detector/Obstacles      OPTIONAL — only when the
                                                      message package is
                                                      installed on this
                                                      machine; the client is
                                                      fully functional
                                                      without it (the plot
                                                      sample carries the
                                                      filter-selected set).

Time axes: every series is keyed on LOCAL receive time (time.monotonic()),
which needs no clock sync between computers. Header stamps are kept only to
derive a wall-clock latency estimate, labeled as such (valid when both
machines run NTP/chrony).
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy)

from dpcbf_viz_msgs.msg import DpcbfPlotSample
from nav_msgs.msg import Odometry

from .ring_buffer import TimeSeriesBuffer, TrailBuffer

try:  # optional: Computer 3 need not build the obstacle_detector package
    from obstacle_detector.msg import Obstacles as _ObstaclesMsg
except ImportError:  # pragma: no cover - exercised on minimal installs
    _ObstaclesMsg = None

STALENESS_NAMES = {0: 'FRESH', 1: 'DEGRADE', 2: 'STOP', 3: 'NO DATA'}
MODE_NAMES = {0: 'oracle', 1: 'shadow', 2: 'estimated'}

SERIES = (
    'nominal_sagittal', 'safe_sagittal',
    'nominal_lateral', 'safe_lateral',
    'nominal_yaw', 'safe_yaw',
    'intervention', 'command_scale',
    'min_h', 'min_clearance',
    'obstacle_age', 'latency',
)


def _best_effort(depth: int) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth)


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def _quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DataHub(Node):
    def __init__(self):
        super().__init__('dpcbf_plot_client')
        self.plot_topic = self.declare_parameter(
            'plot_topic', '/dpcbf/plot').value
        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value
        self.obstacles_topic = self.declare_parameter(
            'obstacles_topic', '/obstacles_safe').value
        self.window_s = float(self.declare_parameter('window_s', 30.0).value)
        self.stale_after_s = float(
            self.declare_parameter('stale_after_s', 1.0).value)

        self._lock = threading.Lock()
        self._series = {k: TimeSeriesBuffer(self.window_s) for k in SERIES}
        self._trail = TrailBuffer(self.window_s)
        self._latest_sample = None      # last DpcbfPlotSample (msg object)
        self._latest_odom = None        # dict(x, y, yaw, vx, vy, wz)
        self._latest_obstacles = None   # list of dicts from /obstacles_safe
        self._rx_monotonic = {'plot': None, 'odom': None, 'obstacles': None}
        self._counts = {'plot': 0, 'odom': 0, 'obstacles': 0}

        self.create_subscription(
            DpcbfPlotSample, self.plot_topic, self._on_plot, _best_effort(5))
        self.create_subscription(
            Odometry, self.odom_topic, self._on_odom, _best_effort(10))
        self.obstacles_available = _ObstaclesMsg is not None
        if self.obstacles_available:
            self.create_subscription(
                _ObstaclesMsg, self.obstacles_topic, self._on_obstacles,
                _best_effort(5))
        else:
            self.get_logger().info(
                'obstacle_detector msgs not installed - %s subscription '
                'disabled (plot sample obstacles still shown)'
                % self.obstacles_topic)

    # ---- callbacks (bounded work, no GUI) --------------------------------

    def _on_plot(self, msg: DpcbfPlotSample):
        t = time.monotonic()
        latency = time.time() - _stamp_to_sec(msg.header.stamp)
        with self._lock:
            self._latest_sample = msg
            self._rx_monotonic['plot'] = t
            self._counts['plot'] += 1
            s = self._series
            s['nominal_sagittal'].append(t, msg.nominal.sagittal)
            s['safe_sagittal'].append(t, msg.safe.sagittal)
            s['nominal_lateral'].append(t, msg.nominal.lateral)
            s['safe_lateral'].append(t, msg.safe.lateral)
            s['nominal_yaw'].append(t, msg.nominal.yaw_rate)
            s['safe_yaw'].append(t, msg.safe.yaw_rate)
            s['intervention'].append(t, 1.0 if msg.intervention else 0.0)
            s['command_scale'].append(t, msg.command_scale)
            if msg.min_h_valid:
                s['min_h'].append(t, msg.min_h)
                s['min_clearance'].append(t, msg.min_clearance)
            if msg.obstacle_age_s >= 0.0:
                s['obstacle_age'].append(t, msg.obstacle_age_s)
            s['latency'].append(t, latency)

    def _on_odom(self, msg: Odometry):
        t = time.monotonic()
        p = msg.pose.pose
        tw = msg.twist.twist
        with self._lock:
            self._latest_odom = {
                'x': p.position.x, 'y': p.position.y,
                'yaw': _quat_to_yaw(p.orientation),
                'vx': tw.linear.x, 'vy': tw.linear.y, 'wz': tw.angular.z,
                'stamp': _stamp_to_sec(msg.header.stamp),
            }
            self._rx_monotonic['odom'] = t
            self._counts['odom'] += 1
            self._trail.append(t, p.position.x, p.position.y)

    def _on_obstacles(self, msg):
        t = time.monotonic()
        circles = [{
            'id': int(c.uid), 'x': c.center.x, 'y': c.center.y,
            'radius': c.radius,
            'true_radius': c.true_radius,
            'vx': c.velocity.x, 'vy': c.velocity.y,
        } for c in msg.circles]
        with self._lock:
            self._latest_obstacles = circles
            self._rx_monotonic['obstacles'] = t
            self._counts['obstacles'] += 1

    # ---- GUI-side API -----------------------------------------------------

    def snapshot(self) -> dict:
        """Copy of everything a frame needs, taken under the lock.

        ages[src] is seconds since last receive on the LOCAL monotonic
        clock (None = never received); the GUI compares against
        stale_after_s. Series come back as ([t..], [v..]) list pairs with t
        already shifted so 0 = now (monotonic), i.e. plot x-axes are
        "seconds ago".
        """
        now = time.monotonic()
        with self._lock:
            series = {}
            for key, buf in self._series.items():
                ts, vs = buf.arrays()
                series[key] = ([t - now for t in ts], vs)
            trail = self._trail.arrays()
            sample = self._latest_sample
            odom = dict(self._latest_odom) if self._latest_odom else None
            obstacles = (list(self._latest_obstacles)
                         if self._latest_obstacles is not None else None)
            ages = {
                src: (None if t0 is None else now - t0)
                for src, t0 in self._rx_monotonic.items()
            }
            counts = dict(self._counts)
        return {
            'series': series,
            'trail': trail,
            'sample': sample,
            'odom': odom,
            'obstacles': obstacles,
            'ages': ages,
            'counts': counts,
            'stale_after_s': self.stale_after_s,
            'obstacles_available': self.obstacles_available,
        }


class HubRunner:
    """Owns rclpy init + a background executor thread around one DataHub."""

    def __init__(self, args=None):
        rclpy.init(args=args)
        self.hub = DataHub()
        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self.hub)
        self._thread = threading.Thread(
            target=self._spin, name='dpcbf_plot_spin', daemon=True)
        self._stop = threading.Event()

    def _spin(self):
        while not self._stop.is_set() and rclpy.ok():
            self._executor.spin_once(timeout_sec=0.1)

    def start(self):
        self._thread.start()
        return self.hub

    def shutdown(self):
        self._stop.set()
        self._thread.join(timeout=2.0)
        self.hub.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
