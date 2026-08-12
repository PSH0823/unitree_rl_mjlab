"""Robot-fixed top-down view: the laser scan under the fitted circles.

    ros2 run dpcbf_plot_client dpcbf_scan_view
    ros2 run dpcbf_plot_client dpcbf_scan_view --ros-args \
        -p range:=3.0 -p scan_history:=5 \
        -p obstacle_topics:="['/raw_obstacles','/obstacles_safe']"

WHY THIS EXISTS, given dpcbf_plot_client already draws obstacles.

  * That view is drawn in ODOM and autoscales. Everything the robot does —
    sway on a stand, DLIO drift, a re-localisation step — moves the whole
    picture, and the axes rescale under it. A static prop looks like it is
    swimming even when perception is perfectly steady. This view pins the
    frame to the ROBOT and the axes to a fixed range, so the only motion left
    on screen is motion perception actually reported.
  * It shows the /scan the circles were fitted FROM. Without the underlying
    points, "the estimate is noisy" cannot be separated from "the scan is
    noisy" — and those have completely different fixes (§9.5 extractor
    parameters vs §9.3/§9.4 crop and projection). With scan_history > 1 the
    last N scans are overdrawn fainter, so scan jitter is visible directly.

FRAMES. /scan is already robot-fixed: pointcloud_to_laserscan runs with
target_frame=base_footprint (§9.4). The obstacle topics are odom-frame
(§7.1) — the extractor transforms them there. So the circles are transformed
BACK, at the message stamp rather than `latest`, exactly as
hw_obstacle_watch.py does it. target_frame:='' (the default) means "whatever
frame /scan arrives in"; set it explicitly to compare against odom.

Requires /tf and /tf_static to reach this machine. If they do not, the scan
still draws and the banner says the circles are missing — that is a link
symptom, not a GUI one (see the field manual §6-1).

Publishes nothing. Subscribes only; it cannot move the robot.
"""
import math
import sys
import threading
import time
from collections import deque

import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

try:  # same optionality as data_hub: the client stays useful without it
    from obstacle_detector.msg import Obstacles as _ObstaclesMsg
except ImportError:  # pragma: no cover - exercised on minimal installs
    _ObstaclesMsg = None

# Distinct per-stage styling. The order matters: it is the pipeline order, so
# a discrepancy reads left-to-right along the chain (§9.5 -> §9.6).
STYLES = {
    '/raw_obstacles':     dict(color='#8c8c8c', lw=1.0, ls=':',  label='raw'),
    '/tracked_obstacles': dict(color='#ffaa3c', lw=1.5, ls='-',  label='tracked'),
    '/obstacles_safe':    dict(color='#ff3c3c', lw=2.0, ls='-',  label='safe'),
}
_FALLBACK_STYLE = dict(color='#78b4ff', lw=1.5, ls='-', label=None)


def _best_effort(depth=1):
    # A best-effort request is compatible with both reliable and best-effort
    # offers, so this matches every publisher in the stack and can never make
    # one of them wait for a laptop on Wi-Fi.
    return QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                      history=QoSHistoryPolicy.KEEP_LAST, depth=depth)


def _yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ScanViewHub(Node):
    """Subscriptions + a lock-guarded latest-state store. No drawing here."""

    def __init__(self):
        super().__init__('dpcbf_scan_view')
        self.scan_topic = self.declare_parameter('scan_topic', '/scan').value
        self.obstacle_topics = list(self.declare_parameter(
            'obstacle_topics',
            ['/raw_obstacles', '/tracked_obstacles', '/obstacles_safe']).value)
        self.target_frame = self.declare_parameter('target_frame', '').value
        self.tf_timeout = float(
            self.declare_parameter('tf_timeout', 0.05).value)
        self.scan_history = max(
            1, int(self.declare_parameter('scan_history', 1).value))
        self.stale_after_s = float(
            self.declare_parameter('stale_after_s', 1.0).value)

        self._lock = threading.Lock()
        self._scans = deque(maxlen=self.scan_history)   # newest last
        self._scan_frame = None
        self._obstacles = {t: None for t in self.obstacle_topics}
        self._rx = {t: None for t in self.obstacle_topics}
        self._rx['scan'] = None
        self._tf_note = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(LaserScan, self.scan_topic,
                                 self._on_scan, _best_effort())
        self.obstacles_available = _ObstaclesMsg is not None
        if self.obstacles_available:
            for topic in self.obstacle_topics:
                self.create_subscription(
                    _ObstaclesMsg, topic,
                    lambda msg, t=topic: self._on_obstacles(msg, t),
                    _best_effort())
        else:
            self.get_logger().warning(
                'obstacle_detector msgs not installed on this machine: '
                'circles disabled, scan only. Build obstacle_detector (§3-2).')

    # ------------------------------------------------------------------
    def _on_scan(self, msg: LaserScan):
        pts = []
        angle = msg.angle_min
        rmin, rmax = msg.range_min, msg.range_max
        for r in msg.ranges:
            if rmin <= r <= rmax and math.isfinite(r):
                pts.append((r * math.cos(angle), r * math.sin(angle)))
            angle += msg.angle_increment
        with self._lock:
            self._scan_frame = msg.header.frame_id
            self._scans.append(pts)
            self._rx['scan'] = time.monotonic()

    def _on_obstacles(self, msg, topic):
        with self._lock:
            self._obstacles[topic] = msg
            self._rx[topic] = time.monotonic()

    # ------------------------------------------------------------------
    def _transform_2d(self, source_frame, target_frame, stamp):
        """(dx, dy, yaw) taking a point in source_frame to target_frame.

        Looked up AT THE MESSAGE STAMP: with the robot swaying on a stand,
        `latest` would pair a 100 ms-old obstacle with the current pose and
        smear exactly the jitter this view exists to rule out. Falls back to
        latest only so a clock skew between the two computers degrades to a
        slightly wrong picture instead of a blank one — and says so.
        """
        if not source_frame or source_frame == target_frame:
            return (0.0, 0.0, 0.0), None
        timeout = Duration(seconds=self.tf_timeout)
        try:
            tr = self._tf_buffer.lookup_transform(
                target_frame, source_frame, Time.from_msg(stamp), timeout)
            note = None
        except tf2_ros.TransformException as exc:
            try:
                tr = self._tf_buffer.lookup_transform(
                    target_frame, source_frame, Time(), timeout)
                note = 'tf at latest (stamp lookup failed)'
            except tf2_ros.TransformException:
                return None, f'no tf {source_frame}->{target_frame}: ' \
                             f'{type(exc).__name__}'
        t = tr.transform.translation
        return (t.x, t.y, _yaw_of(tr.transform.rotation)), note

    def snapshot(self):
        """Everything the renderer needs, already in the target frame."""
        with self._lock:
            scans = list(self._scans)
            scan_frame = self._scan_frame
            msgs = {t: self._obstacles[t] for t in self.obstacle_topics}
            rx = dict(self._rx)

        target = self.target_frame or scan_frame
        note = None
        circles = {t: [] for t in self.obstacle_topics}

        if target and scan_frame and target != scan_frame:
            tf, note = self._transform_2d(scan_frame, target, rclpy.time.Time().to_msg())
            if tf is None:
                scans = []
            else:
                scans = [[_apply(tf, p) for p in pts] for pts in scans]

        for topic, msg in msgs.items():
            if msg is None or target is None:
                continue
            tf, n = self._transform_2d(msg.header.frame_id, target,
                                       msg.header.stamp)
            note = note or n
            if tf is None:
                continue
            for c in msg.circles:
                x, y = _apply(tf, (c.center.x, c.center.y))
                vx, vy = _rotate(tf[2], (c.velocity.x, c.velocity.y))
                circles[topic].append(
                    dict(uid=c.uid, x=x, y=y, vx=vx, vy=vy,
                         radius=c.radius, true_radius=c.true_radius))

        now = time.monotonic()
        ages = {k: (None if t0 is None else now - t0) for k, t0 in rx.items()}
        return dict(scans=scans, circles=circles, ages=ages, frame=target,
                    scan_frame=scan_frame, note=note,
                    stale_after_s=self.stale_after_s,
                    obstacles_available=self.obstacles_available)


def _rotate(yaw, p):
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * p[0] - s * p[1], s * p[0] + c * p[1])


def _apply(tf, p):
    x, y = _rotate(tf[2], p)
    return (x + tf[0], y + tf[1])


class ScanView:
    """matplotlib window. Fixed axes, fixed frame — deliberately no autoscale."""

    def __init__(self, hub, gui_rate_hz=10.0, half_range=5.0):
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        self._plt = plt
        self.hub = hub
        self.fig, self.ax = plt.subplots(
            num='scan + circle fit (robot frame)', figsize=(9, 9))
        self.ax.set_aspect('equal')
        self.ax.grid(alpha=0.3)
        # THE point of this view: the limits never move, so a circle that
        # wobbles on screen is a circle that wobbled in the estimate.
        self.ax.set_xlim(-half_range, half_range)
        self.ax.set_ylim(-half_range, half_range)
        self.ax.set_xlabel('x [m] — robot forward')
        self.ax.set_ylabel('y [m] — robot left')

        self._scan_lines = []
        for i in range(hub.scan_history):
            newest = i == hub.scan_history - 1
            (ln,) = self.ax.plot(
                [], [], '.', ms=2.5 if newest else 1.5,
                color='#50c8ff' if newest else '#3c6478',
                alpha=1.0 if newest else 0.35,
                label='scan' if newest else None)
            self._scan_lines.append(ln)

        self.ax.plot([0], [0], 'o', ms=9, color='#8cdc8c', zorder=5)
        self.ax.plot([0, 0.4], [0, 0], '-', lw=2, color='#8cdc8c', zorder=5)
        for topic in hub.obstacle_topics:
            st = STYLES.get(topic, _FALLBACK_STYLE)
            self.ax.plot([], [], color=st['color'], lw=st['lw'], ls=st['ls'],
                         label=st['label'] or topic)
        self.ax.legend(loc='upper right', fontsize=8)
        self.banner = self.ax.text(
            0.01, 0.99, '', transform=self.ax.transAxes, va='top', ha='left',
            fontsize=8, family='monospace')
        self._patches = []
        self.frames_rendered = 0
        self._anim = FuncAnimation(self.fig, self._on_frame,
                                   interval=max(50, int(1000.0 / gui_rate_hz)),
                                   cache_frame_data=False)

    def _on_frame(self, _frame):
        from matplotlib.patches import Circle
        snap = self.hub.snapshot()

        scans = snap['scans']
        for i, ln in enumerate(self._scan_lines):
            # newest scan in the LAST artist, so it keeps the bright style
            idx = len(scans) - len(self._scan_lines) + i
            pts = scans[idx] if 0 <= idx < len(scans) else []
            ln.set_data([p[0] for p in pts], [p[1] for p in pts])

        for p in self._patches:
            p.remove()
        self._patches = []
        last_topic = self.hub.obstacle_topics[-1] \
            if self.hub.obstacle_topics else None
        for topic, obs in snap['circles'].items():
            st = STYLES.get(topic, _FALLBACK_STYLE)
            for o in obs:
                c = Circle((o['x'], o['y']), max(o['radius'], 0.02),
                           fill=False, color=st['color'], lw=st['lw'],
                           ls=st['ls'])
                self.ax.add_patch(c)
                self._patches.append(c)
                if topic == last_topic:
                    self._patches.append(self.ax.annotate(
                        f"{o['uid']}  r={o['true_radius']:.2f}",
                        (o['x'], o['y']), fontsize=7, color=st['color'],
                        xytext=(4, 4), textcoords='offset points'))
                if o['vx'] or o['vy']:
                    (v,) = self.ax.plot([o['x'], o['x'] + o['vx']],
                                        [o['y'], o['y'] + o['vy']],
                                        color=st['color'], lw=1.0, ls='--')
                    self._patches.append(v)

        lines = [f"frame: {snap['frame'] or '(waiting for scan)'}"]
        any_stale = False
        for name, key in [('scan', 'scan')] + [(t.lstrip('/'), t)
                                               for t in
                                               self.hub.obstacle_topics]:
            age = snap['ages'].get(key)
            if age is None:
                lines.append(f'{name}: NO DATA')
                any_stale = True
            elif age > snap['stale_after_s']:
                lines.append(f'{name}: STALE {age:.1f}s')
                any_stale = True
            else:
                lines.append(f'{name}: ok {age * 1e3:.0f}ms')
        if not snap['obstacles_available']:
            lines.append('obstacles disabled (obstacle_detector not built)')
        if snap['note']:
            lines.append(snap['note'])
            any_stale = True
        self.banner.set_text('\n'.join(lines))
        self.banner.set_color('#ff5050' if any_stale else '#8cdc8c')
        self.frames_rendered += 1
        return []

    def run(self):
        self._plt.show()
        return 0


def main(args=None):
    rclpy.init(args=args)
    hub = ScanViewHub()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(hub)
    stop = threading.Event()

    def _spin():
        while not stop.is_set() and rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    thread = threading.Thread(target=_spin, name='scan_view_spin', daemon=True)
    thread.start()
    try:
        gui_rate_hz = float(hub.declare_parameter('gui_rate_hz', 10.0).value)
        half_range = float(hub.declare_parameter('range', 5.0).value)
        return ScanView(hub, gui_rate_hz=gui_rate_hz,
                        half_range=half_range).run()
    except KeyboardInterrupt:
        return 0
    finally:
        stop.set()
        thread.join(timeout=2.0)
        hub.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
