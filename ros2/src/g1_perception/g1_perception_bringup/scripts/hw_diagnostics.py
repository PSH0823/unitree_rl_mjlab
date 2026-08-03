#!/usr/bin/python3
"""Perception-stack diagnostics summary (Phase 5C, §5.6 of the 5C brief).

Subscribes to the whole hardware chain and publishes one
`diagnostic_msgs/DiagnosticArray` on `/diagnostics` at 1 Hz. It publishes
NOTHING ELSE — no command topic, no TF, no obstacle output. It cannot move
the robot.

The design rule it exists to enforce: **a failure must not be hidden merely
because the topic exists.** Every row therefore reports a measured RATE and
a measured AGE, and goes ERROR on "no data" rather than staying silent.

Rows
  lidar / imu / odom / cloud_to_scan / tracked / safe   rate + age + count
  tf                                                    stamped-lookup success
  dlio                                                  initialised? (odom
                                                        stamps stop being 0)
  self_hit                                              returns still inside
                                                        the self-radius AFTER
                                                        CropBox
  floor_artifact                                        near-field /scan ring
                                                        occupancy (heuristic)
  timestamp_domain                                      host vs device clock

Heuristic rows are labelled `heuristic` in their own key/value pairs. They
are tripwires that tell an operator where to look; they are not detectors and
no gate should be written against them.

Usage
  ros2 run g1_perception_bringup hw_diagnostics.py
  ros2 topic echo /diagnostics
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy                                                    # noqa: E402
import tf2_ros                                                  # noqa: E402
from diagnostic_msgs.msg import (DiagnosticArray,               # noqa: E402
                                 DiagnosticStatus, KeyValue)
from nav_msgs.msg import Odometry                               # noqa: E402
from rclpy.duration import Duration                             # noqa: E402
from rclpy.node import Node                                     # noqa: E402
from rclpy.qos import QoSProfile, qos_profile_sensor_data       # noqa: E402
from rclpy.time import Time                                     # noqa: E402
from sensor_msgs.msg import Imu, LaserScan, PointCloud2         # noqa: E402

import hw_probe_core as core                                    # noqa: E402

OK, WARN, ERROR = (DiagnosticStatus.OK, DiagnosticStatus.WARN,
                   DiagnosticStatus.ERROR)
# DiagnosticStatus.level is a `byte` field, so rclpy wants b'\x00'..b'\x02',
# while hw_probe_core speaks plain ints (it must stay ROS-free to be
# testable). This is the one place the two representations meet.
_LEVEL_BYTE = {core.LEVEL_OK: OK, core.LEVEL_WARN: WARN,
               core.LEVEL_ERROR: ERROR}


class Track:
    """Rolling rate/age for one topic, over a fixed wall-clock window."""

    def __init__(self, name, expect_hz, window=5.0):
        self.name = name
        self.expect_hz = expect_hz
        self.window = window
        self.recv = []
        self.ages = []
        self.total = 0
        self.last_stamp = None
        self.zero_stamps = 0

    def hit(self, stamp_s, now_s):
        self.total += 1
        self.recv.append(now_s)
        if stamp_s == 0.0:
            self.zero_stamps += 1
        else:
            self.ages.append(now_s - stamp_s)
            self.last_stamp = stamp_s
        cut = now_s - self.window
        while self.recv and self.recv[0] < cut:
            self.recv.pop(0)
        del self.ages[:-64]

    def rate(self, now_s):
        recent = [t for t in self.recv if t >= now_s - self.window]
        return len(recent) / self.window

    def age(self, now_s):
        return (now_s - self.last_stamp) if self.last_stamp else None


class HwDiagnostics(Node):
    def __init__(self):
        super().__init__('hw_diagnostics')
        p = self.declare_parameter
        self.period = float(p('period', 1.0).value)
        self.max_age = float(p('max_age', 0.30).value)      # §10.3 staleness
        self.self_radius = float(p('self_radius', 0.35).value)
        self.self_hit_points = int(p('self_hit_points', 20).value)
        self.floor_ring_m = float(p('floor_ring_m', 0.6).value)
        self.floor_ring_frac = float(p('floor_ring_frac', 0.25).value)
        self.tf_pairs = p('tf_pairs',
                          ['mid360_link:base_footprint',
                           'mid360_link:odom']).value

        self.t = {
            'lidar': Track('/livox/lidar', 10.0),
            'imu': Track('/livox/imu', 200.0),
            'odom': Track('/odom', 100.0),
            'self_filtered': Track('/points_self_filtered', 10.0),
            'scan': Track('/scan', 10.0),
            'raw': Track('/raw_obstacles', 10.0),
            'tracked': Track('/tracked_obstacles', 10.0),
            'safe': Track('/obstacles_safe', 10.0),
        }
        self.tf_ok = 0
        self.tf_try = 0
        self.tf_last_err = ''
        self.self_hit_count = None
        self.floor_frac = None
        self.obstacle_count = None
        self.cloud_stride = 0

        self.buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.listener = tf2_ros.TransformListener(self.buffer, self,
                                                  spin_thread=False)

        sd = qos_profile_sensor_data
        self.create_subscription(PointCloud2, '/livox/lidar',
                                 self._cb('lidar'), sd)
        self.create_subscription(Imu, '/livox/imu', self._cb('imu'), sd)
        self.create_subscription(Odometry, '/odom', self._cb('odom'),
                                 QoSProfile(depth=10))
        self.create_subscription(PointCloud2, '/points_self_filtered',
                                 self._on_self_filtered, sd)
        self.create_subscription(LaserScan, '/scan', self._on_scan, sd)
        self._sub_obstacles()

        self.pub = self.create_publisher(DiagnosticArray, '/diagnostics',
                                         QoSProfile(depth=10))
        self.create_timer(self.period, self._publish)
        self.get_logger().info('hw_diagnostics: /diagnostics at '
                               f'{1.0 / self.period:.1f} Hz; no other '
                               'publisher exists in this process')

    def _sub_obstacles(self):
        """The fork's messages are only importable once obstacle_detector is
        built. Degrade to three ERROR rows rather than refusing to start —
        an operator debugging a broken workspace still wants the sensor rows."""
        try:
            from obstacle_detector.msg import Obstacles
        except Exception as exc:
            self.get_logger().warn(
                f'obstacle_detector.msg unavailable ({exc}); the obstacle '
                'rows will report no data')
            return
        rel = QoSProfile(depth=5)
        self.create_subscription(Obstacles, '/raw_obstacles',
                                 self._cb('raw'), rel)
        self.create_subscription(Obstacles, '/tracked_obstacles',
                                 self._cb('tracked'), rel)
        self.create_subscription(Obstacles, '/obstacles_safe',
                                 self._on_safe, QoSProfile(depth=1))

    # --- callbacks ---------------------------------------------------------
    def _cb(self, key):
        def inner(msg):
            self.t[key].hit(_stamp(msg), time.time())
        return inner

    def _on_safe(self, msg):
        self.t['safe'].hit(_stamp(msg), time.time())
        self.obstacle_count = len(msg.circles)

    def _on_scan(self, msg):
        now = time.time()
        self.t['scan'].hit(_stamp(msg), now)
        import math
        finite = [r for r in msg.ranges
                  if r == r and math.isfinite(r) and r >= msg.range_min]
        if finite:
            near = sum(1 for r in finite if r < self.floor_ring_m)
            self.floor_frac = near / len(finite)

    def _on_self_filtered(self, msg):
        now = time.time()
        self.t['self_filtered'].hit(_stamp(msg), now)
        self.cloud_stride += 1
        if self.cloud_stride % 5:            # 2 Hz is plenty for a tripwire
            return
        try:
            import numpy as np
            from sensor_msgs_py import point_cloud2
            a = point_cloud2.read_points(msg, field_names=('x', 'y'),
                                         skip_nans=True)
            x = np.asarray(a['x'], dtype=float)
            y = np.asarray(a['y'], dtype=float)
            self.self_hit_count = int(
                ((x * x + y * y) < self.self_radius ** 2).sum())
        except Exception:
            self.self_hit_count = None

    # --- report ------------------------------------------------------------
    def _publish(self):
        now = time.time()
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()

        for key, label in (('lidar', 'lidar'), ('imu', 'imu'),
                           ('odom', 'odom'),
                           ('self_filtered', 'cloud_self_filtered'),
                           ('scan', 'cloud_to_scan'),
                           ('raw', 'raw_obstacles'),
                           ('tracked', 'tracked_obstacles'),
                           ('safe', 'obstacles_safe')):
            arr.status.append(self._rate_row(label, self.t[key], now))

        arr.status.append(self._tf_row(now))
        arr.status.append(self._dlio_row())
        arr.status.append(self._self_hit_row())
        arr.status.append(self._floor_row())
        arr.status.append(self._clock_row(now))
        self.pub.publish(arr)
        self._probe_tf()

    def _rate_row(self, label, tr, now):
        st = DiagnosticStatus(name=f'perception/{label}',
                              hardware_id='g1_mid360')
        rate = tr.rate(now)
        age = tr.age(now)
        lvl, st.message = core.rate_level(tr.total, rate, tr.expect_hz,
                                          age, self.max_age)
        st.level = _LEVEL_BYTE[lvl]
        if not tr.total:
            st.message = f'NO DATA on {tr.name}'
        st.values = [
            KeyValue(key='topic', value=tr.name),
            KeyValue(key='rate_hz', value=f'{rate:.2f}'),
            KeyValue(key='expect_hz', value=f'{tr.expect_hz:.0f}'),
            KeyValue(key='age_s', value='n/a' if age is None else f'{age:.3f}'),
            KeyValue(key='count', value=str(tr.total)),
            KeyValue(key='zero_stamps', value=str(tr.zero_stamps)),
        ]
        if tr.name == '/obstacles_safe' and self.obstacle_count is not None:
            st.values.append(KeyValue(key='obstacle_count',
                                      value=str(self.obstacle_count)))
        return st

    def _probe_tf(self):
        """One stamped lookup per cycle at the newest LiDAR stamp. Cheap, and
        it asks the same question the projection's MessageFilter asks."""
        stamp = self.t['lidar'].last_stamp
        if stamp is None:
            return
        t = Time(seconds=int(stamp), nanoseconds=int((stamp % 1) * 1e9))
        for pair in self.tf_pairs:
            src, dst = pair.split(':')
            self.tf_try += 1
            try:
                self.buffer.lookup_transform(dst, src, t,
                                             timeout=Duration(seconds=0.02))
                self.tf_ok += 1
            except Exception as exc:
                self.tf_last_err = str(exc)[:200]

    def _tf_row(self, now):
        st = DiagnosticStatus(name='perception/tf', hardware_id='g1_mid360')
        frac = (self.tf_ok / self.tf_try) if self.tf_try else None
        if frac is None:
            st.level, st.message = ERROR, 'no stamped lookup attempted yet'
        elif frac < 0.9:
            st.level = ERROR
            st.message = f'stamped TF available {frac * 100:.0f} % of cycles'
        elif frac < 0.99:
            st.level = WARN
            st.message = f'stamped TF available {frac * 100:.0f} % of cycles'
        else:
            st.level, st.message = OK, 'stamped TF available'
        st.values = [
            KeyValue(key='pairs', value=','.join(self.tf_pairs)),
            KeyValue(key='lookups', value=str(self.tf_try)),
            KeyValue(key='ok', value=str(self.tf_ok)),
            KeyValue(key='last_error', value=self.tf_last_err),
            KeyValue(key='note',
                     value='lookup uses the LiDAR header stamp, not latest'),
        ]
        return st

    def _dlio_row(self):
        """DLIO stamps /odom with imu_stamp, which is 0 until its first IMU
        message; and it calibrates IMU bias over odom/imu/calibration/time
        (3 s) with the robot required to be still. `initialised` here means
        "stamps have stopped being 0", not "the map is good"."""
        st = DiagnosticStatus(name='perception/dlio', hardware_id='g1_mid360')
        tr = self.t['odom']
        if tr.total == 0:
            st.level, st.message = ERROR, 'no /odom — DLIO not running'
        elif tr.last_stamp is None:
            st.level = WARN
            st.message = ('/odom arriving with stamp 0 — DLIO has not seen an '
                          'IMU message yet')
        else:
            st.level, st.message = OK, 'publishing stamped odometry'
        st.values = [
            KeyValue(key='odom_count', value=str(tr.total)),
            KeyValue(key='zero_stamped', value=str(tr.zero_stamps)),
            KeyValue(key='calibration_state',
                     value='not exposed by DLIO; see its stdout'),
        ]
        return st

    def _self_hit_row(self):
        st = DiagnosticStatus(name='perception/self_hit',
                              hardware_id='g1_mid360')
        n = self.self_hit_count
        if self.t['self_filtered'].total == 0:
            st.level, st.message = ERROR, 'no /points_self_filtered'
        elif n is None:
            st.level, st.message = WARN, 'cloud not readable for the check'
        elif n > self.self_hit_points:
            st.level = WARN
            st.message = (f'{n} points still inside r<{self.self_radius:.2f} m '
                          'AFTER CropBox — likely robot-body returns')
        else:
            st.level, st.message = OK, f'{n} near-field points after CropBox'
        st.values = [
            KeyValue(key='points_inside_self_radius',
                     value='n/a' if n is None else str(n)),
            KeyValue(key='self_radius_m', value=f'{self.self_radius:.2f}'),
            KeyValue(key='heuristic',
                     value='true; a real near obstacle also lands here'),
        ]
        return st

    def _floor_row(self):
        st = DiagnosticStatus(name='perception/floor_artifact',
                              hardware_id='g1_mid360')
        f = self.floor_frac
        if self.t['scan'].total == 0:
            st.level, st.message = ERROR, 'no /scan'
        elif f is None:
            st.level, st.message = WARN, 'no finite scan bins'
        elif f > self.floor_ring_frac:
            st.level = WARN
            st.message = (f'{f * 100:.0f} % of finite scan bins are inside '
                          f'{self.floor_ring_m:.2f} m — floor ring or '
                          'self-return leaking into the height band')
        else:
            st.level, st.message = OK, f'{f * 100:.0f} % near-field bins'
        st.values = [
            KeyValue(key='near_bin_fraction',
                     value='n/a' if f is None else f'{f:.3f}'),
            KeyValue(key='ring_radius_m', value=f'{self.floor_ring_m:.2f}'),
            KeyValue(key='heuristic',
                     value='true; there is NO ground segmentation in this '
                           'stack — min_height only'),
        ]
        return st

    def _clock_row(self, now):
        st = DiagnosticStatus(name='perception/timestamp_domain',
                              hardware_id='g1_mid360')
        ages = {k: self.t[k].age(now) for k in ('lidar', 'imu', 'odom')}
        live = {k: v for k, v in ages.items() if v is not None}
        if not live:
            st.level, st.message = ERROR, 'no stamped source messages'
        elif max(live.values()) > 1.0:
            st.level = ERROR
            st.message = ('source stamps are >1 s from this host clock — the '
                          'LiDAR is likely on its own/PTP clock while ROS is '
                          'on the host clock (§14.3)')
        elif max(live.values()) > self.max_age:
            st.level = WARN
            st.message = f'newest source stamp {max(live.values()):.2f} s old'
        else:
            st.level, st.message = OK, 'source stamps within one host-clock ' \
                                       'staleness budget'
        st.values = [KeyValue(key=f'{k}_age_s', value=f'{v:.3f}')
                     for k, v in live.items()]
        st.values.append(KeyValue(
            key='note', value='host-vs-device clock is a MEASUREMENT (see '
                              'hw_source_probe.py), not a configuration'))
        return st


def _stamp(msg):
    h = getattr(msg, 'header', None)
    if h is None:
        return 0.0
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def main():
    rclpy.init(args=sys.argv)
    node = HwDiagnostics()
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
