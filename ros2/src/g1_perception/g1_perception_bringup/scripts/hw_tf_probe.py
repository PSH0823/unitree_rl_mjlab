#!/usr/bin/python3
"""Non-actuating STAMPED TF availability probe (Phase 5C, stages 4/5/12).

For every /livox/lidar message this asks tf2 for the transforms the pipeline
needs AT THAT MESSAGE'S STAMP — never `Time()`/latest.

That distinction is the whole probe. `ros2 run tf2_ros tf2_echo A B` and a
`TimePointZero` lookup both succeed against a TF tree that stopped updating
minutes ago, and both succeed when odometry publishes once and dies. What
`pointcloud_to_laserscan` actually does is a tf2 MessageFilter at the cloud
stamp; if that fails, /scan is silently empty while every topic in
`ros2 topic list` looks healthy. This probe reproduces the real question.

Transforms checked (§8.2):
    mid360_link -> base_link       static, robot_state_publisher
    base_link   -> base_footprint  base_footprint_publisher (via odom)
    base_footprint -> odom         base_footprint_publisher
    mid360_link -> odom            the composite the projection needs

Publishes nothing but /tf subscriptions. Cannot move the robot.

Usage
  ros2 run g1_perception_bringup hw_tf_probe.py --ros-args \
      -p duration:=30.0 -p json:=/tmp/b_tf_probe.json

Exit 0 clean, 1 problems, 2 no LiDAR data (nothing was ever attempted).
"""
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy                                                    # noqa: E402
import tf2_ros                                                  # noqa: E402
from rclpy.duration import Duration                             # noqa: E402
from rclpy.node import Node                                     # noqa: E402
from rclpy.qos import qos_profile_sensor_data                   # noqa: E402
from rclpy.time import Time                                     # noqa: E402
from sensor_msgs.msg import PointCloud2                         # noqa: E402
from tf2_msgs.msg import TFMessage                              # noqa: E402

import hw_probe_core as core                                    # noqa: E402

DEFAULT_PAIRS = [
    'mid360_link:base_link',
    'base_link:base_footprint',
    'base_footprint:odom',
    'mid360_link:odom',
]


class HwTfProbe(Node):
    def __init__(self):
        super().__init__('hw_tf_probe')
        self.duration = float(self.declare_parameter('duration', 30.0).value)
        self.cloud_topic = self.declare_parameter(
            'cloud_topic', '/livox/lidar').value
        pairs = self.declare_parameter('pairs', DEFAULT_PAIRS).value
        # tf2's own wait. Kept SHORT and reported: a long tolerance turns "the
        # TF is late" into "the TF is fine but the pipeline is slow", which is
        # exactly the confusion the 5A latency finding came out of.
        self.timeout = float(self.declare_parameter('timeout', 0.05).value)
        self.min_fraction = float(
            self.declare_parameter('min_fraction', 0.95).value)
        self.json_path = self.declare_parameter('json', '').value

        self.buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.listener = tf2_ros.TransformListener(self.buffer, self,
                                                  spin_thread=False)
        self.stats = {p: core.TfLookupStats(p) for p in pairs}
        self.clouds = 0
        # TF-side observability, independent of any lookup succeeding.
        self.tf_stamps = {}         # (parent,child) -> core.StampSeries
        self.tf_dupes = 0
        self.oldest_tf = None
        self.newest_tf = None

        self.create_subscription(PointCloud2, self.cloud_topic,
                                 self._on_cloud, qos_profile_sensor_data)
        # /tf itself: RELIABLE depth 100 is the tf2 convention.
        self.create_subscription(TFMessage, '/tf', self._on_tf, 100)
        self.t_start = time.time()
        self.get_logger().info(
            f'stamped-TF probe for {self.duration:.0f} s on '
            f'{sorted(self.stats)} — publishing nothing')

    def _on_tf(self, msg):
        now = time.time()
        for t in msg.transforms:
            key = f'{t.header.frame_id}->{t.child_frame_id}'
            s = self.tf_stamps.get(key)
            if s is None:
                s = self.tf_stamps[key] = core.StampSeries(key)
            stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
            if s.last_stamp is not None and stamp == s.last_stamp:
                self.tf_dupes += 1
            s.add(stamp, now)
            if self.oldest_tf is None or stamp < self.oldest_tf:
                self.oldest_tf = stamp
            if self.newest_tf is None or stamp > self.newest_tf:
                self.newest_tf = stamp

    def _on_cloud(self, msg):
        self.clouds += 1
        # THE stamped lookup. `stamp` comes from the message header; passing
        # Time() here would make this probe worthless.
        stamp = Time.from_msg(msg.header.stamp)
        for pair, st in self.stats.items():
            src, dst = pair.split(':')
            t0 = time.time()
            try:
                self.buffer.lookup_transform(
                    dst, src, stamp,
                    timeout=Duration(seconds=self.timeout))
                st.record_ok(time.time() - t0)
            except Exception as exc:
                st.record_fail(str(exc))

    def report(self):
        problems = []
        for st in self.stats.values():
            problems += st.problems(self.min_fraction)
        if self.clouds == 0:
            problems.append(
                f'{self.cloud_topic}: no messages — the probe never attempted '
                'a lookup. Fix the source before reading anything below.')
        if not self.tf_stamps:
            problems.append('/tf: no dynamic transforms at all — odometry is '
                            'not publishing (robot_state_publisher only emits '
                            '/tf_static)')
        for key, s in self.tf_stamps.items():
            if s.regressions:
                problems.append(f'/tf {key}: {len(s.regressions)} stamp '
                                'regression(s) — tf2 clears its cache on a '
                                'backwards jump')
        return {
            'probe': 'hw_tf_probe',
            'duration_s': core._r(time.time() - self.t_start),
            'clouds_seen': self.clouds,
            'timeout_s': self.timeout,
            'min_fraction': self.min_fraction,
            'lookups': [st.summary() for st in self.stats.values()],
            'tf_broadcasts': [s.summary() for s in self.tf_stamps.values()],
            'tf_duplicate_stamps': self.tf_dupes,
            'tf_oldest_stamp': core._r(self.oldest_tf),
            'tf_newest_stamp': core._r(self.newest_tf),
            'tf_span_s': core._r(
                (self.newest_tf - self.oldest_tf)
                if (self.oldest_tf is not None and self.newest_tf is not None)
                else None),
            'problems': problems,
            'verdict': ('no-data' if self.clouds == 0
                        else ('problems' if problems else 'clean')),
        }


def main():
    rclpy.init(args=sys.argv)
    node = HwTfProbe()
    deadline = time.time() + node.duration
    try:
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    out = node.report()
    print(core.render_text('hw_tf_probe',
                           out['lookups'] + out['tf_broadcasts'],
                           out['problems']))
    print(f"  clouds seen {out['clouds_seen']}, duplicate TF stamps "
          f"{out['tf_duplicate_stamps']}, TF span {out['tf_span_s']} s")
    if node.json_path:
        with open(node.json_path, 'w') as f:
            json.dump(out, f, indent=2, sort_keys=True)
        print(f"\nJSON: {node.json_path}")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit({'no-data': 2, 'problems': 1, 'clean': 0}[out['verdict']])


if __name__ == '__main__':
    main()
