#!/usr/bin/python3
"""Non-actuating hardware SOURCE probe (Phase 5C, stage 3).

Subscribes to /livox/lidar and /livox/imu and nothing else. Publishes nothing.
Does not need DLIO, TF, the perception container, or the robot controller.
It cannot move the robot: it owns no publisher of any kind.

What it answers, all of which "the topic is in `ros2 topic list`" does not:

  * is data actually flowing, at what rate, with what arrival gaps
  * is the frame_id the one the whole TF tree assumes (mid360_link)
  * is the PointCloud2 the pinned driver's 7-field / point_step 26 layout
  * do header stamps ever go BACKWARDS (fatal for tf2 and every filter)
  * are stamps in this host's clock or the device's (§14.3, unresolved
    until it is measured on the robot)
  * how many points per frame, what fraction finite, what range distribution
  * is the IMU plausible at rest (|a| ~ g, |w| ~ 0)
  * did OUR best-effort subscription actually match the publisher's QoS

Usage
  ros2 run g1_perception_bringup hw_source_probe.py --ros-args \
      -p duration:=30.0 -p json:=/tmp/b1_source_probe.json

Exit 0 = probe ran and found no problems it can see. Exit 1 = problems found
(they are printed and written to the JSON). Exit 2 = no data at all.
"""
import json
import os
import sys
import time

# hw_probe_core.py is installed into lib/<pkg>/ next to this file and also
# sits next to it in the source tree. Nothing else on PYTHONPATH is assumed.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy                                                    # noqa: E402
from rclpy.node import Node                                     # noqa: E402
from rclpy.qos import qos_profile_sensor_data                   # noqa: E402
from sensor_msgs.msg import Imu, PointCloud2                    # noqa: E402

import hw_probe_core as core                                    # noqa: E402


class HwSourceProbe(Node):
    def __init__(self):
        super().__init__('hw_source_probe')
        self.duration = float(self.declare_parameter('duration', 30.0).value)
        self.cloud_topic = self.declare_parameter(
            'cloud_topic', '/livox/lidar').value
        self.imu_topic = self.declare_parameter('imu_topic', '/livox/imu').value
        self.expect_frame = self.declare_parameter(
            'expect_frame', core.EXPECTED_FRAME).value
        self.expect_cloud_hz = float(
            self.declare_parameter('expect_cloud_hz', 10.0).value)
        self.expect_imu_hz = float(
            self.declare_parameter('expect_imu_hz', 200.0).value)
        # Reading every point of every 10 Hz frame in python is not free and
        # is not the point: geometry sampling is diagnostic, not a product.
        self.sample_every = int(self.declare_parameter('sample_every', 5).value)
        self.json_path = self.declare_parameter('json', '').value

        self.cloud = core.StampSeries(self.cloud_topic,
                                      expect_hz=self.expect_cloud_hz)
        self.imu = core.StampSeries(self.imu_topic,
                                    expect_hz=self.expect_imu_hz)
        self.frames = []          # per-frame geometry samples
        self.frame_ids = {}       # frame_id -> count, cloud
        self.imu_frame_ids = {}
        self.layout_fatal, self.layout_dev = [], []
        self.layout_seen = None
        self.accel_mags, self.gyro_mags = [], []
        self.first_cloud_wall = None
        self.first_imu_wall = None
        self.t_start = time.time()

        # SensorData (best-effort, depth 5) on purpose: it is what every other
        # subscriber in this stack uses, so if the probe matches, they match.
        # The driver publishes RELIABLE depth 256 — the compatible direction.
        self.create_subscription(PointCloud2, self.cloud_topic,
                                 self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(Imu, self.imu_topic,
                                 self._on_imu, qos_profile_sensor_data)
        self.get_logger().info(
            f'probing {self.cloud_topic} + {self.imu_topic} for '
            f'{self.duration:.0f} s — publishing nothing, moving nothing')

    # --- callbacks ---------------------------------------------------------
    def _on_cloud(self, msg):
        recv = time.time()
        if self.first_cloud_wall is None:
            self.first_cloud_wall = recv - self.t_start
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.cloud.add(stamp, recv)
        self.frame_ids[msg.header.frame_id] = \
            self.frame_ids.get(msg.header.frame_id, 0) + 1

        if self.layout_seen is None:
            self.layout_seen = [(f.name, f.offset, f.datatype)
                                for f in msg.fields]
            self.layout_fatal, self.layout_dev = core.check_cloud_layout(
                self.layout_seen, msg.point_step)

        if self.cloud.n % self.sample_every:
            return
        self.frames.append(self._sample_geometry(msg))

    def _sample_geometry(self, msg):
        from sensor_msgs_py import point_cloud2
        import numpy as np
        total = int(msg.width) * int(msg.height)
        rec = {'points': total, 'finite': None, 'r_p05': None, 'r_p50': None,
               'r_p95': None, 'r_max': None, 'near_0p3_frac': None}
        try:
            arr = point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=False)
            xyz = np.stack([np.asarray(arr['x'], dtype=float),
                            np.asarray(arr['y'], dtype=float),
                            np.asarray(arr['z'], dtype=float)], axis=-1)
        except Exception as exc:                    # layout we cannot read
            rec['error'] = f'{type(exc).__name__}: {exc}'
            return rec
        if xyz.size == 0:
            rec['finite'] = 0.0
            return rec
        good = np.isfinite(xyz).all(axis=1)
        rec['finite'] = float(good.mean())
        r = np.linalg.norm(xyz[good], axis=1)
        if r.size:
            rec['r_p05'] = float(np.percentile(r, 5))
            rec['r_p50'] = float(np.percentile(r, 50))
            rec['r_p95'] = float(np.percentile(r, 95))
            rec['r_max'] = float(r.max())
            # Returns inside the projection's range_min. On a clean flat-floor
            # capture with no obstacle nearby these are the robot's own body:
            # the raw material for stage 7's self-hit analysis, and the reason
            # this probe runs BEFORE CropBox exists in the graph.
            rec['near_0p3_frac'] = float((r < 0.30).mean())
        return rec

    def _on_imu(self, msg):
        recv = time.time()
        if self.first_imu_wall is None:
            self.first_imu_wall = recv - self.t_start
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.imu.add(stamp, recv)
        self.imu_frame_ids[msg.header.frame_id] = \
            self.imu_frame_ids.get(msg.header.frame_id, 0) + 1
        a, w = msg.linear_acceleration, msg.angular_velocity
        self.accel_mags.append((a.x ** 2 + a.y ** 2 + a.z ** 2) ** 0.5)
        self.gyro_mags.append((w.x ** 2 + w.y ** 2 + w.z ** 2) ** 0.5)

    # --- report ------------------------------------------------------------
    def _qos_report(self, topic):
        try:
            infos = self.get_publishers_info_by_topic(topic)
        except Exception:
            return []
        return [{'node': i.node_name,
                 'reliability': str(i.qos_profile.reliability),
                 'durability': str(i.qos_profile.durability),
                 'depth': i.qos_profile.depth} for i in infos]

    def report(self):
        import numpy as np
        problems = []
        problems += self.cloud.problems()
        problems += self.imu.problems()
        problems += self.layout_fatal
        problems += core.imu_plausibility(self.accel_mags, self.gyro_mags)

        for label, ids in (('/livox/lidar', self.frame_ids),
                           ('/livox/imu', self.imu_frame_ids)):
            for fid in ids:
                bad = core.check_frame_id(fid, self.expect_frame)
                if bad:
                    problems.append(f'{label}: {bad}')
            if len(ids) > 1:
                problems.append(f'{label}: MIXED frame_ids {sorted(ids)}')

        pubs = self._qos_report(self.cloud_topic)
        if self.cloud.n == 0 and pubs:
            problems.append(
                f'{self.cloud_topic}: publisher present but no message reached '
                'a best-effort SensorData subscription — QoS or DDS transport '
                'problem, not a sensor problem')

        geo = {}
        if self.frames:
            def col(k):
                v = [f[k] for f in self.frames if f.get(k) is not None]
                return v
            pts = col('points')
            fin = col('finite')
            near = col('near_0p3_frac')
            geo = {
                'frames_sampled': len(self.frames),
                'points_per_frame_min': int(min(pts)) if pts else None,
                'points_per_frame_median': float(np.median(pts)) if pts else None,
                'points_per_frame_max': int(max(pts)) if pts else None,
                'finite_fraction_min': core._r(min(fin)) if fin else None,
                'range_p05_m': core._r(np.median(col('r_p05'))) if col('r_p05') else None,
                'range_p50_m': core._r(np.median(col('r_p50'))) if col('r_p50') else None,
                'range_p95_m': core._r(np.median(col('r_p95'))) if col('r_p95') else None,
                'range_max_m': core._r(max(col('r_max'))) if col('r_max') else None,
                'fraction_inside_range_min_0p3': core._r(
                    float(np.median(near))) if near else None,
            }
            if fin and min(fin) < 0.99:
                problems.append(
                    f'{self.cloud_topic}: non-finite points present (min finite '
                    f'fraction {min(fin):.3f}); is_dense is not trustworthy')

        out = {
            'probe': 'hw_source_probe',
            'duration_s': core._r(time.time() - self.t_start),
            'first_cloud_after_s': core._r(self.first_cloud_wall),
            'first_imu_after_s': core._r(self.first_imu_wall),
            'cloud': self.cloud.summary(),
            'imu': self.imu.summary(),
            'cloud_frame_ids': self.frame_ids,
            'imu_frame_ids': self.imu_frame_ids,
            'cloud_layout': [list(f) for f in (self.layout_seen or [])],
            'cloud_layout_deviations': self.layout_dev,
            'cloud_publishers': pubs,
            'imu_publishers': self._qos_report(self.imu_topic),
            'geometry': geo,
            'imu_accel_median': core._r(
                core._pct(sorted(self.accel_mags), 50)),
            'imu_gyro_median': core._r(core._pct(sorted(self.gyro_mags), 50)),
            'problems': problems,
            'verdict': ('no-data' if (self.cloud.n == 0 and self.imu.n == 0)
                        else ('problems' if problems else 'clean')),
        }
        return out


def main():
    rclpy.init(args=sys.argv)
    node = HwSourceProbe()
    deadline = time.time() + node.duration
    try:
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    out = node.report()
    summaries = [out['cloud'], out['imu']]
    if out['geometry']:
        summaries.append(out['geometry'])
    print(core.render_text('hw_source_probe', summaries, out['problems']))
    if out['cloud_layout_deviations']:
        print('cloud layout deviations from the pinned driver:')
        for d in out['cloud_layout_deviations']:
            print('  -', d)
    if node.json_path:
        with open(node.json_path, 'w') as f:
            json.dump(out, f, indent=2, sort_keys=True)
        print(f'\nJSON: {node.json_path}')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit({'no-data': 2, 'problems': 1, 'clean': 0}[out['verdict']])


if __name__ == '__main__':
    main()
