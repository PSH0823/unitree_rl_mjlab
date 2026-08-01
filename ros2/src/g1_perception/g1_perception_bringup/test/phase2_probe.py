#!/usr/bin/python3
"""Phase-2 measurement probe (§17.4 discipline; reused for Phase-5 Q-3).

Attaches to a running projection chain (live sim or bag replay) and measures:
  - /livox/lidar, /points_self_filtered, /scan rates + drop fractions
    (computed from sim-time stamp spans, so realtime factor cancels)
  - per-hop pipeline latency (stamp-matched reception times on this probe):
    cloud->filtered and cloud->scan, p50/p95
  - CropBox removal (input width - filtered width per matched stamp)
  - T9 live: odom->base_footprint within 50 ms of every cloud receipt
    (counting starts at first success — pre-discovery TF is unobservable)
  - /scan occupied-bin fraction
  - with --gt-json (wall scene): per-target occupied-bin fraction and
    points/bin distribution inside each target's bearing window, from the
    filtered cloud transformed to base_footprint (Q-3 baseline)
  - with --hash-out: per-scan (stamp, md5(ranges)) lines for the T8
    determinism comparison

Usage:
  phase2_probe.py --duration 60 [--gt-json GT] [--hash-out F] [--json F]
"""
import argparse
import hashlib
import json
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
import tf2_ros

ANGLE_INCREMENT = 0.0058
BAND = (0.15, 1.60)
RANGE = (0.3, 5.0)
CONTRACT_HZ = 10.0


def stamp_to_f(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def pct(a, q):
    return float(np.percentile(np.asarray(a), q)) if len(a) else float('nan')


class Probe(Node):
    def __init__(self, args):
        super().__init__('phase2_probe')
        self.args = args
        self.lock = threading.Lock()
        self.clouds = {}     # stamp -> (t_recv, width)
        self.cloud_order = []
        self.filtered = {}   # stamp -> (t_recv, width, xyz or None)
        self.scans = []      # (stamp, t_recv, msg)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.keep_xyz = args.gt_json is not None
        self.create_subscription(PointCloud2, '/livox/lidar',
                                 self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/points_self_filtered',
                                 self.on_filtered, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan',
                                 self.on_scan, qos_profile_sensor_data)

    def on_cloud(self, m):
        with self.lock:
            t = stamp_to_f(m.header.stamp)
            self.clouds[t] = (time.monotonic(), m.width)
            self.cloud_order.append((m.header.stamp, time.monotonic()))

    def on_filtered(self, m):
        xyz = None
        if self.keep_xyz and m.width:
            xyz = np.frombuffer(m.data, dtype=np.float32).reshape(-1, 3).copy()
        with self.lock:
            self.filtered[stamp_to_f(m.header.stamp)] = (
                time.monotonic(), m.width, xyz)

    def on_scan(self, m):
        with self.lock:
            self.scans.append((stamp_to_f(m.header.stamp),
                               time.monotonic(), m))


def t9_worker(probe, state):
    done = 0
    warm = False
    while not state['stop']:
        with probe.lock:
            pending = probe.cloud_order[done:]
        if not pending:
            time.sleep(0.005)
            continue
        for stamp, t_recv in pending:
            deadline = t_recv + 0.050
            ok = probe.tf_buffer.can_transform(
                'odom', 'base_footprint', Time.from_msg(stamp))
            while not ok and time.monotonic() < deadline:
                time.sleep(0.002)
                ok = probe.tf_buffer.can_transform(
                    'odom', 'base_footprint', Time.from_msg(stamp))
            if not warm:
                warm = ok
                continue
            state['checked'] += 1
            state['misses'] += 0 if ok else 1
        done += len(pending)


def tf_to_mat(tr):
    q = tr.transform.rotation
    t = tr.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return R, np.array([t.x, t.y, t.z])


def rate_block(stamps):
    if len(stamps) < 3:
        return {'frames': len(stamps)}
    stamps = sorted(stamps)
    span = stamps[-1] - stamps[0]
    expected = span * CONTRACT_HZ + 1
    return {'frames': len(stamps), 'span_s': round(span, 2),
            'hz': round((len(stamps) - 1) / span, 3),
            'drop_fraction': round(1 - len(stamps) / expected, 4)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=60.0)
    ap.add_argument('--gt-json')
    ap.add_argument('--hash-out')
    ap.add_argument('--json')
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    probe = Probe(args)
    execu = rclpy.executors.SingleThreadedExecutor()
    execu.add_node(probe)
    spin = threading.Thread(target=execu.spin, daemon=True)
    spin.start()

    t9 = {'stop': False, 'checked': 0, 'misses': 0}
    t9t = threading.Thread(target=t9_worker, args=(probe, t9), daemon=True)
    t9t.start()

    time.sleep(args.duration)
    t9['stop'] = True
    t9t.join(timeout=2)
    execu.shutdown()

    with probe.lock:
        clouds = dict(probe.clouds)
        filtered = dict(probe.filtered)
        scans = list(probe.scans)

    out = {'t9': {'checked': t9['checked'], 'misses': t9['misses']}}
    out['rates'] = {
        'livox_lidar': rate_block(list(clouds)),
        'points_self_filtered': rate_block(list(filtered)),
        'scan': rate_block([s for s, _, _ in scans]),
    }

    lat_filt, lat_scan, removed = [], [], []
    for t, (rc, w_in) in clouds.items():
        if t in filtered:
            lat_filt.append(filtered[t][0] - rc)
            removed.append(w_in - filtered[t][1])
    scan_by_stamp = {s: rt for s, rt, _ in scans}
    for t, (rc, _) in clouds.items():
        if t in scan_by_stamp:
            lat_scan.append(scan_by_stamp[t] - rc)
    out['latency_ms'] = {
        'cloud_to_filtered': {'p50': round(pct(lat_filt, 50) * 1e3, 2),
                              'p95': round(pct(lat_filt, 95) * 1e3, 2),
                              'n': len(lat_filt)},
        'cloud_to_scan': {'p50': round(pct(lat_scan, 50) * 1e3, 2),
                          'p95': round(pct(lat_scan, 95) * 1e3, 2),
                          'n': len(lat_scan)},
    }
    if removed:
        out['cropbox_removed_points'] = {
            'median': int(np.median(removed)), 'max': int(np.max(removed)),
            'mean': round(float(np.mean(removed)), 1), 'n': len(removed)}

    occ = [float(np.isfinite(np.asarray(m.ranges)).mean())
           for _, _, m in scans]
    if occ:
        out['scan_occupied_bin_fraction'] = {
            'mean': round(float(np.mean(occ)), 4),
            'min': round(float(np.min(occ)), 4),
            'max': round(float(np.max(occ)), 4)}

    if args.hash_out:
        with open(args.hash_out, 'w') as f:
            for s, _, m in scans:
                h = hashlib.md5(np.asarray(
                    m.ranges, dtype=np.float32).tobytes()).hexdigest()
                f.write(f'{s:.9f} {h}\n')

    if args.gt_json:
        with open(args.gt_json) as f:
            gt = json.load(f)
        out['targets'] = {}
        for tgt in gt['targets']:
            b = math.radians(tgt['bearing_deg'])
            if tgt['kind'] == 'cylinder':
                half = math.atan2(tgt['radius_m'],
                                  tgt['range_m'] + tgt['radius_m'])
            else:
                half = math.radians(2.0)
            occ_frac, pts_per_bin = [], []
            for s, _, m in scans:
                n = len(m.ranges)
                idx = [i for i in range(n)
                       if abs(math.atan2(
                           math.sin(m.angle_min + (i + .5) * m.angle_increment
                                    - b),
                           math.cos(m.angle_min + (i + .5) * m.angle_increment
                                    - b))) <= half]
                if not idx:
                    continue
                finite = [i for i in idx if math.isfinite(m.ranges[i])]
                occ_frac.append(len(finite) / len(idx))
                if s in filtered and filtered[s][2] is not None:
                    try:
                        tr = probe.tf_buffer.lookup_transform(
                            'base_footprint', 'mid360_link',
                            Time(seconds=int(s), nanoseconds=int(
                                round((s - int(s)) * 1e9))))
                    except Exception:  # noqa: BLE001 — TF expired, skip frame
                        continue
                    R, T = tf_to_mat(tr)
                    p = filtered[s][2] @ R.T + T
                    r = np.hypot(p[:, 0], p[:, 1])
                    theta = np.arctan2(p[:, 1], p[:, 0])
                    dth = np.arctan2(np.sin(theta - b), np.cos(theta - b))
                    sel = ((np.abs(dth) <= half) &
                           (p[:, 2] >= BAND[0]) & (p[:, 2] <= BAND[1]) &
                           (r >= RANGE[0]) & (r <= RANGE[1]))
                    if sel.any():
                        bins = ((theta[sel] + math.pi) //
                                ANGLE_INCREMENT).astype(int)
                        _, counts = np.unique(bins, return_counts=True)
                        pts_per_bin.extend(counts.tolist())
            out['targets'][tgt['name']] = {
                'window_halfangle_deg': round(math.degrees(half), 2),
                'occupied_bin_fraction_mean': round(
                    float(np.mean(occ_frac)), 4) if occ_frac else None,
                'points_per_occupied_bin': {
                    'mean': round(float(np.mean(pts_per_bin)), 2),
                    'p50': pct(pts_per_bin, 50), 'p95': pct(pts_per_bin, 95),
                    'n_bins': len(pts_per_bin)} if pts_per_bin else None,
            }

    print(json.dumps(out, indent=2))
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(out, f, indent=2)
    probe.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
