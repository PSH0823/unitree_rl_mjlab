#!/usr/bin/python3
"""CropBox self-filter retune from a raw-cloud bag (5B block 4, offline).

Reads /livox/lidar out of a bag recorded with the perception stack NOT
running (so nothing is filtered), keeps returns inside `--radius` of the
sensor origin — the self-hit candidates — and reports the axis-aligned
sensor-frame envelope they occupy, which is exactly what §9.3's CropBox
needs. Robot-body returns and genuine near obstacles are indistinguishable
geometrically, so the operator's job is to record a sequence where nothing is
within `--radius` except the robot (checklist block 4 says so explicitly);
this script only measures.

Reported at several percentiles because a max-envelope box grows without
bound with one stray point: the 99.9th percentile is the recommended basis,
with the max printed so the tail is visible rather than hidden.

  selfhit_analysis.py BAG [--radius 0.8] [--topic /livox/lidar] [--json OUT]

Sanity-check it on a sim fixture first: the s1_* bags contain the known
wrist returns at 0.28-0.36 m horizontal / 0.34-0.41 m below the sensor
(Phase 2/3 finding), so a run against those must reproduce that envelope.
"""
import argparse
import json
import sys

import numpy as np


def read_clouds(bag, topic):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs_py import point_cloud2

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        raise SystemExit(f'{topic} not in {bag}: {sorted(types)}')
    msg_type = get_message(types[topic])
    n = 0
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name != topic:
            continue
        msg = deserialize_message(data, msg_type)
        pts = np.array(list(point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)))
        if len(pts):
            yield np.asarray(pts.tolist(), dtype=float).reshape(-1, 3)
        n += 1
    if n == 0:
        raise SystemExit(f'no {topic} messages in {bag}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--radius', type=float, default=0.8,
                    help='self-hit candidate radius from the sensor origin (m)')
    ap.add_argument('--topic', default='/livox/lidar')
    ap.add_argument('--json', default='')
    args = ap.parse_args()

    near = []
    frames = 0
    per_frame = []
    for pts in read_clouds(args.bag, args.topic):
        frames += 1
        d = np.linalg.norm(pts, axis=1)
        sel = pts[d < args.radius]
        per_frame.append(len(sel))
        if len(sel):
            near.append(sel)
    if not near:
        print(f'no returns within {args.radius} m over {frames} frames — '
              'either the sensor sees nothing of the robot, or this bag was '
              'recorded with CropBox in the loop')
        return 0
    P = np.vstack(near)
    rxy = np.hypot(P[:, 0], P[:, 1])
    per_frame = np.array(per_frame)

    def q(a, p):
        return float(np.percentile(a, p))

    print(f'frames {frames}; self-hit candidates {len(P)} '
          f'(per frame: mean {per_frame.mean():.1f}, p95 {q(per_frame, 95):.0f}, '
          f'max {per_frame.max()})')
    print(f'{"":8s} {"p50":>9s} {"p99":>9s} {"p99.9":>9s} {"max":>9s} {"min":>9s}')
    out = {}
    for name, a in (('r_xy', rxy), ('|x|', np.abs(P[:, 0])),
                    ('|y|', np.abs(P[:, 1])), ('z', P[:, 2])):
        print(f'{name:8s} {q(a, 50):9.3f} {q(a, 99):9.3f} {q(a, 99.9):9.3f} '
              f'{a.max():9.3f} {a.min():9.3f}')
        out[name] = {'p50': q(a, 50), 'p99': q(a, 99), 'p999': q(a, 99.9),
                     'max': float(a.max()), 'min': float(a.min())}

    # Recommended box: 99.9th-percentile envelope + 2 cm margin, in the
    # SENSOR frame, which is what config/cropbox_self_filter.yaml expresses.
    m = 0.02
    box = {
        'min_x': -(q(np.abs(P[:, 0]), 99.9) + m),
        'max_x': +(q(np.abs(P[:, 0]), 99.9) + m),
        'min_y': -(q(np.abs(P[:, 1]), 99.9) + m),
        'max_y': +(q(np.abs(P[:, 1]), 99.9) + m),
        'min_z': float(np.percentile(P[:, 2], 0.1)) - m,
        'max_z': float(np.percentile(P[:, 2], 99.9)) + m,
    }
    print('\nproposed CropBox (sensor frame, negative=true, p99.9 + 2 cm):')
    for k in ('min_x', 'max_x', 'min_y', 'max_y', 'min_z', 'max_z'):
        print(f'  {k}: {box[k]:+.3f}')
    print('\nCROSS-CHECK BEFORE APPLYING: this box must not swallow a real '
          'obstacle at range_min (0.30 m). If max(|x|,|y|) approaches 0.30, '
          'the arms are reaching into the detection band and the answer is a '
          'shaped filter or an arm-pose-aware mask, not a bigger box.')
    out['proposed_box'] = box
    out['frames'] = frames
    out['radius'] = args.radius
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(out, f, indent=2)
        print('wrote', args.json)
    return 0


if __name__ == '__main__':
    sys.exit(main())
