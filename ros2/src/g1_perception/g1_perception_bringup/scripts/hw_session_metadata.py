#!/usr/bin/python3
"""Hardware-session metadata writer (Phase 5C, §7 of the 5C brief).

A bag without provenance is not evidence. This collects everything the
machine can know by itself — commit, branch, architecture, ROS/DDS
environment, the checksums of the YAMLs that were actually LOADED, the
network configuration — and leaves an explicit, non-optional slot for
everything only the operator knows: which G1, which LiDAR serial, what the
robot was doing, where the props were.

Machine-derivable fields are filled in. Operator fields default to `""` /
`null` and are REPORTED AS MISSING at the end, so an incomplete record is
visible at capture time instead of six weeks later.

  hw_session_metadata.py --out evidence/hardware/2026-08-10/s1/session.json \
      --g1-variant "G1 EDU" --lidar-serial 47MDL... --robot-state Passive \
      --scenario "flat floor, three surveyed cylinders" \
      --note "arms at nominal pose"

`--from-json extra.json` merges a partially filled record (e.g. the props
table) so the operator can prepare it before the session.
"""
import argparse
import hashlib
import json
import os
import platform
import shutil
import subprocess
import sys
import time

TRACKED_CONFIGS = (
    'config/MID360_config.json',
    'config/livox_driver.yaml',
    'config/dlio.yaml',
    'config/cropbox_self_filter.yaml',
    'config/pointcloud_to_laserscan.yaml',
    'config/obstacle_detector.yaml',
    'config/safety_obstacle_filter.yaml',
)

# The fields nothing but a human can supply. Reported as missing when blank.
OPERATOR_FIELDS = (
    ('hardware', 'g1_variant'),
    ('hardware', 'lidar_serial'),
    ('hardware', 'lidar_firmware'),
    ('hardware', 'mounted_extrinsic'),
    ('session', 'robot_state'),
    ('session', 'scenario'),
    ('session', 'operator'),
)

ROBOT_STATES = ('powered_off', 'powered_not_standing', 'Passive', 'FixStand',
                'RLBase', 'walking', 'externally_supported', 'unknown')


def sh(cmd, default=''):
    try:
        return subprocess.run(cmd, capture_output=True, text=True,
                              timeout=10).stdout.strip() or default
    except Exception:
        return default


def md5(path):
    h = hashlib.md5()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()


def share_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('g1_perception_bringup')
    except Exception:
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.abspath(os.path.join(here, '..'))


def collect(args):
    share = share_dir()
    cfg = {}
    for rel in TRACKED_CONFIGS:
        p = os.path.join(share, rel)
        cfg[rel] = md5(p) if os.path.isfile(p) else None

    rec = {
        'schema': 'g1_hw_perception_session/1',
        'date': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        'unix_time': time.time(),
        'repo': {
            'commit': sh(['git', 'rev-parse', 'HEAD'], 'unknown'),
            'branch': sh(['git', 'rev-parse', '--abbrev-ref', 'HEAD'],
                         'unknown'),
            'dirty': bool(sh(['git', 'status', '--porcelain'])),
        },
        'hardware': {
            'g1_variant': args.g1_variant,
            'target_arch': platform.machine(),
            'target_host': platform.node(),
            'os': sh(['bash', '-c',
                      '. /etc/os-release 2>/dev/null && echo $PRETTY_NAME'],
                     'unknown'),
            'kernel': platform.release(),
            'lidar_serial': args.lidar_serial,
            'lidar_firmware': args.lidar_firmware,
            'mounted_extrinsic': args.mounted_extrinsic,
            'perception_runs_on': args.perception_host,
        },
        'network': {
            'ros_interface': args.ros_interface,
            'livox_interface': args.livox_interface,
            'ros_domain_id': int(os.environ.get('ROS_DOMAIN_ID', 0)),
            'rmw_implementation': os.environ.get('RMW_IMPLEMENTATION',
                                                 'unset'),
            'cyclonedds_uri': os.environ.get('CYCLONEDDS_URI', 'unset'),
            'interfaces': sh(['ip', '-br', 'addr']).splitlines(),
            'routes': sh(['ip', 'route']).splitlines(),
        },
        'config': {
            'share_dir': share,
            'checksums': cfg,
        },
        'session': {
            'name': args.name,
            'operator': args.operator,
            'robot_state': args.robot_state,
            'scenario': args.scenario,
            'stage': args.stage,
            'notes': args.note,
            'obstacles': [],          # [{name, x, y, r, measured_how}]
            'known_failures': [],
            'robot_motion_occurred': args.robot_motion,
            'command_publisher_active': False,
        },
        # The §7 summary block. Nulls are filled in offline from the probes'
        # own JSON; they are never guessed.
        'results': {
            'lidar_rate_hz': None,
            'imu_rate_hz': None,
            'odom_rate_hz': None,
            'tf_lookup_success': None,
            'cloud_to_safe_p95_ms': None,
        },
        'actuation_enabled': False,
    }
    if args.bag:
        rec['bag'] = {
            'path': os.path.abspath(args.bag),
            'topics': [],             # filled by `ros2 bag info` post hoc
        }
    return rec


def deep_merge(base, extra):
    for k, v in extra.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            deep_merge(base[k], v)
        else:
            base[k] = v
    return base


def missing(rec):
    out = []
    for sect, key in OPERATOR_FIELDS:
        if not rec.get(sect, {}).get(key):
            out.append(f'{sect}.{key}')
    if not rec['session']['obstacles'] and rec['session'].get('stage') in (
            '10', '11', '12'):
        out.append('session.obstacles (a detector/tracker stage without '
                   'surveyed fixture geometry cannot be scored)')
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--bag', default='')
    ap.add_argument('--name', default='')
    ap.add_argument('--stage', default='')
    ap.add_argument('--operator', default='')
    ap.add_argument('--g1-variant', dest='g1_variant', default='')
    ap.add_argument('--lidar-serial', dest='lidar_serial', default='')
    ap.add_argument('--lidar-firmware', dest='lidar_firmware', default='')
    ap.add_argument('--mounted-extrinsic', dest='mounted_extrinsic',
                    default='', help='measured mount, e.g. '
                                     '"x=0.000 y=0.000 z=0.428 r=pi p=0.0009 y=0"')
    ap.add_argument('--perception-host', dest='perception_host',
                    default='', choices=['', 'onboard', 'workstation'])
    ap.add_argument('--ros-interface', dest='ros_interface', default='')
    ap.add_argument('--livox-interface', dest='livox_interface', default='')
    ap.add_argument('--robot-state', dest='robot_state', default='',
                    choices=('',) + ROBOT_STATES)
    ap.add_argument('--robot-motion', dest='robot_motion',
                    action='store_true',
                    help='the robot physically moved during this capture')
    ap.add_argument('--scenario', default='')
    ap.add_argument('--note', action='append', default=[])
    ap.add_argument('--from-json', dest='from_json', default='')
    ap.add_argument('--copy-configs', action='store_true',
                    help='also copy the loaded YAMLs next to the record')
    args = ap.parse_args()

    rec = collect(args)
    if args.from_json:
        with open(args.from_json) as f:
            deep_merge(rec, json.load(f))

    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or '.',
                exist_ok=True)
    with open(args.out, 'w') as f:
        json.dump(rec, f, indent=2, sort_keys=True)

    if args.copy_configs:
        dst = os.path.join(os.path.dirname(os.path.abspath(args.out)),
                           'configs')
        os.makedirs(dst, exist_ok=True)
        for rel in TRACKED_CONFIGS:
            src = os.path.join(rec['config']['share_dir'], rel)
            if os.path.isfile(src):
                shutil.copy2(src, os.path.join(dst, os.path.basename(src)))
        print(f'configs copied to {dst}')

    print(f'session metadata: {args.out}')
    miss = missing(rec)
    if miss:
        print('\nINCOMPLETE — these fields are the operator\'s and are blank:')
        for m in miss:
            print('  -', m)
        print('Fill them in now, while the robot is in front of you. A bag '
              'whose provenance is reconstructed later is not evidence.')
        sys.exit(1)
    print('metadata complete')
    sys.exit(0)


if __name__ == '__main__':
    main()
