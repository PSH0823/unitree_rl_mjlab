#!/usr/bin/python3
"""Mid360 bring-up preflight (Phase 5A; checklist step 5B-1).

Run this on the machine that will run the driver, BEFORE launching anything.
Every check here corresponds to a failure mode that otherwise presents as
"the driver starts, prints nothing unusual, and /livox/lidar stays empty":

  1. host_net_info IPs are actually assigned to a local interface. The SDK
     binds those addresses; if none matches, the sockets never receive.
  2. host ports are free.
  3. the LiDAR IP is on the same /24 as the host IP it must answer.
  4. the driver YAML says xfer_format 0 (the only PointCloud2 format the
     ROS2 build implements) and frame_id mid360_link (§7.1).
  5. extrinsic_parameter is identity — H-1 lives in the xacro only (§8.3);
     setting it here too would apply the extrinsic twice.
  6. the config still carries the Q-1 placeholder IPs -> loud warning.

Usage:  hw_config_check.py [MID360_config.json] [livox_driver.yaml]
Exit 0 pass, 1 fail, 2 placeholder config (fail on the robot, expected on
the dev machine).
"""
import json
import os
import socket
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
# Works both from the source tree (../config) and installed into
# lib/<pkg>/ (where the configs live in the package's share directory).
CONFIG_DIR = os.path.abspath(os.path.join(HERE, '..', 'config'))
if not os.path.isdir(CONFIG_DIR):
    try:
        from ament_index_python.packages import get_package_share_directory
        CONFIG_DIR = os.path.join(
            get_package_share_directory('g1_perception_bringup'), 'config')
    except Exception:
        pass
DEFAULT_JSON = os.path.join(CONFIG_DIR, 'MID360_config.json')
DEFAULT_YAML = os.path.join(CONFIG_DIR, 'livox_driver.yaml')

# Upstream sample values — never a real robot's.
PLACEHOLDER_HOST = '192.168.1.5'
PLACEHOLDER_LIDAR = '192.168.1.12'


def local_ips():
    ips = set()
    try:
        import fcntl
        import array
        import struct
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        names = array.array('B', b'\0' * 4096)
        outbytes = struct.unpack('iL', fcntl.ioctl(
            s.fileno(), 0x8912,  # SIOCGIFCONF
            struct.pack('iL', 4096, names.buffer_info()[0])))[0]
        raw = names.tobytes()[:outbytes]
        for i in range(0, outbytes, 40):
            ips.add(socket.inet_ntoa(raw[i + 20:i + 24]))
    except Exception:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None,
                                       socket.AF_INET):
            ips.add(info[4][0])
    except Exception:
        pass
    ips.add('127.0.0.1')
    return ips


def port_free(ip, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind((ip, port))
        return True
    except OSError:
        return False
    finally:
        s.close()


def main():
    json_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_JSON
    yaml_path = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_YAML
    fails, warns = [], []

    with open(json_path) as f:
        cfg = json.load(f)
    with open(yaml_path) as f:
        drv = yaml.safe_load(f)['/**']['ros__parameters']

    host = cfg['MID360']['host_net_info']
    lidars = cfg['lidar_configs']
    host_ips = {v for k, v in host.items() if k.endswith('_ip') and v}
    lidar_ips = {c['ip'] for c in lidars}

    placeholder = (PLACEHOLDER_HOST in host_ips or
                   PLACEHOLDER_LIDAR in lidar_ips)

    mine = local_ips()
    print('local IPv4 addresses:', sorted(mine))
    for ip in sorted(host_ips):
        if ip not in mine:
            fails.append(f'host_net_info IP {ip} is not assigned to any local '
                         'interface — the SDK will bind nothing')
    for key, val in sorted(host.items()):
        if key.endswith('_port') and host.get(key.replace('_port', '_ip')):
            ip = host[key.replace('_port', '_ip')]
            if ip in mine and not port_free(ip, val):
                fails.append(f'{key} {ip}:{val} is already bound '
                             '(another driver still running?)')

    for lip in sorted(lidar_ips):
        if not any(lip.rsplit('.', 1)[0] == h.rsplit('.', 1)[0]
                   for h in host_ips):
            warns.append(f'LiDAR {lip} shares no /24 with any host IP '
                         f'{sorted(host_ips)} — routing must be explicit')

    if drv.get('xfer_format') != 0:
        fails.append(f"xfer_format {drv.get('xfer_format')}: only 0 produces a "
                     'PointCloud2 in the ROS2 build (1 = CustomMsg, '
                     '2 = ROS1-only pcl::PointCloud, publishes nothing)')
    if drv.get('frame_id') != 'mid360_link':
        fails.append(f"frame_id {drv.get('frame_id')!r} != mid360_link (§7.1)")
    if float(drv.get('publish_freq', 0)) != 10.0:
        warns.append(f"publish_freq {drv.get('publish_freq')} != 10 Hz (§7.1)")

    for c in lidars:
        ex = c.get('extrinsic_parameter', {})
        if any(float(ex.get(k, 0)) != 0.0 for k in
               ('roll', 'pitch', 'yaw', 'x', 'y', 'z')):
            fails.append(f"lidar {c['ip']} extrinsic_parameter is not identity "
                         '— H-1 belongs in the xacro only (§8.3); setting it '
                         'here applies the extrinsic twice')

    for w in warns:
        print('WARN:', w)
    for f in fails:
        print('FAIL:', f)

    if placeholder:
        print('\nPLACEHOLDER CONFIG: the IPs are upstream sample values, not '
              'this robot (Q-1). Fill them in from the LiDAR serial sticker '
              'and the onboard PC network before 5B block 1.')
        sys.exit(2)
    if fails:
        sys.exit(1)
    print('hw_config_check: PASS')
    sys.exit(0)


if __name__ == '__main__':
    main()
