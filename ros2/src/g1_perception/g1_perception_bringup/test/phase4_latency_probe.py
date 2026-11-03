#!/usr/bin/python3
"""Stamp-matched reception-latency probe: cloud -> /scan -> /tracked ->
/obstacles_safe, plus container CPU. argv: <secs> <out.txt>"""
import sys, time, subprocess
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, LaserScan
from obstacle_detector.msg import Obstacles

secs, out = float(sys.argv[1]), sys.argv[2]
rclpy.init()
node = rclpy.create_node('phase4_latency_probe')
arr = {t: {} for t in ('cloud', 'scan', 'tracked', 'safe')}

def key(stamp):
    return (stamp.sec, stamp.nanosec)

def mk(topic):
    def cb(m):
        arr[topic].setdefault(key(m.header.stamp), time.time())
    return cb

be = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
rel = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE)
node.create_subscription(PointCloud2, '/livox/lidar', mk('cloud'), be)
node.create_subscription(LaserScan, '/scan', mk('scan'), be)
node.create_subscription(Obstacles, '/tracked_obstacles', mk('tracked'), rel)
node.create_subscription(Obstacles, '/obstacles_safe', mk('safe'), rel)

def cpu(pid):
    try:
        with open(f'/proc/{pid}/stat') as f:
            v = f.read().split()
        return (int(v[13]) + int(v[14])) / 100.0
    except Exception:
        return None

def pid_of(pat):
    try:
        return int(subprocess.check_output(['pgrep', '-f', pat]).split()[0])
    except Exception:
        return None

cont = pid_of('rclcpp_components/component_containe[r]')
side = pid_of('sim_mjlidar_bridg[e]')
c0 = cpu(cont) if cont else None
s0 = cpu(side) if side else None
t0 = time.time()
while time.time() - t0 < secs:
    rclpy.spin_once(node, timeout_sec=0.2)
dt = time.time() - t0
c1 = cpu(cont) if cont else None
s1 = cpu(side) if side else None

with open(out, 'w') as f:
    def q(v, p):
        return sorted(v)[min(len(v)-1, int(p*len(v)))] if v else float('nan')
    for name in ('scan', 'tracked', 'safe'):
        d = [arr[name][k]-arr['cloud'][k] for k in arr[name] if k in arr['cloud']]
        f.write(f'cloud->{name}: n={len(d)} p50={q(d,0.5)*1000:.2f}ms '
                f'p95={q(d,0.95)*1000:.2f}ms p99={q(d,0.99)*1000:.2f}ms '
                f'max={max(d)*1000:.2f}ms\n' if d else f'cloud->{name}: none\n')
    if c0 is not None and c1 is not None:
        f.write(f'container CPU: {100*(c1-c0)/dt:.1f}% of one core over {dt:.0f}s\n')
    if s0 is not None and s1 is not None:
        f.write(f'sidecar CPU: {100*(s1-s0)/dt:.1f}%\n')
    f.write(f'frames: cloud={len(arr["cloud"])} scan={len(arr["scan"])} '
            f'tracked={len(arr["tracked"])} safe={len(arr["safe"])}\n')
print(open(out).read())
