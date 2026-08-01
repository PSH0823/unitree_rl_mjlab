#!/usr/bin/python3
"""Mini-sim for the measured-wall gate: publishes /clock (200 Hz) and
/sim/mj_state (100 Hz, §7.1) holding the grounded standing qpos from
wall_scene_gt.json. Replaces the simulate binary for this self-contained
test — no dynamics, no GL, no gamepad (the mirror is kinematic, §11.2).

Runs under system python (R-11) with rclpy.
"""
import argparse
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time as TimeMsg

from sim_msgs.msg import MjState

TICK = 0.005  # 200 Hz wall pacing, sim time advances at RT factor 1


class WallStateSource(Node):
    def __init__(self, qpos):
        super().__init__('wall_state_source')
        self.qpos = [float(v) for v in qpos]
        self.sim_time = 0.0
        self.tick_count = 0
        self.clock_pub = self.create_publisher(
            Clock, '/clock',
            QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=1))
        self.state_pub = self.create_publisher(
            MjState, '/sim/mj_state',
            QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=1))
        self.timer = self.create_timer(TICK, self.on_tick)

    def on_tick(self):
        self.sim_time += TICK
        self.tick_count += 1
        sec = int(self.sim_time)
        stamp = TimeMsg(sec=sec, nanosec=int(round((self.sim_time - sec) * 1e9)))
        self.clock_pub.publish(Clock(clock=stamp))
        if self.tick_count % 2 == 0:  # 100 Hz
            msg = MjState()
            msg.sim_time = self.sim_time
            msg.qpos = self.qpos
            msg.mocap_pos = []
            msg.mocap_quat = []
            self.state_pub.publish(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gt-json', required=True)
    args, ros_args = ap.parse_known_args()
    with open(args.gt_json) as f:
        gt = json.load(f)
    rclpy.init(args=ros_args)
    node = WallStateSource(gt['qpos'])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
