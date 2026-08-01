#!/usr/bin/python3
"""Dump /raw_obstacles + /tracked_obstacles to JSONL for the T8 comparison.
argv: <secs> <out_jsonl>"""
import json
import sys
import time

import rclpy
from rclpy.qos import QoSProfile

from obstacle_detector.msg import Obstacles


def main():
    secs, out = float(sys.argv[1]), sys.argv[2]
    rclpy.init()
    node = rclpy.create_node('t8_dump_probe')
    f = open(out, 'w')

    def on_msg(topic, m):
        f.write(json.dumps({
            't': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
            'topic': topic, 'frame': m.header.frame_id,
            'circles': [{'x': c.center.x, 'y': c.center.y, 'r': c.radius,
                         'tr': c.true_radius, 'vx': c.velocity.x,
                         'vy': c.velocity.y} for c in m.circles],
            'nseg': len(m.segments)}) + '\n')

    for topic in ('/raw_obstacles', '/tracked_obstacles'):
        node.create_subscription(Obstacles, topic,
                                 lambda m, t=topic: on_msg(t, m),
                                 QoSProfile(depth=5))
    t0 = time.time()
    while time.time() - t0 < secs:
        rclpy.spin_once(node, timeout_sec=0.2)
    f.close()


if __name__ == '__main__':
    main()
