#!/usr/bin/python3
"""Phase-4 stream dumper: /sim/gt_obstacles + /tracked_obstacles +
/obstacles_safe to JSONL (with uid) for the §9.6 containment calibration and
the §17.3 A/B export. argv: <secs> <out_jsonl>"""
import json
import sys
import time

import rclpy
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy)

from obstacle_detector.msg import Obstacles

TOPICS = {
    '/sim/gt_obstacles': 1,
    # Pre-tracker detections: the KF's actual MEASUREMENTS. Their scatter on a
    # static scene is what `measurement_variance` is supposed to be (gap G1);
    # without them a robot-day capture cannot derive R and the sigma term
    # stays uncalibratable. See measure_measurement_variance.py.
    '/raw_obstacles': 5,
    '/tracked_obstacles': 5,
    '/obstacles_safe': 1,
}


def main():
    secs, out = float(sys.argv[1]), sys.argv[2]
    rclpy.init()
    node = rclpy.create_node('phase4_obstacles_dump')
    f = open(out, 'w')

    def on_msg(topic, m):
        f.write(json.dumps({
            't': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
            'topic': topic,
            # 'cov' = CircleObstacle.covariance [var_x, var_y, var_r] (P-3, fork
            # patch 0007). Zero from producers that are not the tracker; kept in
            # the dump so calibrate_k_sigma.py can run on the same capture the
            # containment calibration uses — including 5B's hardware session.
            'circles': [{'x': c.center.x, 'y': c.center.y, 'r': c.radius,
                         'tr': c.true_radius, 'vx': c.velocity.x,
                         'vy': c.velocity.y, 'uid': c.uid,
                         'cov': list(c.covariance)}
                        for c in m.circles]}) + '\n')

    for topic, depth in TOPICS.items():
        node.create_subscription(
            Obstacles, topic, lambda m, t=topic: on_msg(t, m),
            QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=depth))
    t0 = time.time()
    while time.time() - t0 < secs:
        rclpy.spin_once(node, timeout_sec=0.2)
    f.close()


if __name__ == '__main__':
    main()
