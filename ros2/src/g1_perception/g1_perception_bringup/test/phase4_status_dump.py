#!/usr/bin/python3
"""Dump /dpcbf/status (DiagnosticArray) to JSONL. argv: <secs> <out>"""
import json
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray


def main():
    secs, out = float(sys.argv[1]), sys.argv[2]
    rclpy.init()
    node = rclpy.create_node('phase4_status_dump')
    f = open(out, 'w')

    def on_msg(m):
        for st in m.status:
            f.write(json.dumps({
                'wall': time.time(), 'name': st.name, 'level': int.from_bytes(st.level, 'little') if isinstance(st.level, bytes) else int(st.level),
                'message': st.message,
                'kv': {kv.key: kv.value for kv in st.values}}) + '\n')
        f.flush()

    node.create_subscription(DiagnosticArray, '/dpcbf/status', on_msg, 10)
    t0 = time.time()
    while time.time() - t0 < secs:
        rclpy.spin_once(node, timeout_sec=0.2)
    f.close()


if __name__ == '__main__':
    main()
