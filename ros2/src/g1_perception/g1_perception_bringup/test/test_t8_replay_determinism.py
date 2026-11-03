#!/usr/bin/python3
"""T8 replay-determinism HARD gate (§16.2, Phase 3 — promoted from
"measured" to CTest because P-2 landed): the s1_surveyed fixture is replayed
through the perception container twice; the /raw_obstacles and
/tracked_obstacles streams must be identical (stamps + full circle content).

Runs two sequential container sessions via subprocess (a single
launch_testing description cannot restart the container), so it is wired
into CTest as a plain test with SKIP_RETURN_CODE 77 when the fixture bag or
a built workspace is missing.
"""
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..'))
BAG = os.path.join(WS, 'test_fixtures', 's1_surveyed')
SKIP = 77


def kill_containers(sig):
    subprocess.run(['pkill', f'-{sig}', '-f',
                    'rclcpp_components/component_containe[r]'],
                   check=False, capture_output=True)


def one_run(out_path, collect_s):
    probe = subprocess.Popen(
        [sys.executable, os.path.join(HERE, 't8_dump_probe.py'),
         str(collect_s), out_path])
    launch = subprocess.Popen(
        ['ros2', 'launch', 'g1_perception_bringup', 'perception.launch.py',
         'use_sim_time:=true'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        time.sleep(6.0)
        subprocess.run(['ros2', 'bag', 'play', BAG], check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        probe.wait(timeout=collect_s + 30)
    finally:
        launch.send_signal(signal.SIGINT)
        try:
            launch.wait(timeout=10)
        except subprocess.TimeoutExpired:
            launch.kill()
        # orphaned-twin discipline (Phase-2 finding)
        kill_containers('INT')
        time.sleep(1.0)
        kill_containers('KILL')


def key(msg):
    return (round(msg['t'], 9), msg['frame'],
            tuple(sorted((c['x'], c['y'], c['r'], c['tr'], c['vx'], c['vy'])
                         for c in msg['circles'])), msg['nseg'])


def load(path, topic):
    with open(path) as f:
        return [key(r) for r in map(json.loads, f) if r['topic'] == topic]


def main():
    if not os.path.isdir(BAG):
        print(f'SKIP: fixture bag missing: {BAG}')
        return SKIP
    try:
        subprocess.run(['ros2', 'pkg', 'prefix', 'g1_perception_bringup'],
                       check=True, capture_output=True)
    except Exception:
        print('SKIP: workspace not sourced/built')
        return SKIP

    collect_s = 40.0  # bag is ~30 s + margin

    tmp = tempfile.mkdtemp(prefix='t8_')
    runs = [os.path.join(tmp, f'run{i}.jsonl') for i in (1, 2)]
    for p in runs:
        one_run(p, collect_s)

    ok = True
    for topic in ('/raw_obstacles', '/tracked_obstacles'):
        a, b = load(runs[0], topic), load(runs[1], topic)
        same = a == b and len(a) > 0
        print(f'{topic}: run1 {len(a)} msgs, run2 {len(b)} msgs -> '
              f'{"IDENTICAL" if same else "MISMATCH"}')
        ok = ok and same
    if not ok:
        print('T8 FAIL: replay is not deterministic')
        return 1
    print('T8 PASS: both obstacle streams bitwise-identical across replays')
    return 0


if __name__ == '__main__':
    sys.exit(main())
