#!/usr/bin/python3
"""Mini-sim scenario driver (Phase 3): generalizes wall_state_source.py to
scripted mocap trajectories with ground truth.

Publishes, all derived from one sim-time counter (RT factor 1):
  /clock             200 Hz
  /sim/mj_state      100 Hz  (grounded qpos + scripted mocap_pos/quat)
  /sim/gt_obstacles   50 Hz  (obstacle_detector/Obstacles, odom frame,
                              Reliable depth 1 per §7.1; uid = mocap index,
                              velocity = commanded trajectory velocity)

Scenario json comes from test_fixtures/scenarios/make_scenario_scene.py.
Exits after the scenario duration unless --loop is given.

Runs under system python (R-11) with rclpy.
"""
import argparse
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time as TimeMsg

from obstacle_detector.msg import Obstacles, CircleObstacle
from sim_msgs.msg import MjState

TICK = 0.005  # 200 Hz wall pacing, sim time advances at RT factor 1


def obstacle_state(ob, t, park):
    """Return ([x, y], [vx, vy]) for obstacle `ob` at sim time `t`."""
    if ob['mode'] == 'static':
        return list(ob['pos']), [0.0, 0.0]
    if ob['mode'] == 'cross':
        p0, p1, v = ob['p0'], ob['p1'], ob['speed']
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        length = (dx * dx + dy * dy) ** 0.5
        ux, uy = dx / length, dy / length
        s = max(0.0, min(length, v * (t - ob['t_start'])))
        moving = 0.0 < s < length
        return ([p0[0] + ux * s, p0[1] + uy * s],
                [v * ux, v * uy] if moving else [0.0, 0.0])
    raise ValueError(f"unknown obstacle mode {ob['mode']}")


class ScenarioStateSource(Node):
    def __init__(self, scenario, loop):
        super().__init__('scenario_state_source')
        self.sc = scenario
        self.loop = loop
        self.qpos = [float(v) for v in scenario['qpos']]
        self.half_h = scenario['half_height_m']
        self.radius = scenario['radius_m']
        self.park = scenario['park']
        self.by_body = {ob['body']: ob for ob in scenario['obstacles']}
        self.sim_time = 0.0
        self.tick_count = 0
        self.done = False
        reliable1 = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                               history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.clock_pub = self.create_publisher(Clock, '/clock', reliable1)
        self.state_pub = self.create_publisher(
            MjState, '/sim/mj_state',
            QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=1))
        self.gt_pub = self.create_publisher(
            Obstacles, '/sim/gt_obstacles', reliable1)
        self.timer = self.create_timer(TICK, self.on_tick)

    def stamp(self):
        sec = int(self.sim_time)
        return TimeMsg(sec=sec,
                       nanosec=int(round((self.sim_time - sec) * 1e9)))

    def on_tick(self):
        self.sim_time += TICK
        self.tick_count += 1
        if self.sim_time >= self.sc['duration_s']:
            if self.loop:
                self.sim_time = 0.0
            else:
                self.done = True
                return
        self.clock_pub.publish(Clock(clock=self.stamp()))

        if self.tick_count % 2 == 0:  # 100 Hz
            msg = MjState()
            msg.sim_time = self.sim_time
            msg.qpos = self.qpos
            mocap_pos, mocap_quat = [], []
            for body in self.sc['mocap_bodies']:
                ob = self.by_body.get(body)
                if ob is None:
                    xy = self.park
                else:
                    xy, _ = obstacle_state(ob, self.sim_time, self.park)
                mocap_pos += [xy[0], xy[1], self.half_h]
                mocap_quat += [1.0, 0.0, 0.0, 0.0]
            msg.mocap_pos = mocap_pos
            msg.mocap_quat = mocap_quat
            self.state_pub.publish(msg)

        if self.tick_count % 4 == 0:  # 50 Hz
            gt = Obstacles()
            gt.header.stamp = self.stamp()
            gt.header.frame_id = 'odom'
            for i, body in enumerate(self.sc['mocap_bodies']):
                ob = self.by_body.get(body)
                if ob is None:
                    continue
                xy, vel = obstacle_state(ob, self.sim_time, self.park)
                c = CircleObstacle()
                c.uid = i
                c.center.x, c.center.y, c.center.z = xy[0], xy[1], 0.0
                c.velocity.x, c.velocity.y = vel[0], vel[1]
                c.radius = self.radius
                c.true_radius = self.radius
                gt.circles.append(c)
            self.gt_pub.publish(gt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario-json', required=True)
    ap.add_argument('--loop', action='store_true')
    args, ros_args = ap.parse_known_args()
    with open(args.scenario_json) as f:
        scenario = json.load(f)
    rclpy.init(args=ros_args)
    node = ScenarioStateSource(scenario, args.loop)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
