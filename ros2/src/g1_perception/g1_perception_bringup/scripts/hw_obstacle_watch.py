#!/usr/bin/python3
"""Live obstacle read-out in the ROBOT frame (Phase 5C, stage 10 case A).

Answers, on the console, the one question stage 10 actually asks — *what does
perception think is out there, where, and how big* — without a bag, without a
replay and without RViz:

    /tracked_obstacles  base_link   2 circles
      uid    x       y     range   bearing      r    true_r    |v|
        7   1.02   -0.03    1.02     -1.7deg  0.201   0.150   0.01

Why a separate tool rather than `ros2 topic echo /tracked_obstacles`:

  * the pipeline publishes obstacles in **odom** (`obstacle_detector.yaml`
    frame_id, both extractor and tracker). An echo therefore prints numbers
    the operator cannot compare against a tape measure taken from the robot's
    feet. This node does the tf2 lookup — at the message stamp, not `latest`
    — and prints x/y/range/bearing in `base_link` (or any frame given);
  * `radius` and `true_radius` are different numbers with different meanings
    (margin-added vs measured, and §9.6 inflates again downstream). Echo shows
    both without saying which stream you are looking at; the table labels the
    stream and shows both columns side by side so the stage-10 success
    criterion — `true_radius` error on **/tracked_obstacles**, pre-inflation —
    can be read off directly;
  * with `layout:=<t4_layout.yaml>` it scores the frame against the surveyed
    props live, so a mis-survey or a dropped prop is visible while the props
    are still on the floor, instead of at the offline harness the next day.

Velocity is rotated, never subtracted: `|v|` and vx/vy are the obstacle's
velocity **in odom**, expressed in the robot's axes. A static prop reads ~0
even while the robot walks. That is deliberate — it is what the tracker
estimates and what §9.6 extrapolates with.

Publishes nothing. Subscribes to /tf, /tf_static and the obstacle topics
only; it cannot move the robot.

Usage
  ros2 run g1_perception_bringup hw_obstacle_watch.py
  ros2 run g1_perception_bringup hw_obstacle_watch.py --ros-args \
      -p target_frame:=base_footprint -p rate:=1.0 \
      -p layout:=$SESSION/t4_layout.yaml -p duration:=60.0 \
      -p json:=$SESSION/stage10_watch.jsonl

Parameters
  topics        obstacle topics to watch, in print order
                (default: /raw_obstacles /tracked_obstacles /obstacles_safe)
  target_frame  frame the table is printed in (default base_link)
  rate          table refresh, Hz (default 2.0)
  tf_timeout    tf2 wait at the message stamp, s (default 0.05)
  layout        t4_layout.yaml with surveyed odom-frame targets; '' = off
  duration      seconds then exit; 0 = until Ctrl+C
  json          JSONL of every printed snapshot; '' = off

Exit 0 clean, 2 if no obstacle message ever arrived on any topic.
"""
import json
import math
import sys
import time

import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy)
from rclpy.time import Time

from obstacle_detector.msg import Obstacles

DEFAULT_TOPICS = ['/raw_obstacles', '/tracked_obstacles', '/obstacles_safe']

HEAD = ('    uid       x       y   range  bearing       r  true_r      vx'
        '      vy')
ROW = ('  {uid:>5}  {x:6.2f}  {y:6.2f}  {rng:6.2f}  {brg:6.1f}  {r:6.3f}'
       '  {tr:6.3f}  {vx:6.2f}  {vy:6.2f}')


def _quat_rotate(q, v):
    """Rotate v=(x,y,z) by the geometry_msgs quaternion q. Written out rather
    than pulled from tf2_geometry_msgs: that package's Python bindings are an
    optional install on an onboard PC and this node must not fail to start on
    a robot for the sake of one 8-line function."""
    x, y, z = v
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (x + qw * tx + (qy * tz - qz * ty),
            y + qw * ty + (qz * tx - qx * tz),
            z + qw * tz + (qx * ty - qy * tx))


def _load_layout(path):
    """t4_layout.yaml -> (match_radius, [{name,x,y,r}]). Same file the offline
    T4 harness eats, same odom-frame convention (§ stage 10 survey)."""
    import yaml
    with open(path) as f:
        doc = yaml.safe_load(f) or {}
    targets = []
    for t in doc.get('targets', []):
        targets.append({'name': str(t.get('name', '?')),
                        'x': float(t['x']), 'y': float(t['y']),
                        'r': float(t.get('r', 0.0))})
    return float(doc.get('match_radius', 0.5)), targets


class ObstacleWatch(Node):
    def __init__(self):
        super().__init__('hw_obstacle_watch')
        self.topics = list(self.declare_parameter(
            'topics', DEFAULT_TOPICS).value)
        self.target_frame = self.declare_parameter(
            'target_frame', 'base_link').value
        self.rate = float(self.declare_parameter('rate', 2.0).value)
        self.tf_timeout = float(
            self.declare_parameter('tf_timeout', 0.05).value)
        self.duration = float(self.declare_parameter('duration', 0.0).value)
        self.json_path = self.declare_parameter('json', '').value
        layout = self.declare_parameter('layout', '').value

        self.match_radius, self.targets = (
            _load_layout(layout) if layout else (0.5, []))
        if layout:
            self.get_logger().info(
                'layout %s: %d surveyed target(s), match_radius %.2f m'
                % (layout, len(self.targets), self.match_radius))

        self.buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        # spin_thread=True on purpose: the lookups below happen inside a timer
        # callback, so with a single-threaded listener the `tf_timeout` wait
        # would block the very executor that has to deliver /tf — the timeout
        # could only ever expire. A dedicated listener thread makes it mean
        # what it says.
        self.listener = tf2_ros.TransformListener(self.buffer, self,
                                                  spin_thread=True)
        # last message + arrival wall-clock + a message counter per topic, so
        # the header can show rate and staleness. A topic that is advertised
        # but silent must look different from one that is publishing zero
        # circles — those are different faults (dead node vs nothing detected).
        self.last = {t: None for t in self.topics}
        self.at = {t: 0.0 for t in self.topics}
        self.count = {t: 0 for t in self.topics}
        self.t0 = time.time()
        self.jf = open(self.json_path, 'w') if self.json_path else None

        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        for t in self.topics:
            self.create_subscription(
                Obstacles, t, lambda m, tt=t: self._on_msg(tt, m), qos)
        self.create_timer(1.0 / max(self.rate, 0.1), self._print)

    def _on_msg(self, topic, msg):
        self.last[topic] = msg
        self.at[topic] = time.time()
        self.count[topic] += 1

    # -- geometry ---------------------------------------------------------
    def _tf(self, source_frame, stamp):
        """(transform, note) for target<-source at `stamp`. Falls back to the
        latest available transform and SAYS SO — silently using `latest` is
        how a frozen TF tree reads as a healthy detection table."""
        try:
            return self.buffer.lookup_transform(
                self.target_frame, source_frame, stamp,
                timeout=Duration(seconds=self.tf_timeout)), ''
        except tf2_ros.TransformException as exc:
            try:
                return self.buffer.lookup_transform(
                    self.target_frame, source_frame, Time()), '(TF: latest)'
            except tf2_ros.TransformException:
                return None, '(TF FAILED: %s)' % str(exc).split('\n')[0]

    def _project(self, msg):
        """[(uid, x, y, r, true_r, vx, vy)] in target_frame + a note."""
        src = msg.header.frame_id
        if src == self.target_frame:
            tf, note = None, ''
        else:
            tf, note = self._tf(src, Time.from_msg(msg.header.stamp))
            if tf is None:
                return [], note
        rows = []
        for c in msg.circles:
            if tf is None:
                x, y = c.center.x, c.center.y
                vx, vy = c.velocity.x, c.velocity.y
            else:
                q, tr = tf.transform.rotation, tf.transform.translation
                px, py, _ = _quat_rotate(q, (c.center.x, c.center.y,
                                             c.center.z))
                x, y = px + tr.x, py + tr.y
                # velocity is a free vector: rotation only, no translation.
                vx, vy, _ = _quat_rotate(q, (c.velocity.x, c.velocity.y,
                                             c.velocity.z))
            rows.append((c.uid, x, y, c.radius, c.true_radius, vx, vy))
        return rows, note

    # -- survey scoring ---------------------------------------------------
    def _score(self, msg):
        """Greedy nearest-target match in the message's OWN (odom) frame —
        the surveyed layout is odom-frame, so scoring there avoids folding the
        odometry-to-base_link transform's own error into the detector error
        this stage is trying to measure."""
        free = list(msg.circles)
        lines = []
        for t in self.targets:
            best, best_d = None, 1e9
            for c in free:
                d = math.hypot(c.center.x - t['x'], c.center.y - t['y'])
                if d < best_d:
                    best, best_d = c, d
            if best is not None and best_d <= self.match_radius:
                free.remove(best)
                lines.append('  %-12s HIT   centre err %6.3f m   '
                             'true_r %.3f (err %+0.3f)   r %.3f'
                             % (t['name'], best_d, best.true_radius,
                                best.true_radius - t['r'], best.radius))
            else:
                # "nearest UNMATCHED circle": earlier targets have already
                # consumed theirs, so an empty list here means every detection
                # is spoken for, NOT that the frame was empty.
                lines.append('  %-12s MISS  (nearest unmatched circle: %s)'
                             % (t['name'],
                                '%.3f m' % best_d if best is not None
                                else 'none left'))
        for c in free:
            lines.append('  %-12s EXTRA uid %d at odom (%.2f, %.2f) r %.3f'
                         % ('-', c.uid, c.center.x, c.center.y, c.true_radius))
        return lines

    # -- output -----------------------------------------------------------
    def _print(self):
        now = time.time()
        if self.duration > 0.0 and now - self.t0 >= self.duration:
            raise SystemExit(0 if any(self.count.values()) else 2)
        out = ['', '=== t+%6.1f s   frame=%s   (source: obstacle_detector '
                   'publishes in odom)' % (now - self.t0, self.target_frame)]
        snapshot = {'t': now - self.t0, 'frame': self.target_frame,
                    'topics': {}}
        for topic in self.topics:
            msg = self.last[topic]
            if msg is None:
                out.append('--- %-20s NO DATA (%d msgs)' % (topic,
                                                            self.count[topic]))
                continue
            age = now - self.at[topic]
            hz = self.count[topic] / max(now - self.t0, 1e-6)
            rows, note = self._project(msg)
            out.append('--- %-20s %5.1f Hz  age %4.2f s  %d circle(s) %s'
                       % (topic, hz, age, len(msg.circles), note))
            if rows:
                out.append(HEAD)
            for uid, x, y, r, tr, vx, vy in rows:
                out.append(ROW.format(
                    uid=uid, x=x, y=y, rng=math.hypot(x, y),
                    brg=math.degrees(math.atan2(y, x)), r=r, tr=tr,
                    vx=vx, vy=vy))
            snapshot['topics'][topic] = {
                'hz': hz, 'age': age, 'note': note,
                'circles': [{'uid': u, 'x': x, 'y': y, 'r': r, 'true_r': tr,
                             'vx': vx, 'vy': vy}
                            for u, x, y, r, tr, vx, vy in rows]}
            if self.targets and topic == '/tracked_obstacles':
                out.append('  -- vs surveyed layout (odom frame) --')
                out.extend(self._score(msg))
        print('\n'.join(out), flush=True)
        if self.jf:
            self.jf.write(json.dumps(snapshot) + '\n')
            self.jf.flush()


def main():
    rclpy.init()
    node = ObstacleWatch()
    code = 0
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit) as exc:
        code = getattr(exc, 'code', 0) or 0
    finally:
        # Stop the listener's own executor BEFORE rclpy shuts down, or its
        # thread dies inside spin() and dumps an ExternalShutdownException
        # traceback over the last table the operator wanted to read.
        try:
            node.listener.executor.shutdown()
        except Exception:
            pass
        if node.jf:
            node.jf.close()
        if not any(node.count.values()):
            print('\nNO OBSTACLE MESSAGE on %s — check /scan first '
                  '(stage 9 territory).' % ', '.join(node.topics),
                  file=sys.stderr)
            code = code or 2
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
