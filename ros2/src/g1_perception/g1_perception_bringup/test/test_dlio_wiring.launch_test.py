"""DLIO wiring smoke (Phase 5A, §5.7/§8.2/§12.3).

Runs source_hw.launch.py with the driver off and hw_source_stub.py standing
in for it, so DLIO is exercised on the real topic names, the real cloud
format and a real (if synthetic) IMU stream before robot day. It asserts
WIRING, never odometry quality: the clouds are replayed sim geometry and the
IMU says "stationary" the whole time, so the pose it computes is meaningless
by construction. Drift is a 5B measurement (§18 gate: <1 cm/min).

What would have failed here without the Phase-5A patches/config:
  * patch 0006 — upstream subscribes `pointcloud` RELIABLE; against any
    best-effort cloud (sim sidecar, every bag from it, and the stub) DLIO
    shows a connected topic and receives nothing.
  * dlio.yaml frames/{lidar,imu} — upstream names them `lidar`/`imu`; naming
    either one mid360_link gives that frame a second parent, because DLIO
    broadcasts base_link -> <frames/lidar> unconditionally.
  * dlio.yaml use_sim_time — upstream cfg/params.yaml ships TRUE; on
    hardware there is no /clock and the 100 Hz publish timer never fires.

SKIPs when the fixture bag is absent (gitignored; see test_fixtures/README).

Standalone:
  launch_test src/g1_perception/g1_perception_bringup/test/test_dlio_wiring.launch_test.py
"""

# Concurrent launch tests share topic names; give this one a private DDS
# domain before anything else touches ROS (see isolate_domain.py).
# launch_test loads this file by path without putting its directory on
# sys.path, so the sibling import needs the insert first.
import os as _os, sys as _sys  # noqa: E402
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))  # noqa: E402
import isolate_domain  # noqa: E402
isolate_domain.isolate(isolate_domain.DLIO_WIRING)  # noqa: E402
import os
import threading
import time
import unittest

import launch
import launch_testing.actions
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..'))
BAG = os.path.join(WS, 'test_fixtures', 's1_static_reference')
STUB = os.path.join(HERE, 'hw_source_stub.py')

SKIP_REASON = None
if not os.path.isdir(BAG):
    SKIP_REASON = f'fixture bag missing: {BAG} (gitignored; see README)'

ODOM_MIN_HZ = 50.0    # §7.1


@pytest.mark.launch_test
def generate_test_description():
    if SKIP_REASON:
        return launch.LaunchDescription([launch_testing.actions.ReadyToTest()])
    from ament_index_python.packages import get_package_share_directory
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    launch_dir = os.path.join(
        get_package_share_directory('g1_perception_bringup'), 'launch')
    inc = lambda f, **a: launch.actions.IncludeLaunchDescription(  # noqa: E731
        PythonLaunchDescriptionSource(os.path.join(launch_dir, f)),
        launch_arguments=list(a.items()))
    return launch.LaunchDescription([
        inc('description.launch.py', use_sim_time='false'),
        inc('source_hw.launch.py', driver='off', lio='dlio', map='false'),
        launch.actions.TimerAction(
            period=3.0,
            actions=[launch.actions.ExecuteProcess(
                cmd=['/usr/bin/python3', STUB, '--ros-args',
                     '-p', f'bag:={BAG}', '-p', 'loop:=true',
                     '-p', 'imu_noise:=0.0005'],
                output='screen')]),
        launch_testing.actions.ReadyToTest(),
    ])


class TestDlioWiring(unittest.TestCase):
    COLLECT_S = 25.0    # 3 s startup + 3 s IMU calibration + margin

    @classmethod
    def setUpClass(cls):
        if SKIP_REASON:
            return
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.qos import QoSProfile
        import tf2_ros

        rclpy.init()
        cls.node = rclpy.create_node('dlio_wiring_probe')
        cls.lock = threading.Lock()
        cls.odoms = []
        cls.tf_buffer = tf2_ros.Buffer()
        cls.tf_listener = tf2_ros.TransformListener(cls.tf_buffer, cls.node)
        cls.tf_odom_base = []
        cls.node.create_subscription(
            Odometry, '/odom',
            lambda m: cls._push(cls.odoms, (time.monotonic(), m.header.stamp,
                                            m.header.frame_id,
                                            m.child_frame_id)),
            QoSProfile(depth=10))
        from tf2_msgs.msg import TFMessage
        cls.node.create_subscription(
            TFMessage, '/tf',
            lambda m: [cls._push(cls.tf_odom_base, time.monotonic())
                       for t in m.transforms
                       if t.header.frame_id == 'odom'
                       and t.child_frame_id == 'base_link'],
            QoSProfile(depth=200))
        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        threading.Thread(target=cls.executor.spin, daemon=True).start()
        time.sleep(cls.COLLECT_S)
        cls.cloud_subs = cls.node.get_subscriptions_info_by_topic('/livox/lidar')
        cls.imu_subs = cls.node.get_subscriptions_info_by_topic('/livox/imu')
        cls.odom_pubs = cls.node.get_publishers_info_by_topic('/odom')
        cls.node_names = cls.node.get_node_names()

    @classmethod
    def _push(cls, store, item):
        with cls.lock:
            store.append(item)

    @classmethod
    def tearDownClass(cls):
        if SKIP_REASON:
            return
        import rclpy
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        if SKIP_REASON:
            self.skipTest(SKIP_REASON)

    def test_dlio_running_and_subscribed(self):
        self.assertIn('dlio_odom_node', self.node_names)
        subs = {e.node_name for e in self.cloud_subs}
        self.assertIn('dlio_odom_node', subs,
                      '/livox/lidar has no DLIO subscriber')
        self.assertIn('dlio_odom_node',
                      {e.node_name for e in self.imu_subs})

    def test_dlio_cloud_subscription_is_best_effort(self):
        """Patch 0006. Reliable here silently receives nothing from the
        best-effort publishers this architecture uses (§7.1)."""
        from rclpy.qos import QoSReliabilityPolicy
        e = next(e for e in self.cloud_subs if e.node_name == 'dlio_odom_node')
        print('dlio /livox/lidar sub reliability:',
              e.qos_profile.reliability.name)
        self.assertEqual(e.qos_profile.reliability,
                         QoSReliabilityPolicy.BEST_EFFORT)

    def test_odom_contract(self):
        with self.lock:
            odoms = list(self.odoms)
        self.assertGreater(len(odoms), 200, 'DLIO published almost no /odom')
        # Wall-clock arrival rate: header stamps are DLIO's imu_stamp, which
        # is 0 until the first IMU message arrives, so a stamp-span rate is
        # meaningless over the startup window.
        wall = [w for w, _, _, _ in odoms]
        hz = (len(wall) - 1) / (wall[-1] - wall[0])
        print(f'/odom: {len(odoms)} msgs -> {hz:.1f} Hz (wall)')
        self.assertGreaterEqual(hz, ODOM_MIN_HZ, '§7.1 wants >=50 Hz')
        self.assertEqual(odoms[-1][2], 'odom')
        self.assertEqual(odoms[-1][3], 'base_link')
        from rclpy.qos import QoSReliabilityPolicy
        for e in self.odom_pubs:
            self.assertEqual(e.qos_profile.reliability,
                             QoSReliabilityPolicy.RELIABLE)

    def test_tf_rate_is_scan_rate_not_odom_rate(self):
        """Recorded Phase-5A seam finding, asserted so it cannot regress
        silently.

        DLIO splits its output: publishPose() runs on a 100 Hz wall timer and
        emits ONLY /odom and /pose; all three TF broadcasts live in
        publishToROS(), which is spawned as a thread once per scan. So on
        hardware TF odom->base_link arrives at SCAN rate (~10 Hz), while
        /odom is 100 Hz — and base_footprint_publisher, which is driven by
        that TF, inherits the 10 Hz. §7.1's "TF odom->base_footprint 100 Hz"
        row describes the sim sidecar, not hardware.

        Measured consequence (dev machine, this rig): cloud -> /scan p50
        9.2 ms / p95 12.8 ms and cloud -> /obstacles_safe p95 13.2 ms, versus
        0.5/1.0 ms in sim — the tf2 MessageFilter waits for the bracketing
        TF sample. Still far inside the §17.2 60 ms budget, so the obvious
        patch (broadcast TF from publishPose too) is NOT taken; it is
        recorded as conditional patch P-5, to fire only if T9-hardware or the
        Orin latency benchmark says the wait grows.
        """
        with self.lock:
            tf_t, odom_t = list(self.tf_odom_base), [w for w, _, _, _ in self.odoms]
        self.assertGreater(len(tf_t), 20, 'no odom->base_link TF at all')
        tf_hz = (len(tf_t) - 1) / (tf_t[-1] - tf_t[0])
        odom_hz = (len(odom_t) - 1) / (odom_t[-1] - odom_t[0])
        print(f'TF odom->base_link {tf_hz:.1f} Hz vs /odom {odom_hz:.1f} Hz')
        self.assertLess(tf_hz, odom_hz / 2.0,
                        'TF now tracks /odom — publishToROS/publishPose split '
                        'changed upstream; re-derive the latency numbers and '
                        'update this docstring')
        self.assertGreater(tf_hz, 5.0, 'TF slower than the 10 Hz scan rate')

    def test_tf_tree_has_no_frame_collision(self):
        """DLIO broadcasts base_link -> frames/lidar and -> frames/imu on top
        of robot_state_publisher's tree; those names must be disjoint."""
        import yaml as _yaml
        from rclpy.time import Time
        self.assertTrue(self.tf_buffer.can_transform('odom', 'base_link', Time()),
                        'DLIO is not publishing odom -> base_link')
        tree = _yaml.safe_load(self.tf_buffer.all_frames_as_yaml()) or {}
        print(tree)
        self.assertEqual(tree.get('mid360_link', {}).get('parent'),
                         'torso_link',
                         'mid360_link must have exactly one parent')
        self.assertEqual(tree.get('base_link', {}).get('parent'), 'odom')
        for f in ('dlio_lidar_link', 'dlio_imu_link'):
            self.assertEqual(tree.get(f, {}).get('parent'), 'base_link',
                             f'{f} missing — frames/* renaming regressed')

    def test_dlio_use_sim_time_is_false(self):
        """Upstream cfg/params.yaml ships use_sim_time: true."""
        import subprocess
        out = subprocess.run(
            ['ros2', 'param', 'get', '/dlio_odom_node', 'use_sim_time'],
            capture_output=True, text=True, timeout=20).stdout
        print('dlio use_sim_time ->', out.strip())
        self.assertIn('False', out)
