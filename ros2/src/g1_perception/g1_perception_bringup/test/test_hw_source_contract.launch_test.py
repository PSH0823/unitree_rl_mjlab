"""Hardware-source contract gate (Phase 5A, §7.1/§7.2).

The shared perception stack has only ever been fed the sim sidecar's cloud:
3 float32 fields, 12-byte stride, best-effort publisher, sim-time stamps. The
real driver produces a 7-field 26-byte #pragma pack(1) record with a FLOAT64
at an unaligned offset, publishes it RELIABLE, and stamps it with wall time.
This test runs the unmodified perception.launch.py against hw_source_stub.py
(the driver's output format, byte for byte) and asserts:

  * the whole chain to /obstacles_safe survives the hardware point layout,
  * every /livox/lidar subscriber is best-effort, so the driver's RELIABLE
    publisher matches it (a best-effort PUBLISHER would not have matched the
    Reliable subscribers upstream ships — that is patch 0003/0006's subject),
  * the §8.2 TF tree resolves with exactly one parent per frame,
  * nothing needs /clock: use_sim_time is false everywhere.

Odometry here is the stub's own motionless-but-dynamic odom->base_link (robot
at the origin, standing): this test is about the cloud seam, not about LIO.
DLIO is exercised separately by test_dlio_wiring.launch_test.py.

SKIPs when the fixture bag is absent (gitignored; see test_fixtures/README).

Standalone:
  launch_test src/g1_perception/g1_perception_bringup/test/test_hw_source_contract.launch_test.py
"""

# Concurrent launch tests share topic names; give this one a private DDS
# domain before anything else touches ROS (see isolate_domain.py).
# launch_test loads this file by path without putting its directory on
# sys.path, so the sibling import needs the insert first.
import os as _os, sys as _sys  # noqa: E402
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))  # noqa: E402
import isolate_domain  # noqa: E402
isolate_domain.isolate(isolate_domain.HW_SOURCE_CONTRACT)  # noqa: E402
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

DRIVER_POINT_STEP = 26
DRIVER_FIELDS = ['x', 'y', 'z', 'intensity', 'tag', 'line', 'timestamp']


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
        inc('perception.launch.py', use_sim_time='false'),
        # The stub also plays the odometry source (odom_hz:=100): robot at
        # the odom origin, upright, but a DYNAMIC transform — see the stub's
        # docstring for why static_transform_publisher cannot stand in here.
        launch.actions.TimerAction(
            period=4.0,
            actions=[launch.actions.ExecuteProcess(
                cmd=['/usr/bin/python3', STUB, '--ros-args',
                     '-p', f'bag:={BAG}', '-p', 'loop:=false',
                     '-p', 'imu_noise:=0.0', '-p', 'odom_hz:=100.0'],
                output='screen')]),
        launch_testing.actions.ReadyToTest(),
    ])


class TestHwSourceContract(unittest.TestCase):
    COLLECT_S = 34.0     # 4 s startup + 27.9 s of replay + margin

    @classmethod
    def setUpClass(cls):
        if SKIP_REASON:
            return
        import rclpy
        from rclpy.qos import qos_profile_sensor_data, QoSProfile
        from sensor_msgs.msg import LaserScan, PointCloud2
        from obstacle_detector.msg import Obstacles
        import tf2_ros

        rclpy.init()
        cls.node = rclpy.create_node('hw_contract_probe')
        cls.lock = threading.Lock()
        cls.clouds, cls.filtered, cls.scans = [], [], []
        cls.raw, cls.tracked, cls.safe = [], [], []
        cls.tf_buffer = tf2_ros.Buffer()
        cls.tf_listener = tf2_ros.TransformListener(cls.tf_buffer, cls.node)

        cls.node.create_subscription(
            PointCloud2, '/livox/lidar',
            lambda m: cls._push(cls.clouds, m), qos_profile_sensor_data)
        cls.node.create_subscription(
            PointCloud2, '/points_self_filtered',
            lambda m: cls._push(cls.filtered, m.width), qos_profile_sensor_data)
        cls.node.create_subscription(
            LaserScan, '/scan',
            lambda m: cls._push(cls.scans, m), qos_profile_sensor_data)
        rel = QoSProfile(depth=5)
        for topic, store in (('/raw_obstacles', cls.raw),
                             ('/tracked_obstacles', cls.tracked),
                             ('/obstacles_safe', cls.safe)):
            cls.node.create_subscription(
                Obstacles, topic,
                (lambda s: lambda m: cls._push(s, m))(store), rel)

        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        threading.Thread(target=cls.executor.spin, daemon=True).start()
        time.sleep(cls.COLLECT_S)
        # QoS snapshot while everything is still up
        cls.cloud_subs = cls.node.get_subscriptions_info_by_topic('/livox/lidar')
        cls.scan_pubs = cls.node.get_publishers_info_by_topic('/scan')
        cls.safe_pubs = cls.node.get_publishers_info_by_topic('/obstacles_safe')

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

    def test_driver_point_layout_on_the_wire(self):
        with self.lock:
            clouds = list(self.clouds)
        self.assertGreater(len(clouds), 100, 'stub produced too few clouds')
        c = clouds[0]
        self.assertEqual(c.header.frame_id, 'mid360_link', '§7.1 frame')
        self.assertEqual(c.point_step, DRIVER_POINT_STEP)
        self.assertEqual([f.name for f in c.fields], DRIVER_FIELDS)
        print(f'/livox/lidar: {len(clouds)} clouds, point_step {c.point_step}, '
              f'fields {[f.name for f in c.fields]}, width {c.width}')

    def test_chain_survives_hardware_layout(self):
        """§7.2: perception must consume x,y,z and tolerate the extra fields."""
        with self.lock:
            n = (len(self.clouds), len(self.filtered), len(self.scans),
                 len(self.raw), len(self.tracked), len(self.safe))
        print('cloud/filtered/scan/raw/tracked/safe =', n)
        self.assertGreater(n[1], 100, 'CropBox produced nothing from the '
                                      'hardware point layout')
        self.assertGreater(n[2], 100, 'no /scan from the hardware layout')
        self.assertGreater(n[3], 100, 'no /raw_obstacles')
        self.assertGreater(n[4], 100, 'no /tracked_obstacles')
        self.assertGreater(n[5], 100, 'no /obstacles_safe')

    def test_scan_sanity(self):
        with self.lock:
            scans = list(self.scans)
        first = scans[0]
        self.assertEqual(first.header.frame_id, 'base_footprint')
        self.assertAlmostEqual(first.angle_increment, 0.0058, places=9)

    def test_qos_wire_check_driver_path(self):
        """Every /livox/lidar subscriber must be best-effort.

        The driver publishes RELIABLE and offers no QoS parameter, so
        Reliable-pub/BestEffort-sub is the only match that exists. A Reliable
        subscriber would also match here but would NOT match the sim sidecar
        or any bag recorded from it — the single-contract rule (§7.1) is what
        keeps one perception.launch.py valid in both worlds.
        """
        from rclpy.qos import QoSReliabilityPolicy
        self.assertTrue(self.cloud_subs, 'nobody subscribed to /livox/lidar')
        for e in self.cloud_subs:
            print(f'/livox/lidar sub {e.node_name}: '
                  f'{e.qos_profile.reliability.name}')
            self.assertEqual(e.qos_profile.reliability,
                             QoSReliabilityPolicy.BEST_EFFORT,
                             f'{e.node_name} subscribes /livox/lidar Reliable')
        for e in self.scan_pubs:
            self.assertEqual(e.qos_profile.reliability,
                             QoSReliabilityPolicy.BEST_EFFORT,
                             '/scan publisher must be SensorData (§7.1)')
        for e in self.safe_pubs:
            self.assertEqual(e.qos_profile.reliability,
                             QoSReliabilityPolicy.RELIABLE,
                             '/obstacles_safe must be Reliable (§7.1)')

    def test_tf_tree_shape(self):
        """§8.2 tree resolves and no frame has two parents."""
        import rclpy
        from rclpy.time import Time
        for parent, child in (('odom', 'base_link'),
                              ('base_link', 'torso_link'),
                              ('torso_link', 'mid360_link'),
                              ('odom', 'base_footprint'),
                              ('odom', 'mid360_link')):
            self.assertTrue(
                self.tf_buffer.can_transform(parent, child, Time()),
                f'TF {parent} -> {child} does not resolve')
        yaml_tree = self.tf_buffer.all_frames_as_yaml()
        print(yaml_tree)
        import yaml as _yaml
        frames = _yaml.safe_load(yaml_tree) or {}
        for frame, info in frames.items():
            self.assertIsInstance(info.get('parent'), str,
                                  f'{frame} has no single parent')
        self.assertEqual(frames.get('mid360_link', {}).get('parent'),
                         'torso_link',
                         'mid360_link must hang off torso_link only')
