"""Hardware sensor source (§12, §14.1): livox_ros_driver2 + DLIO.

Produces exactly the §7.1 contract that source_sim.launch.py produces —
/livox/lidar (+ /livox/imu), /odom and TF odom->base_link — so the shared
perception.launch.py stays conditional-free (D4). Every hardware-only
assumption lives in this file or in the two configs it loads.

Arguments
  driver:=on|off      run livox_ros_driver2 (off = replay a bag into DLIO)
  lio:=dlio|off       run DLIO (off = some other odom source, e.g. a bag)
  map:=off|on         also run DLIO's mapping node (extra CPU, not needed
                      for DPCBF: perception consumes odom + TF only)
  driver_config:=<path>  MID360_config.json override (default: our config)
  dlio_config:=<path>    DLIO parameter file override (default: our config).
                      Its extrinsics are DERIVED from g1_mid360.xacro and
                      guarded by t7_hw_extrinsic_guard.py; an override that
                      has not been through that guard is unguarded.

Not covered here, deliberately: DDS interface pinning (CYCLONEDDS_URI must
name the robot's LiDAR-facing NIC, §12.2 — process environment, not launch)
and use_sim_time, which is FALSE for every hardware node and is asserted
below rather than inherited.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_CONFIG = os.path.join(
    get_package_share_directory('g1_perception_bringup'), 'config')


def generate_launch_description():
    driver_config = LaunchConfiguration('driver_config')
    dlio_config = LaunchConfiguration('dlio_config')

    # livox_ros_driver2 (§5.6). Params: livox_driver.yaml + the JSON path,
    # injected here so the two cannot drift apart. The node publishes
    # /livox/lidar and /livox/imu as RELATIVE names -> keep it unnamespaced.
    livox = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        condition=LaunchConfigurationEquals('driver', 'on'),
        parameters=[
            os.path.join(_CONFIG, 'livox_driver.yaml'),
            {'user_config_path': driver_config,
             'use_sim_time': False},
        ],
    )

    # DLIO (§5.7, §12.3). Upstream's launch renames odom to
    # dlio/odom_node/odom; we publish the §7.1 name /odom instead and push the
    # rest of its chatter under /dlio/. DLIO has no TF listener: its notion of
    # base_link comes entirely from extrinsics/baselink2* in dlio.yaml.
    dlio_odom = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_odom_node',
        name='dlio_odom_node',
        output='screen',
        condition=LaunchConfigurationEquals('lio', 'dlio'),
        parameters=[dlio_config, {'use_sim_time': False}],
        remappings=[
            ('pointcloud', '/livox/lidar'),
            ('imu', '/livox/imu'),
            ('odom', '/odom'),
            ('pose', '/dlio/pose'),
            ('path', '/dlio/path'),
            ('kf_pose', '/dlio/keyframes'),
            ('kf_cloud', '/dlio/pointcloud/keyframe'),
            ('deskewed', '/dlio/pointcloud/deskewed'),
        ],
    )

    dlio_map = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_map_node',
        name='dlio_map_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('map')),
        parameters=[dlio_config, {'use_sim_time': False}],
        remappings=[('keyframes', '/dlio/pointcloud/keyframe')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('driver', default_value='on',
                              choices=['on', 'off']),
        DeclareLaunchArgument('lio', default_value='dlio',
                              choices=['dlio', 'off']),
        DeclareLaunchArgument('map', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument(
            'driver_config',
            default_value=os.path.join(_CONFIG, 'MID360_config.json'),
            description='Mid360 network config; IPs are Q-1 placeholders '
                        'until the robot session (see test/hw_config_check.py)'),
        DeclareLaunchArgument(
            'dlio_config',
            default_value=os.path.join(_CONFIG, 'dlio.yaml'),
            description='DLIO parameters; extrinsics are derived from '
                        'g1_mid360.xacro (t7_hw_extrinsic_guard.py)'),
        livox,
        dlio_odom,
        dlio_map,
    ])
