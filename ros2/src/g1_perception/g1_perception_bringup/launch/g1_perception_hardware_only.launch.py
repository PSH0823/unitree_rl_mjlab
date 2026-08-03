"""PERCEPTION-ONLY hardware bring-up (Phase 5C, stage 12).

    ros2 launch g1_perception_bringup g1_perception_hardware_only.launch.py

Brings up, on a real G1 with a real Mid-360:

    livox_ros_driver2  ->  /livox/lidar, /livox/imu
    DLIO               ->  /odom, TF odom->base_link
    robot_state_publisher + base_footprint_publisher  ->  the rest of §8.2
    perception container  ->  CropBox -> (VoxelGrid) -> /scan -> extractor
                             -> tracker -> safety filter -> /obstacles_safe
    hw_diagnostics     ->  /diagnostics
    optionally RViz and a metadata-stamped rosbag

and NOTHING ELSE. Specifically it does not, and structurally cannot:

  * start `g1_ctrl`, the `deploy` FSM, or any Unitree binary — no node in the
    graph below comes from a package that links unitree_sdk2;
  * publish a velocity, low-level or joystick command — the only publishers
    are the perception topics above, /tf, /diagnostics and RViz's own;
  * construct a DPCBF control seam — `dpcbf_ros_adapter` is a LIBRARY, it is
    not a node, and nothing here loads it. /obstacles_safe is published and
    consumed by nobody;
  * change FSM state or send a joystick chord.

`test/test_hw_offline_gates.py` asserts every clause of that paragraph
against this file and everything it includes, so the guarantee is checked on
each build rather than believed.

Why a separate file rather than `bringup.launch.py source:=hw`: bringup
carries `mode:=oracle|shadow|estimated` and `ground_seg`, both of which are
DPCBF/segmentation vocabulary that does not apply to a perception-only
session, and its `viz` path defaults the DPCBF overlay ON. Keeping the
hardware-only entry point argument-poor is the isolation guarantee — there is
no argument here that can turn actuation on, because none exists.

Arguments
  use_rviz:=false|true    RViz2 with the committed layout, overlay OFF
  record:=off|on          rosbag2 + session metadata (record_hw.launch.py)
  voxel:=off|on           §9.3 VoxelGrid between CropBox and projection
  lidar_config:=<path>    MID360_config.json (default: the installed one)
  dlio_config:=<path>     DLIO parameter file (default: the installed one)
  rviz_config:=<path>     RViz layout
  bag_path:=<path>        where record:=on writes
  driver:=on|off          off = replay a bag into DLIO instead of the device
  lio:=dlio|off           off = odometry comes from somewhere else
  diagnostics:=on|off

There is deliberately NO `ground_seg` argument. There is no ground
segmentation in this stack: `min_height` in pointcloud_to_laserscan is the
only thing that rejects floor returns, Patchwork++ is not imported, not
built, not launched, and `/points_no_ground` does not exist. `bringup.launch.py`
still accepts the argument for its sim history and now rejects
`ground_seg:=patchwork` loudly instead of ignoring it.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_SHARE = get_package_share_directory('g1_perception_bringup')
_CONFIG = os.path.join(_SHARE, 'config')
_LAUNCH = os.path.join(_SHARE, 'launch')


def _include(name, condition=None, **launch_args):
    kwargs = {'launch_arguments': list(launch_args.items())} if launch_args else {}
    if condition is not None:
        kwargs['condition'] = condition
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(_LAUNCH, name)), **kwargs)


def generate_launch_description():
    return LaunchDescription([
        # use_sim_time is FALSE and is not an argument. There is no /clock on
        # hardware; a node left true never advances its clock and its timers
        # never fire. Making it configurable here would only make that
        # failure reachable.
        DeclareLaunchArgument('use_rviz', default_value='false',
                              choices=['false', 'true']),
        DeclareLaunchArgument('record', default_value='off',
                              choices=['off', 'on']),
        DeclareLaunchArgument('voxel', default_value='off',
                              choices=['off', 'on']),
        DeclareLaunchArgument('driver', default_value='on',
                              choices=['on', 'off']),
        DeclareLaunchArgument('lio', default_value='dlio',
                              choices=['dlio', 'off']),
        DeclareLaunchArgument('diagnostics', default_value='on',
                              choices=['on', 'off']),
        DeclareLaunchArgument(
            'lidar_config',
            default_value=os.path.join(_CONFIG, 'MID360_config.json'),
            description='Mid-360 network JSON. The installed default carries '
                        'placeholder IPs until this robot is recorded; '
                        'g1_hw_preflight.sh refuses to pass while it does.'),
        DeclareLaunchArgument(
            'dlio_config', default_value=os.path.join(_CONFIG, 'dlio.yaml'),
            description='DLIO parameters. Extrinsics are DERIVED from '
                        'g1_mid360.xacro — edit the xacro, never this.'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(_SHARE, 'rviz', 'perception.rviz')),
        DeclareLaunchArgument('bag_path', default_value='hw_perception_bag'),

        # --- source: driver + DLIO ---------------------------------------
        _include('source_hw.launch.py',
                 driver=LaunchConfiguration('driver'),
                 lio=LaunchConfiguration('lio'),
                 driver_config=LaunchConfiguration('lidar_config'),
                 dlio_config=LaunchConfiguration('dlio_config')),

        # --- robot TF (static chain + odom->base_footprint) --------------
        _include('description.launch.py', use_sim_time='false'),

        # --- the shared perception stack, unchanged from sim (D4) --------
        _include('perception.launch.py', use_sim_time='false',
                 voxel=LaunchConfiguration('voxel')),

        # --- observability -----------------------------------------------
        Node(
            package='g1_perception_bringup',
            executable='hw_diagnostics.py',
            name='hw_diagnostics',
            output='screen',
            condition=LaunchConfigurationEquals('diagnostics', 'on'),
            parameters=[{'use_sim_time': False}],
        ),
        # viz.launch.py's dpcbf_overlay is GT-consuming and there is no GT on
        # hardware, so it is explicitly OFF and the GT relay is pointed at a
        # topic that does not exist here rather than being left on /sim/*.
        _include('viz.launch.py',
                 condition=IfCondition(LaunchConfiguration('use_rviz')),
                 use_sim_time='false',
                 overlay='off',
                 obstacles_topic='/hw/no_ground_truth',
                 rviz_config=LaunchConfiguration('rviz_config')),

        _include('record_hw.launch.py',
                 condition=LaunchConfigurationEquals('record', 'on'),
                 bag_path=LaunchConfiguration('bag_path')),
    ])
