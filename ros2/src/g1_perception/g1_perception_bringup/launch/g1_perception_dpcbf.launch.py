"""Computer 2 entry point for a DPCBF session WITH off-board plotting.

    ros2 launch g1_perception_bringup g1_perception_dpcbf.launch.py \
        enable_plot_bridge:=true plot_publish_rate:=30.0

What this starts: the perception-only hardware stack, unchanged, by
INCLUDING g1_perception_hardware_only.launch.py (all of its isolation
guarantees — no actuation path, no DPCBF seam, no command topic — hold
verbatim; hw_offline_gates keeps asserting them against that file).

What this does NOT start: the DPCBF control binary. The control seam lives
in the (ROS2-enabled) `unitree_mujoco` process today and in the §9 hardware
seam later; both are started from a shell, not from ros2 launch. The
`enable_plot_bridge` / `plot_publish_rate` arguments are therefore exported
as the environment variables the control binary reads at startup

    UNITREE_DPCBF_PLOT=0|1        (overrides plot_bridge.enabled in
                                   config/dpcbf_ros_adapter.yaml)
    UNITREE_DPCBF_PLOT_RATE=<hz>  (overrides plot_bridge.rate_hz)

so they take effect for any control process started FROM THIS LAUNCH in the
future; a control binary started in another shell needs the same variables
exported there (or just the yaml). The yaml is the durable control surface;
the env vars are the per-run override.

The plotting client (Computer 3) needs nothing from this file: it
subscribes /dpcbf/plot, /odom and /obstacles_safe over CycloneDDS.

synthetic:=on replaces the hardware stack question entirely: it starts the
dpcbf_plot_client synthetic source instead (stack:=off implied unless
overridden), which is the no-robot bench verification path.

Arguments
  enable_plot_bridge:=true|false   exported as UNITREE_DPCBF_PLOT
  plot_publish_rate:=<hz>          exported as UNITREE_DPCBF_PLOT_RATE
  stack:=hw|off                    hw = include the perception-only stack
  synthetic:=off|on                start the synthetic /dpcbf/plot + /odom
                                   source (bench only — NEVER on the robot
                                   while the real pipeline runs: it would
                                   publish a second /odom)
  + every g1_perception_hardware_only.launch.py argument, passed through
    (use_rviz, record, voxel, driver, lio, diagnostics, lidar_config,
    dlio_config, rviz_config, bag_path)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_SHARE = get_package_share_directory('g1_perception_bringup')
_CONFIG = os.path.join(_SHARE, 'config')
_LAUNCH = os.path.join(_SHARE, 'launch')


def _reject_synthetic_with_hw(context, *_args, **_kwargs):
    """synthetic:=on next to the real stack would publish a second /odom and
    a fake /obstacles_safe INTO the live graph. Refuse loudly instead of
    letting a bench flag corrupt a robot session."""
    if (context.launch_configurations.get('synthetic') == 'on'
            and context.launch_configurations.get('stack') == 'hw'):
        raise RuntimeError(
            'synthetic:=on requires stack:=off — the synthetic source '
            'publishes /odom and /obstacles_safe and must never run beside '
            'the real perception pipeline.')
    return []


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_plot_bridge', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('plot_publish_rate', default_value='30.0'),
        DeclareLaunchArgument('stack', default_value='hw',
                              choices=['hw', 'off']),
        DeclareLaunchArgument('synthetic', default_value='off',
                              choices=['off', 'on']),
        OpaqueFunction(function=_reject_synthetic_with_hw),

        # Pass-throughs for the included hardware stack.
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
            default_value=os.path.join(_CONFIG, 'MID360_config.json')),
        DeclareLaunchArgument(
            'dlio_config', default_value=os.path.join(_CONFIG, 'dlio.yaml')),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(_SHARE, 'rviz', 'perception.rviz')),
        DeclareLaunchArgument('bag_path', default_value='hw_perception_bag'),

        # Control-binary override surface (see module docstring).
        SetEnvironmentVariable(
            'UNITREE_DPCBF_PLOT',
            PythonExpression(["'1' if '",
                              LaunchConfiguration('enable_plot_bridge'),
                              "' == 'true' else '0'"])),
        SetEnvironmentVariable('UNITREE_DPCBF_PLOT_RATE',
                               LaunchConfiguration('plot_publish_rate')),
        LogInfo(msg=['plot bridge: enable=',
                     LaunchConfiguration('enable_plot_bridge'),
                     ' rate=', LaunchConfiguration('plot_publish_rate'),
                     ' Hz (env override for a control binary started from '
                     'this launch; the yaml plot_bridge section governs '
                     'otherwise)']),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(_LAUNCH,
                             'g1_perception_hardware_only.launch.py')),
            condition=LaunchConfigurationEquals('stack', 'hw'),
            launch_arguments=[
                ('use_rviz', LaunchConfiguration('use_rviz')),
                ('record', LaunchConfiguration('record')),
                ('voxel', LaunchConfiguration('voxel')),
                ('driver', LaunchConfiguration('driver')),
                ('lio', LaunchConfiguration('lio')),
                ('diagnostics', LaunchConfiguration('diagnostics')),
                ('lidar_config', LaunchConfiguration('lidar_config')),
                ('dlio_config', LaunchConfiguration('dlio_config')),
                ('rviz_config', LaunchConfiguration('rviz_config')),
                ('bag_path', LaunchConfiguration('bag_path')),
            ]),

        Node(
            package='dpcbf_plot_client',
            executable='synthetic_dpcbf_publisher',
            name='synthetic_dpcbf_publisher',
            output='screen',
            condition=LaunchConfigurationEquals('synthetic', 'on'),
            parameters=[{
                'rate_hz': ParameterValue(
                    LaunchConfiguration('plot_publish_rate'),
                    value_type=float),
            }],
        ),
    ])
