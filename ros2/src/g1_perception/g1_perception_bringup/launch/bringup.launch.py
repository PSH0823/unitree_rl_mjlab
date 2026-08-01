"""Top-level bringup (§14.1).

ros2 launch g1_perception_bringup bringup.launch.py \
    source:=sim|hw mode:=oracle|shadow|estimated ground_seg:=off|patchwork \
    viz:=off|rviz record:=off|on

Phase 0/1 scope: source_sim, description, viz, record are functional; the
perception container is an empty shell until Phase 2; source_hw is a Phase 5
placeholder; `mode` is plumbed through for the Phase 4 adapter.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(name, condition=None, **launch_args):
    share = get_package_share_directory('g1_perception_bringup')
    kwargs = {'launch_arguments': list(launch_args.items())} if launch_args else {}
    if condition is not None:
        kwargs['condition'] = condition
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', name)),
        **kwargs)


def generate_launch_description():
    source = LaunchConfiguration('source')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('source', default_value='sim',
                              choices=['sim', 'hw']),
        DeclareLaunchArgument('mode', default_value='oracle',
                              choices=['oracle', 'shadow', 'estimated']),
        DeclareLaunchArgument('ground_seg', default_value='off',
                              choices=['off', 'patchwork']),
        DeclareLaunchArgument('viz', default_value='off',
                              choices=['off', 'rviz']),
        DeclareLaunchArgument('record', default_value='off',
                              choices=['off', 'on']),
        # sim time follows the source unless overridden
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        _include('source_sim.launch.py',
                 condition=LaunchConfigurationEquals('source', 'sim')),
        _include('source_hw.launch.py',
                 condition=LaunchConfigurationEquals('source', 'hw')),
        _include('description.launch.py', use_sim_time=use_sim_time),
        _include('perception.launch.py', use_sim_time=use_sim_time),
        _include('viz.launch.py',
                 condition=LaunchConfigurationEquals('viz', 'rviz'),
                 use_sim_time=use_sim_time),
        _include('record.launch.py',
                 condition=LaunchConfigurationEquals('record', 'on')),
    ])
