"""Top-level bringup (§14.1).

ros2 launch g1_perception_bringup bringup.launch.py \
    source:=sim|hw mode:=oracle|shadow|estimated ground_seg:=off|patchwork \
    viz:=off|rviz record:=off|on

Phase 5A: source_hw is functional (livox_ros_driver2 + DLIO) and
`use_sim_time` now defaults FALSE when source:=hw — the one place the sim/hw
difference is expressed, keeping perception.launch.py conditional-free (D4).
`mode` is plumbed through for the Phase 4 adapter.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def _reject_patchwork(context, *_args, **_kwargs):
    """`ground_seg:=patchwork` was a NO-OP: nothing in this workspace ever
    read the argument. Patchwork++ is not in deps.repos, is not built, is not
    launched, and `/points_no_ground` does not exist — the only thing that
    rejects floor returns is `min_height` in pointcloud_to_laserscan.

    Silently accepting the argument is worse than not having it, because it
    lets a session believe ground segmentation was on. Phase 5C makes it an
    explicit error rather than deleting it, so an old command line gets an
    explanation instead of a shrug.
    """
    if context.launch_configurations.get('ground_seg') == 'patchwork':
        raise RuntimeError(
            'ground_seg:=patchwork is NOT IMPLEMENTED. Patchwork++ is not '
            'imported (deps.repos), not built, and not launched; there is no '
            '/points_no_ground topic and no ground-segmentation stage in '
            'perception.launch.py. Floor returns are rejected ONLY by '
            "pointcloud_to_laserscan's min_height (0.15 m in "
            'base_footprint), which is a height band, not segmentation, and '
            'is valid on flat floors only. Re-run with ground_seg:=off.')
    return []


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
        # Kept only so an old command line gets an explanation. `patchwork`
        # raises below; it never did anything (see _reject_patchwork).
        DeclareLaunchArgument('ground_seg', default_value='off',
                              choices=['off', 'patchwork'],
                              description='off only — patchwork is NOT '
                                          'implemented and is now rejected'),
        OpaqueFunction(function=_reject_patchwork),
        DeclareLaunchArgument('viz', default_value='off',
                              choices=['off', 'rviz']),
        DeclareLaunchArgument('record', default_value='off',
                              choices=['off', 'on']),
        # Sim time follows the source unless explicitly overridden. This is
        # THE place the sim/hw difference is allowed to live (D4): sim runs on
        # simulate's /clock, hardware has none, and a node left at
        # use_sim_time:=true on hardware simply never advances its clock.
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=PythonExpression(
                ["'true' if '", source, "' == 'sim' else 'false'"])),

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
