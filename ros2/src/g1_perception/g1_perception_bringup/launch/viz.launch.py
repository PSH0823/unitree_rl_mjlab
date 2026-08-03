"""RViz2 + obstacle marker relays (§14.4).

Four relays overlay GT (green, ground truth), raw (grey, extractor output),
tracked (orange, tracker output) and safe (red, gated+inflated — what DPCBF
would eat) obstacles — the primary visual debugging surface from Phase 3 on.

A fifth node, `dpcbf_overlay` (§4.6), adds the estimated-vs-GT overlay and the
DPCBF constraint geometry. It is GT-consuming, so — per D4 — it lives here
behind `overlay:=on|off` rather than anywhere near perception.launch.py, and
it degrades visibly (banner marker) when /sim/gt_obstacles is absent.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    rviz_cfg = os.path.join(get_package_share_directory('g1_perception_bringup'),
                            'rviz', 'perception.rviz')
    # The overlay must read the SAME config the simulator loaded, not a copy:
    # a drifting duplicate is how a viewer starts confidently lying. Default
    # is the repo file; point it at $SHADOW/dpcbf/config/dpcbf_config.yaml
    # whenever the live run uses a shadow tree (runbook §3.2).
    default_dpcbf_cfg = os.path.normpath(os.path.join(
        get_package_share_directory('g1_perception_bringup'),
        '..', '..', '..', '..', 'dpcbf', 'config', 'dpcbf_config.yaml'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('obstacles_topic', default_value='/sim/gt_obstacles'),
        DeclareLaunchArgument('overlay', default_value='on',
                              description='dpcbf_overlay marker layer: on|off'),
        DeclareLaunchArgument('dpcbf_config', default_value=default_dpcbf_cfg,
                              description='dpcbf_config.yaml the sim loaded'),
        DeclareLaunchArgument('overlay_log', default_value='',
                              description='JSONL path for the §4.6 gate; empty = off'),
        # The committed layout is tuned for the top-down 2-D default, which
        # means LivoxCloud and RawObstacles start off (they cover everything
        # from above). An interactive session that wants the raw cloud can
        # point this at a modified copy instead of editing the tracked file
        # and rebuilding — see run_live_w4_view.sh.
        DeclareLaunchArgument('rviz_config', default_value=rviz_cfg,
                              description='RViz layout to open (default: the '
                                          'committed perception.rviz)'),
        Node(
            package='g1_perception_utils',
            executable='obstacles_marker_relay',
            name='gt_obstacles_marker_relay',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'topic': LaunchConfiguration('obstacles_topic'),
                         'color_r': 0.2, 'color_g': 0.8, 'color_b': 0.2,
                         'alpha': 0.35, 'show_ids': True}],
        ),
        Node(
            package='g1_perception_utils',
            executable='obstacles_marker_relay',
            name='raw_obstacles_marker_relay',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'topic': '/raw_obstacles',
                         'cylinder_height': 0.8,
                         'color_r': 0.6, 'color_g': 0.6, 'color_b': 0.6,
                         'alpha': 0.5, 'show_ids': False}],
        ),
        Node(
            package='g1_perception_utils',
            executable='obstacles_marker_relay',
            name='tracked_obstacles_marker_relay',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'topic': '/tracked_obstacles',
                         'cylinder_height': 1.2,
                         'color_r': 1.0, 'color_g': 0.55, 'color_b': 0.1,
                         'alpha': 0.6, 'show_ids': True}],
        ),
        Node(
            package='g1_perception_utils',
            executable='obstacles_marker_relay',
            name='safe_obstacles_marker_relay',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'topic': '/obstacles_safe',
                         'cylinder_height': 1.5,
                         'color_r': 0.9, 'color_g': 0.15, 'color_b': 0.15,
                         'alpha': 0.35, 'show_ids': False}],
        ),
        Node(
            package='g1_perception_utils',
            executable='dpcbf_overlay',
            name='dpcbf_overlay',
            output='screen',
            # on|off, not sim|hw — the GT dependency is handled at runtime by
            # the banner, so this argument only turns the layer off.
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration('overlay'), "' == 'on'"])),
            parameters=[{'use_sim_time': use_sim_time,
                         'dpcbf_config': LaunchConfiguration('dpcbf_config'),
                         'log_path': LaunchConfiguration('overlay_log')}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
