"""RViz2 + obstacle marker relays (§14.4).

Four relays overlay GT (green, ground truth), raw (grey, extractor output),
tracked (orange, tracker output) and safe (red, gated+inflated — what DPCBF
would eat) obstacles — the primary visual debugging surface from Phase 3 on.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_cfg = os.path.join(get_package_share_directory('g1_perception_bringup'),
                            'rviz', 'perception.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('obstacles_topic', default_value='/sim/gt_obstacles'),
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
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
