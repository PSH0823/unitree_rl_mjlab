"""RViz2 for the 3D pipeline (pipeline:=3d).

Two marker layers over the raw cloud: GT (green cylinders, the same
gt_obstacles_marker_relay the 2D view uses — DynamicObstacleManager circles
are the ground truth either way) and Tracked3D (the tracked_objects_viz
component inside perception_3d_container publishes it; nothing to start
here). The 2D-pipeline relays (raw/tracked/safe) and dpcbf_overlay have no
3D-pipeline meaning and are deliberately absent.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_cfg = os.path.join(get_package_share_directory('g1_perception_bringup'),
                            'rviz', 'perception_3d.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('obstacles_topic', default_value='/sim/gt_obstacles'),
        DeclareLaunchArgument('rviz_config', default_value=rviz_cfg,
                              description='RViz layout to open (default: the '
                                          'committed perception_3d.rviz)'),
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
            package='rviz2',
            executable='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
