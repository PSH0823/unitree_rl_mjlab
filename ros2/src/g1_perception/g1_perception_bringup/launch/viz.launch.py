"""RViz2 + GT-obstacle marker relay (§14.4)."""
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
                         'topic': LaunchConfiguration('obstacles_topic')}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
