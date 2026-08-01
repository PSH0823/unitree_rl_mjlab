"""Robot TF (§8): robot_state_publisher on g1_mid360.xacro (static
base_link->torso_link->mid360_link) + base_footprint_publisher
(odom->base_footprint)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory('g1_description'),
                              'urdf', 'g1_mid360.xacro')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]),
                                       value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': use_sim_time}],
        ),
        Node(
            package='g1_perception_utils',
            executable='base_footprint_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
