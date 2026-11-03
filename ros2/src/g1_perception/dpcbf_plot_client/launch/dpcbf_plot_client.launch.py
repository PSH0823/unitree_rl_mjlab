"""Computer 3 entry point: the plotting client, nothing else.

    ros2 launch dpcbf_plot_client dpcbf_plot_client.launch.py
    ros2 launch dpcbf_plot_client dpcbf_plot_client.launch.py \
        backend:=matplotlib window_s:=60.0 synthetic:=on

synthetic:=on additionally starts the in-package synthetic source — the
one-command bench demo on a single machine. On the real Computer 3 leave it
off; the data comes from Computer 2 over CycloneDDS.

This launch constructs no publisher of anything the robot stack consumes:
the client subscribes BestEffort and renders. use_sim_time is deliberately
not an argument (Computer 3 has no /clock; the client keys everything on its
local monotonic clock).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='auto',
                              choices=['auto', 'pyqtgraph', 'matplotlib']),
        DeclareLaunchArgument('window_s', default_value='30.0',
                              description='time-series window [s]'),
        DeclareLaunchArgument('gui_rate_hz', default_value='25.0'),
        DeclareLaunchArgument('stale_after_s', default_value='1.0',
                              description='age beyond which a source is '
                                          'flagged STALE'),
        DeclareLaunchArgument('plot_topic', default_value='/dpcbf/plot'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('obstacles_topic',
                              default_value='/obstacles_safe'),
        DeclareLaunchArgument('synthetic', default_value='off',
                              choices=['off', 'on'],
                              description='also start the synthetic source '
                                          '(single-machine bench demo)'),
        Node(
            package='dpcbf_plot_client',
            executable='synthetic_dpcbf_publisher',
            name='synthetic_dpcbf_publisher',
            output='screen',
            condition=LaunchConfigurationEquals('synthetic', 'on'),
        ),
        Node(
            package='dpcbf_plot_client',
            executable='dpcbf_plot_client',
            name='dpcbf_plot_client',
            output='screen',
            parameters=[{
                'backend': LaunchConfiguration('backend'),
                'window_s': ParameterValue(
                    LaunchConfiguration('window_s'), value_type=float),
                'gui_rate_hz': ParameterValue(
                    LaunchConfiguration('gui_rate_hz'), value_type=float),
                'stale_after_s': ParameterValue(
                    LaunchConfiguration('stale_after_s'), value_type=float),
                'plot_topic': LaunchConfiguration('plot_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'obstacles_topic': LaunchConfiguration('obstacles_topic'),
            }],
        ),
    ])
