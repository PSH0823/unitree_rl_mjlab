"""THE shared perception stack (§14.1) — identical for sim and hardware, no
source branches inside (D4).

Phase 0/1: the composable container is an empty shell; Phase 2 adds CropBox +
pointcloud_to_laserscan, Phase 3 the obstacle_detector fork, Phase 4 the
safety_obstacle_filter. Parameter YAMLs for those stages already live in
config/ (Appendix A verbatim).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        ComposableNodeContainer(
            name='perception_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            composable_node_descriptions=[],  # Phases 2-4 fill this
        ),
    ])
