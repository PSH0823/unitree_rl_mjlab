"""THE shared perception stack (§14.1) — identical for sim and hardware, no
source branches inside (D4).

Phase 2: perception_container holds CropBox (self-filter, §9.3) +
pointcloud_to_laserscan (projection, §9.4) with intra-process comms.
Phase 3 adds the obstacle_detector fork, Phase 4 the safety_obstacle_filter.

VoxelGrid (§9.3 option) stays OFF by default: pass voxel:=on to insert it
between CropBox and projection (leaf 0.05 m) if CPU ever requires it.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

_CONFIG = os.path.join(
    get_package_share_directory('g1_perception_bringup'), 'config')
_INTRA = [{'use_intra_process_comms': True}]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    crop_box = ComposableNode(
        package='pcl_ros',
        plugin='pcl_ros::CropBox',
        name='crop_box_self_filter',
        parameters=[os.path.join(_CONFIG, 'cropbox_self_filter.yaml'),
                    {'use_sim_time': use_sim_time}],
        remappings=[('input', '/livox/lidar'),
                    ('output', '/points_self_filtered')],
        extra_arguments=_INTRA,
    )
    # §9.3 option, OFF by default (voxel:=on). Output feeds projection.
    voxel_grid = ComposableNode(
        package='pcl_ros',
        plugin='pcl_ros::VoxelGrid',
        name='voxel_grid',
        parameters=[{'leaf_size': 0.05, 'use_sim_time': use_sim_time}],
        remappings=[('input', '/points_self_filtered'),
                    ('output', '/points_voxel')],
        extra_arguments=_INTRA,
    )
    projection = ComposableNode(
        package='pointcloud_to_laserscan',
        plugin='pointcloud_to_laserscan::PointCloudToLaserScanNode',
        name='pointcloud_to_laserscan',
        parameters=[os.path.join(_CONFIG, 'pointcloud_to_laserscan.yaml'),
                    {'use_sim_time': use_sim_time}],
        remappings=[('cloud_in', '/points_self_filtered'),
                    ('scan', '/scan')],
        extra_arguments=_INTRA,
    )
    projection_from_voxel = ComposableNode(
        package='pointcloud_to_laserscan',
        plugin='pointcloud_to_laserscan::PointCloudToLaserScanNode',
        name='pointcloud_to_laserscan',
        parameters=[os.path.join(_CONFIG, 'pointcloud_to_laserscan.yaml'),
                    {'use_sim_time': use_sim_time}],
        remappings=[('cloud_in', '/points_voxel'),
                    ('scan', '/scan')],
        extra_arguments=_INTRA,
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('voxel', default_value='off',
                              description='§9.3 VoxelGrid option (off|on)'),
        ComposableNodeContainer(
            name='perception_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            composable_node_descriptions=[crop_box, projection],
            condition=LaunchConfigurationEquals('voxel', 'off'),
        ),
        ComposableNodeContainer(
            name='perception_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            composable_node_descriptions=[crop_box, voxel_grid,
                                          projection_from_voxel],
            condition=LaunchConfigurationEquals('voxel', 'on'),
        ),
    ])
