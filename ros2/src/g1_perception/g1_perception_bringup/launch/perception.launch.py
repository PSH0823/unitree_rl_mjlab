"""THE shared perception stack (§14.1) — identical for sim and hardware, no
source branches inside (D4).

Phase 2: perception_container holds CropBox (self-filter, §9.3) +
pointcloud_to_laserscan (projection, §9.4) with intra-process comms.
Phase 3 adds obstacle_extractor + obstacle_tracker (§9.5, fork components via
patch P-1). Phase 4 adds the safety_obstacle_filter (§9.6) — the container
now holds the full chain to /obstacles_safe.

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
    # §9.5 detection & tracking (fork components, patch P-1). Extractor output
    # and tracker output are odom-frame per §7.1; params are Appendix A.
    extractor = ComposableNode(
        package='obstacle_detector',
        plugin='obstacle_detector::ObstacleExtractorComponent',
        name='obstacle_extractor',
        # Keep the critical frame setting inline.  On Foxy, parameter-file
        # overrides for loaded components can be dropped silently; without
        # this explicit override obstacle_detector falls back to ``map``.
        parameters=[os.path.join(_CONFIG, 'obstacle_detector.yaml'),
                    {'use_sim_time': use_sim_time,
                     'frame_id': 'odom',
                     'transform_coordinates': True}],
        remappings=[('scan', '/scan'),
                    ('raw_obstacles', '/raw_obstacles')],
        extra_arguments=_INTRA,
    )
    tracker = ComposableNode(
        package='obstacle_detector',
        plugin='obstacle_detector::ObstacleTrackerComponent',
        name='obstacle_tracker',
        parameters=[os.path.join(_CONFIG, 'obstacle_detector.yaml'),
                    {'use_sim_time': use_sim_time,
                     'frame_id': 'odom'}],
        remappings=[('raw_obstacles', '/raw_obstacles'),
                    ('tracked_obstacles', '/tracked_obstacles')],
        extra_arguments=_INTRA,
    )
    safety_filter = ComposableNode(
        package='safety_obstacle_filter',
        plugin='safety_obstacle_filter::SafetyObstacleFilterNode',
        name='safety_obstacle_filter',
        parameters=[os.path.join(_CONFIG, 'safety_obstacle_filter.yaml'),
                    {'use_sim_time': use_sim_time}],
        remappings=[('tracked_obstacles', '/tracked_obstacles'),
                    ('obstacles_safe', '/obstacles_safe')],
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
            composable_node_descriptions=[crop_box, projection,
                                          extractor, tracker,
                                          safety_filter],
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
                                          projection_from_voxel,
                                          extractor, tracker,
                                          safety_filter],
            condition=LaunchConfigurationEquals('voxel', 'on'),
        ),
    ])
