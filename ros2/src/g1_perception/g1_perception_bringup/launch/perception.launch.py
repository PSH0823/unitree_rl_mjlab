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
import yaml

_CONFIG = os.path.join(
    get_package_share_directory('g1_perception_bringup'), 'config')
_INTRA = [{'use_intra_process_comms': True}]


def _params(fname, key):
    """Read `fname`'s ros__parameters into a plain dict.

    Every node in this file is a COMPONENT, and a component must not be handed
    a parameter *file*. Foxy's launch_ros builds the LoadNode request with
    ``to_parameters_list(context, evaluated_parameters)`` — no node name — so a
    file whose top-level key names the node matches nothing and is dropped
    WITHOUT a warning; ``/**`` does not rescue it either (measured on the G1,
    2026-08-12). The component then starts on its plugin's declare_parameter
    defaults: for pcl_ros::CropBox that is a +/-1.0 m box with negative=false,
    i.e. the self-filter running INVERTED, keeping only the robot's own
    returns and discarding the world. Nothing in the console says so.

    Humble passes the node name through and matches the key, which is why sim
    never showed this and only the robot did. A dict is expanded by launch_ros
    itself and behaves identically on both distros, so the YAML stays the
    single source of truth and config_diff.py keeps meaning what it says.

    A missing file or missing key raises here instead of degrading to
    defaults. The silence is what made this expensive to find.
    """
    path = os.path.join(_CONFIG, fname)
    with open(path) as f:
        doc = yaml.safe_load(f) or {}
    params = {}
    for k in ('/**', key):          # wildcard first, node-specific key wins
        if k in doc:
            params.update(doc[k].get('ros__parameters', {}))
    if not params:
        raise RuntimeError(
            "{}: no 'ros__parameters' under '{}' or '/**'".format(path, key))
    return params


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    crop_box = ComposableNode(
        package='pcl_ros',
        plugin='pcl_ros::CropBox',
        name='crop_box_self_filter',
        parameters=[_params('cropbox_self_filter.yaml',
                            'crop_box_self_filter'),
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
        parameters=[_params('pointcloud_to_laserscan.yaml',
                            'pointcloud_to_laserscan'),
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
        # frame_id and transform_coordinates were repeated inline here to
        # survive the dropped parameter file (see _params). Now that the file
        # is actually loaded they would only be a second place to forget to
        # edit — the same failure mode one level up — so the YAML owns them.
        parameters=[_params('obstacle_detector.yaml', 'obstacle_extractor'),
                    {'use_sim_time': use_sim_time}],
        remappings=[('scan', '/scan'),
                    ('raw_obstacles', '/raw_obstacles')],
        extra_arguments=_INTRA,
    )
    tracker = ComposableNode(
        package='obstacle_detector',
        plugin='obstacle_detector::ObstacleTrackerComponent',
        name='obstacle_tracker',
        parameters=[_params('obstacle_detector.yaml', 'obstacle_tracker'),
                    {'use_sim_time': use_sim_time}],
        remappings=[('raw_obstacles', '/raw_obstacles'),
                    ('tracked_obstacles', '/tracked_obstacles')],
        extra_arguments=_INTRA,
    )
    safety_filter = ComposableNode(
        package='safety_obstacle_filter',
        plugin='safety_obstacle_filter::SafetyObstacleFilterNode',
        name='safety_obstacle_filter',
        parameters=[_params('safety_obstacle_filter.yaml',
                            'safety_obstacle_filter'),
                    {'use_sim_time': use_sim_time}],
        remappings=[('tracked_obstacles', '/tracked_obstacles'),
                    ('obstacles_safe', '/obstacles_safe')],
        extra_arguments=_INTRA,
    )
    projection_from_voxel = ComposableNode(
        package='pointcloud_to_laserscan',
        plugin='pointcloud_to_laserscan::PointCloudToLaserScanNode',
        name='pointcloud_to_laserscan',
        parameters=[_params('pointcloud_to_laserscan.yaml',
                            'pointcloud_to_laserscan'),
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
