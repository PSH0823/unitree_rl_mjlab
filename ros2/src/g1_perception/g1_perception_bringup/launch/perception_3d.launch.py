"""The 3D perception stack (pipeline:=3d) — HUMBLE ONLY.

perception_3d_container: CropBox (self-filter, same config as the 2D path) →
cloud_object_detector (clustering + convex footprint, DetectedObjects) →
autoware_multi_object_tracker (muSSP association + per-class EKF,
TrackedObjects with velocity). All components, intra-process, no sim/hw
branches (D4) — the sensor source and TF come from source_*/description
exactly like the 2D path.

The 2D chain (/scan → obstacle_detector → safety filter) is untouched and
selected by pipeline:=2d; nothing downstream of /obstacles_safe consumes the
3D output yet (Phase 5 bridge, not built).

Topics: /points_self_filtered → /perception/detected_objects (+
/perception/detected_convex, the 3D hulls) → /perception/tracked_objects
(autoware TrackedObjects, 2.5D) → /perception/tracked_convex
(perception_3d_msgs/ConvexObjects: one convex polytope per track id, world
frame velocity) — the geometry output. All frame odom.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

_CONFIG = os.path.join(
    get_package_share_directory('g1_perception_bringup'), 'config')
_INTRA = [{'use_intra_process_comms': True}]


def _params(fname, key):
    """Same contract as perception.launch.py's _params (see its docstring:
    components must get dicts, not parameter files; missing file/key raises
    instead of silently degrading to plugin defaults)."""
    path = os.path.join(_CONFIG, fname)
    with open(path) as f:
        doc = yaml.safe_load(f) or {}
    params = {}
    for k in ('/**', key):
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
    detector = ComposableNode(
        package='cloud_object_detector',
        plugin='cloud_object_detector::CloudObjectDetectorNode',
        name='cloud_object_detector',
        parameters=[_params('cloud_object_detector.yaml',
                            'cloud_object_detector'),
                    {'use_sim_time': use_sim_time}],
        remappings=[('points', '/points_self_filtered'),
                    ('detected_objects', '/perception/detected_objects'),
                    ('detected_convex', '/perception/detected_convex')],
        extra_arguments=_INTRA,
    )
    # Three parameter sources: node settings (ours), association matrix and
    # input-channel catalog (verbatim upstream copies) — see
    # multi_object_tracker.yaml's header for what was changed and why.
    tracker = ComposableNode(
        package='autoware_multi_object_tracker',
        plugin='autoware::multi_object_tracker::MultiObjectTracker',
        name='multi_object_tracker',
        parameters=[_params('multi_object_tracker.yaml',
                            'multi_object_tracker'),
                    _params('multi_object_tracker_association.yaml',
                            'multi_object_tracker'),
                    _params('multi_object_tracker_input_channels.yaml',
                            'multi_object_tracker'),
                    {'use_sim_time': use_sim_time}],
        remappings=[('~/input/detection01/objects',
                     '/perception/detected_objects'),
                    ('~/output/objects', '/perception/tracked_objects')],
        extra_arguments=_INTRA,
    )
    # RViz relay (obstacles_marker_relay pattern): prism wireframe + velocity
    # arrow + short id. Cheap enough to keep always-on like the 2D tracker's
    # visualization output.
    viz = ComposableNode(
        package='tracked_objects_viz',
        plugin='tracked_objects_viz::TrackedObjectsVizNode',
        name='tracked_objects_viz',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('tracked_objects', '/perception/tracked_objects'),
                    ('tracked_objects_markers',
                     '/perception/tracked_objects_markers')],
        extra_arguments=_INTRA,
    )

    # Binds each track (id, velocity) to the detector's 3D convex hull of
    # the same measurement — the polytope output whose cross-section varies
    # with z, unlike autoware's single-footprint Shape.
    attach = ComposableNode(
        package='cloud_object_detector',
        plugin='cloud_object_detector::TrackedConvexAttachNode',
        name='tracked_convex_attach',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('tracked_objects', '/perception/tracked_objects'),
                    ('detected_convex', '/perception/detected_convex'),
                    ('tracked_convex', '/perception/tracked_convex')],
        extra_arguments=_INTRA,
    )
    convex_viz = ComposableNode(
        package='tracked_objects_viz',
        plugin='tracked_objects_viz::ConvexObjectsVizNode',
        name='convex_objects_viz',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('convex_objects', '/perception/tracked_convex'),
                    ('convex_markers', '/perception/tracked_convex_markers')],
        extra_arguments=_INTRA,
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        ComposableNodeContainer(
            name='perception_3d_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            composable_node_descriptions=[crop_box, detector, tracker,
                                          attach, viz, convex_viz],
        ),
    ])
