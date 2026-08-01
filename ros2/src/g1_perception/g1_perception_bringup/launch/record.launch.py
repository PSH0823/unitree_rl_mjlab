"""rosbag2 recording (§14.1). MCAP storage once ros-humble-rosbag2-storage-mcap
is installable (needs sudo/apt — recorded follow-up); sqlite3 until then."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

TOPICS = [
    '/livox/lidar', '/odom', '/tf', '/tf_static', '/scan',
    '/raw_obstacles', '/tracked_obstacles', '/obstacles_safe',
    '/sim/gt_obstacles', '/dpcbf/status', '/clock',
]


def generate_launch_description():
    out = LaunchConfiguration('bag_path')
    return LaunchDescription([
        DeclareLaunchArgument('bag_path', default_value='perception_bag'),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', out,
                 '--include-unpublished-topics', *TOPICS],
            output='screen',
        ),
    ])
