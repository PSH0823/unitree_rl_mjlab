"""Hardware rosbag2 recording WITH provenance (Phase 5C, §5.7 of the brief).

`record.launch.py` is the sim-era recorder: one topic list covering both
worlds, no metadata. This is the hardware one. Two differences, both of them
the point:

1. The topic list is the hardware chain only — no /clock, no /sim/*. The full
   §5.7 list, so a bag can be re-run through the whole offline pipeline.
2. It writes `<bag_path>.session.json` NEXT TO THE BAG at capture time, with
   the commit, the architecture, the DDS environment and the checksums of the
   YAMLs that were actually loaded. The operator-only fields (G1 variant,
   LiDAR serial, robot state, prop survey) are left blank and REPORTED, so an
   under-documented capture is visible while the robot is still in front of
   you.

`hw_session_metadata.py` exits 1 when operator fields are blank. That is
information, not a launch failure — this action does not gate the recording
on it, because losing a capture over a missing serial number would be worse
than an incomplete record. The operator fills it in afterwards with the same
script and `--from-json`.

Arguments
  bag_path:=<path>        output bag (default hw_perception_bag)
  storage:=sqlite3|mcap
  metadata:=on|off
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression

# §5.7. /diagnostics is included so a bag carries the stack's own opinion of
# itself: an offline reader can see that the LiDAR was at 6 Hz without having
# to re-derive it.
TOPICS = [
    '/livox/lidar',
    '/livox/imu',
    '/odom',
    '/tf',
    '/tf_static',
    '/points_self_filtered',
    '/scan',
    '/raw_obstacles',
    '/tracked_obstacles',
    '/obstacles_safe',
    '/diagnostics',
]


def generate_launch_description():
    bag = LaunchConfiguration('bag_path')
    return LaunchDescription([
        DeclareLaunchArgument('bag_path', default_value='hw_perception_bag'),
        DeclareLaunchArgument('storage', default_value='sqlite3',
                              choices=['sqlite3', 'mcap']),
        DeclareLaunchArgument('metadata', default_value='on',
                              choices=['on', 'off']),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', bag,
                 '-s', LaunchConfiguration('storage'),
                 # A topic that has not been created yet (the driver's
                 # publishers are lazy) is otherwise silently absent from the
                 # bag for the whole run.
                 '--include-unpublished-topics', *TOPICS],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'g1_perception_bringup',
                 'hw_session_metadata.py',
                 '--out', PythonExpression(["'", bag, "' + '.session.json'"]),
                 '--bag', bag, '--copy-configs'],
            output='screen',
            condition=LaunchConfigurationEquals('metadata', 'on'),
        ),
    ])
