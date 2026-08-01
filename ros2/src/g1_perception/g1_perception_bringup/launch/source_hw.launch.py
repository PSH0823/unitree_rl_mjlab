"""Hardware sensor source (§12): livox_ros_driver2 + DLIO. Phase 5 scope —
this placeholder fails loudly so nobody believes hw bringup exists yet."""
from launch import LaunchDescription
from launch.actions import LogInfo, Shutdown


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg='source_hw is Phase 5 scope: livox_ros_driver2 + DLIO are '
                    'pinned in deps.repos but not yet imported/configured.'),
        Shutdown(reason='source_hw not implemented (Phase 5)'),
    ])
