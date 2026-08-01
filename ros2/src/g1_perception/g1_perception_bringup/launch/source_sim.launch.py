"""Simulated sensor source (§11): the sim_mjlidar_bridge sidecar.

Expects `simulate` to be running with the ROS2 module ON (it publishes
/clock, /sim/mj_state, /sim/gt_obstacles and dumps the mirror model XML).
PYTHONPATH for the pinned MuJoCo-LiDAR checkout is injected here so nothing
needs pip-installing; the executable itself runs under system python (R-11).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _mujoco_lidar_src():
    # merged install: <ws>/install/share/<pkg> -> workspace root is 3 up
    share = get_package_share_directory('g1_perception_bringup')
    ws = os.path.abspath(os.path.join(share, '..', '..', '..'))
    return os.path.join(ws, 'src', 'external', 'MuJoCo-LiDAR', 'src')


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('g1_perception_bringup'),
                       'config', 'sim_mjlidar_bridge.yaml')
    mjlidar = _mujoco_lidar_src()
    if not os.path.isdir(mjlidar):
        raise RuntimeError(f'MuJoCo-LiDAR checkout not found: {mjlidar} '
                           '(run ros2/setup_external.sh)')
    pythonpath = mjlidar + os.pathsep + os.environ.get('PYTHONPATH', '')

    return LaunchDescription([
        DeclareLaunchArgument('mirror_model_path',
                              default_value='/tmp/unitree_mujoco_mirror_model.xml'),
        Node(
            package='sim_mjlidar_bridge',
            executable='sim_mjlidar_bridge',
            name='sim_mjlidar_bridge',
            output='screen',
            additional_env={'PYTHONPATH': pythonpath},
            parameters=[
                cfg,
                {'use_sim_time': True,
                 'mirror_model_path': LaunchConfiguration('mirror_model_path')},
            ],
        ),
    ])
