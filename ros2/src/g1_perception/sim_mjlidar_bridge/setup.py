from setuptools import setup

package_name = 'sim_mjlidar_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Seohyeon Lim',
    maintainer_email='joan0219@yonsei.ac.kr',
    description='Mirror-state MuJoCo-LiDAR sidecar (§11.2)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_mjlidar_bridge = sim_mjlidar_bridge.bridge_node:main',
        ],
    },
)
