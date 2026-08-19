import os
from glob import glob

from setuptools import setup

package_name = 'dpcbf_plot_client'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Seohyeon Lim',
    maintainer_email='joan0219@yonsei.ac.kr',
    description='Read-only live plotting client for the DPCBF stack '
                '(Computer 3).',
    license='Apache-2.0',
    # extras_require['test'] (not tests_require, removed in setuptools>=72)
    # is what makes `colcon test` pick the pytest step for this package.
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'dpcbf_plot_client = dpcbf_plot_client.plot_client_main:main',
            'synthetic_dpcbf_publisher = '
            'dpcbf_plot_client.synthetic_publisher:main',
            # Robot-frame /scan + fitted circles. Separate entry point rather
            # than a backend of the one above: different question (is the FIT
            # noisy?), different frame, and it must stay runnable when the
            # control seam is absent.
            'dpcbf_scan_view = dpcbf_plot_client.scan_view:main',
            'navigation_goal_view = '
            'dpcbf_plot_client.navigation_goal_view:main',
        ],
    },
)
