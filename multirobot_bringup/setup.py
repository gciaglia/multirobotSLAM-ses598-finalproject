import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'multirobot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gianna',
    maintainer_email='giannac10@gmail.com',
    description='Multi-robot SLAM bringup: simulation, spawning, and launch files',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'map_merger = multirobot_bringup.map_merger:main',
            'explore_controller = multirobot_bringup.explore_controller:main',
            'frontier_detector = multirobot_bringup.frontier_detector:main',
            'explorer = multirobot_bringup.explorer:main',
            'metrics_logger = multirobot_bringup.metrics_logger:main',
        ],
    },
)
