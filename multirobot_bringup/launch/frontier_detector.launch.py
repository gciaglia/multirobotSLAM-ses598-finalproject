"""Launch the frontier detector on /map_merged.

Override params via --ros-args, e.g.:
    ros2 launch multirobot_bringup frontier_detector.launch.py \
        --ros-args -p min_cluster_size:=10 -p rate_hz:=2.0
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='multirobot_bringup',
            executable='frontier_detector',
            name='frontier_detector',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'map_topic': '/map_merged',
                'frontiers_topic': '/frontiers',
                'viz_topic': '/frontiers_viz',
                'min_cluster_size': 5,
                'rate_hz': 1.0,
            }],
        ),
    ])
