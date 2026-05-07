"""Launch the metrics logger.

Reads robot_namespaces (same source-of-truth as multirobot.launch.py).
By default writes ~/multirobot_metrics_<timestamp>.csv. Override with
output_csv:=/path/to/file.csv.
"""

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    namespaces = yaml.safe_load(
        LaunchConfiguration('robot_namespaces').perform(context))
    poses = [float(v) for v in yaml.safe_load(
        LaunchConfiguration('robot_initial_poses').perform(context))]
    output = LaunchConfiguration('output_csv').perform(context)

    return [Node(
        package='multirobot_bringup',
        executable='metrics_logger',
        name='metrics_logger',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_namespaces': namespaces,
            'robot_initial_poses': poses,
            'output_csv': output,
            'period_sec': 1.0,
        }],
    )]


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'robot_namespaces', default_value='[robot1, robot2]'))
    ld.add_action(DeclareLaunchArgument(
        'robot_initial_poses',
        default_value='[0.0, 0.5, 0.0, 1.5]'))
    ld.add_action(DeclareLaunchArgument(
        'output_csv', default_value='',
        description='Path to CSV output. Empty = ~/multirobot_metrics_<ts>.csv'))
    ld.add_action(OpaqueFunction(function=_setup))
    return ld
