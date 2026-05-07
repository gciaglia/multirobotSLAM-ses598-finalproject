"""Launch the multi-robot exploration coordinator.

Reads the same robot_namespaces / robot_initial_poses launch args as
multirobot.launch.py so the spawn poses stay a single source of truth.
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
    strategy = LaunchConfiguration('selection_strategy').perform(context)
    use_sim_time = (LaunchConfiguration('use_sim_time')
                    .perform(context).lower() == 'true')

    return [Node(
        package='multirobot_bringup',
        executable='explorer',
        name='explorer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_namespaces': namespaces,
            'robot_initial_poses': poses,
            'selection_strategy': strategy,
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
        'selection_strategy', default_value='random',
        description='random | nearest (UCB1 plugs in later)'))
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='true'))
    ld.add_action(OpaqueFunction(function=_setup))
    return ld
