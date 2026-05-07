"""Launch the multi-robot map merger.

Defaults match the spawn poses in README_NOTES.md (robot1 at origin,
robot2 at x=2.0 y=0.5). Override via launch args, e.g.:

    ros2 launch multirobot_bringup map_merger.launch.py \\
        robot_namespaces:='[robot1, robot2, robot3]' \\
        robot_initial_poses:='[0.0, 0.0, 2.0, 0.5, -2.0, 0.5]'
"""

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    namespaces = yaml.safe_load(
        LaunchConfiguration('robot_namespaces').perform(context)
    )
    poses = [float(v) for v in yaml.safe_load(
        LaunchConfiguration('robot_initial_poses').perform(context)
    )]
    merged_frame = LaunchConfiguration('merged_frame').perform(context)
    publish_rate = float(LaunchConfiguration('publish_rate_hz').perform(context))
    use_sim_time = (LaunchConfiguration('use_sim_time')
                    .perform(context).lower() == 'true')

    return [Node(
        package='multirobot_bringup',
        executable='map_merger',
        name='map_merger',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_namespaces': namespaces,
            'robot_initial_poses': poses,
            'merged_frame': merged_frame,
            'publish_rate_hz': publish_rate,
        }],
    )]


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'robot_namespaces', default_value='[robot1, robot2]',
        description='YAML list of robot namespaces',
    ))
    ld.add_action(DeclareLaunchArgument(
        'robot_initial_poses',
        default_value='[0.0, 0.5, 0.0, 1.5]',
        description='Flat YAML list of [x1,y1, x2,y2, ...] spawn poses',
    ))
    ld.add_action(DeclareLaunchArgument(
        'merged_frame', default_value='map_merged',
        description='Frame id for the published /map_merged grid',
    ))
    ld.add_action(DeclareLaunchArgument(
        'publish_rate_hz', default_value='1.0',
        description='Rate at which the merged map is republished',
    ))
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo /clock (set false for standalone testing)',
    ))
    ld.add_action(OpaqueFunction(function=_launch_setup))
    return ld
