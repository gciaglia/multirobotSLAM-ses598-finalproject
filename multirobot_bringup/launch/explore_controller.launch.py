"""Launch one explore_controller for a given robot namespace.

Usage:
    ros2 launch multirobot_bringup explore_controller.launch.py namespace:=robot1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    import yaml
    from launch.actions import OpaqueFunction

    def _setup(context):
        ns = LaunchConfiguration('namespace').perform(context)
        ust = (LaunchConfiguration('use_sim_time')
               .perform(context).lower() == 'true')
        all_ns = yaml.safe_load(
            LaunchConfiguration('all_namespaces').perform(context))
        all_off = [float(v) for v in yaml.safe_load(
            LaunchConfiguration('all_spawn_offsets').perform(context))]
        return [Node(
            package='multirobot_bringup',
            executable='explore_controller',
            name='explore_controller',
            output='screen',
            parameters=[{
                'namespace': ns,
                'use_sim_time': ust,
                'all_namespaces': all_ns,
                'all_spawn_offsets': all_off,
            }],
        )]

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'namespace', default_value='robot1',
        description='Robot namespace (must match spawn + slam namespace)',
    ))
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo /clock',
    ))
    ld.add_action(DeclareLaunchArgument(
        'all_namespaces', default_value='[robot1, robot2]',
        description='All robot namespaces (for peer-repulsion subscriptions)',
    ))
    ld.add_action(DeclareLaunchArgument(
        'all_spawn_offsets', default_value='[0.0, 0.5, 0.0, 1.5]',
        description='Flat [x1,y1, x2,y2, ...] spawn offsets for all robots',
    ))
    ld.add_action(OpaqueFunction(function=_setup))
    return ld
