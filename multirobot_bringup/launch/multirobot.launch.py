"""One-shot bringup: Gazebo + N robots + per-robot SLAM + map merger.

Single source of truth for namespaces and spawn poses, so the merger and the
spawner can't drift apart. Override defaults via launch args:

    ros2 launch multirobot_bringup multirobot.launch.py \\
        robot_namespaces:='[robot1, robot2, robot3]' \\
        robot_initial_poses:='[0.0, 0.0, 2.0, 0.5, -2.0, 0.5]'

Robots are spawned a few seconds after Gazebo starts so the spawner doesn't
race the simulator.
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    bringup_share = get_package_share_directory('multirobot_bringup')
    launch_dir = os.path.join(bringup_share, 'launch')
    default_world = os.path.join(bringup_share, 'worlds', 'mars_exploration.sdf')
    no_clock_bridge = os.path.join(
        bringup_share, 'config', 'turtlebot3_waffle_bridge_no_clock.yaml')

    namespaces = yaml.safe_load(
        LaunchConfiguration('robot_namespaces').perform(context)
    )
    flat = [float(v) for v in yaml.safe_load(
        LaunchConfiguration('robot_initial_poses').perform(context)
    )]
    if len(flat) != 2 * len(namespaces):
        raise ValueError(
            f'robot_initial_poses must have 2 entries per namespace '
            f'(got {len(flat)} for {len(namespaces)} robots)'
        )
    poses = [(flat[2 * i], flat[2 * i + 1]) for i in range(len(namespaces))]

    world = LaunchConfiguration('world').perform(context) or default_world
    spawn_delay = float(LaunchConfiguration('spawn_delay_sec').perform(context))
    slam_delay = float(LaunchConfiguration('slam_delay_sec').perform(context))

    actions = []

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'simulation_launch.py')),
        launch_arguments={'world': world}.items(),
    ))

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # Anchor the map_merged frame in the global TF tree so RViz can render
    # the merged occupancy grid. Identity transform under a synthetic root.
    map_merged_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_merged_static_tf',
        arguments=['--frame-id', 'world', '--child-frame-id', 'map_merged'],
        output='screen',
    )

    spawn_actions = []
    slam_actions = []
    controller_actions = []
    for i, (ns, (x, y)) in enumerate(zip(namespaces, poses)):
        spawn_args = {
            'namespace': ns,
            'x_pose': str(x),
            'y_pose': str(y),
            'z_pose': '0.5',
            'bridge_config': no_clock_bridge,
        }
        if i > 0:
            spawn_args['robot_name'] = f'turtlebot3_waffle_{i + 1}'
        spawn_actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'spawn_robot.launch.py')),
            launch_arguments=spawn_args.items(),
        ))
        slam_actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam_per_robot.launch.py')),
            launch_arguments={'namespace': ns}.items(),
        ))
        controller_actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'explore_controller.launch.py')),
            launch_arguments={
                'namespace': ns,
                'use_sim_time': 'true',
                'all_namespaces': str(namespaces).replace("'", ""),
                'all_spawn_offsets': str(flat),
            }.items(),
        ))

    actions.append(TimerAction(period=spawn_delay,
                                actions=[clock_bridge, map_merged_tf]
                                        + spawn_actions))
    actions.append(TimerAction(period=slam_delay,
                                actions=slam_actions + controller_actions))

    actions.append(TimerAction(period=slam_delay, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'map_merger.launch.py')),
            launch_arguments={
                'robot_namespaces': str(namespaces).replace("'", ""),
                'robot_initial_poses': str(flat),
                'use_sim_time': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'frontier_detector.launch.py')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'explorer.launch.py')),
            launch_arguments={
                'robot_namespaces': str(namespaces).replace("'", ""),
                'robot_initial_poses': str(flat),
                'selection_strategy': LaunchConfiguration(
                    'selection_strategy').perform(context),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'metrics_logger.launch.py')),
            launch_arguments={
                'robot_namespaces': str(namespaces).replace("'", ""),
                'robot_initial_poses': str(flat),
            }.items(),
        ),
    ]))

    return actions


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
        'world', default_value='',
        description='World SDF/xacro path. Empty uses simulation_launch default (Mars).',
    ))
    ld.add_action(DeclareLaunchArgument(
        'spawn_delay_sec', default_value='5.0',
        description='Seconds to wait after Gazebo start before spawning robots',
    ))
    ld.add_action(DeclareLaunchArgument(
        'slam_delay_sec', default_value='8.0',
        description='Seconds to wait after Gazebo start before launching SLAM and merger',
    ))
    ld.add_action(DeclareLaunchArgument(
        'selection_strategy', default_value='ucb1',
        description='Frontier selection strategy: ucb1 (default), random, nearest, furthest, spread',
    ))
    ld.add_action(OpaqueFunction(function=_setup))
    return ld
