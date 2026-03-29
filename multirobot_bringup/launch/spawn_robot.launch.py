#Spawn a single TurtleBot3 Waffle into Gazebo with the ros_gz_bridge

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions.command import Command
from launch.substitutions.find_executable import FindExecutable
from launch_ros.actions import Node



def generate_launch_description():
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')
    # At the top, read the URDF file
    urdf = os.path.join(sim_dir, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf, 'r') as f:
        robot_description = f.read()


    namespace = LaunchConfiguration('namespace')
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')
    pose = {
        'x': LaunchConfiguration('x_pose', default='0.0'),
        'y': LaunchConfiguration('y_pose', default='0.0'),
        'z': LaunchConfiguration('z_pose', default='0.01'),
        'R': LaunchConfiguration('roll', default='0.0'),
        'P': LaunchConfiguration('pitch', default='0.0'),
        'Y': LaunchConfiguration('yaw', default='0.0'),
    }

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='robot1',
        description='Robot namespace (e.g., robot1, robot2, robot3)',
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='turtlebot3_waffle',
        description='Name of the robot in Gazebo',
    )

    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(sim_dir, 'urdf', 'gz_waffle.sdf.xacro'),
        description='Full path to robot SDF xacro file',
    )

    # Gazebo-ROS bridge: bridges all sensor/cmd topics under the robot namespace.
    # With expand_gz_topic_names=True the bridge automatically maps
    # <namespace>/scan (Gz) -> <namespace>/scan (ROS), etc.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        parameters=[
            {
                'config_file': os.path.join(
                    sim_dir, 'configs', 'turtlebot3_waffle_bridge.yaml'
                ),
                'expand_gz_topic_names': True,
                'use_sim_time': True,
            }
        ],
        output='screen',
    )

    # Spawn the robot model into Gazebo with the given namespace and pose.
    # The xacro namespace arg prefixes all Gazebo plugin topics.
    spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        namespace=namespace,
        arguments=[
            '-name', robot_name,
            '-string', Command([
                FindExecutable(name='xacro'), ' ',
                'namespace:=', namespace, ' ',
                robot_sdf,
            ]),
            '-x', pose['x'],
            '-y', pose['y'],
            '-z', pose['z'],
            '-R', pose['R'],
            '-P', pose['P'],
            '-Y', pose['Y'],
        ],
    )

    # Add this node
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(bridge)
    ld.add_action(spawn_model)
    ld.add_action(robot_state_pub)

    return ld
