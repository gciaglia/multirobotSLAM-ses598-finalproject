"""Launch Gazebo with a world file (supports both .sdf and .sdf.xacro)."""

import os
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration


# Find the repo root by walking up from this file until we find space_robotics_gz_envs
def _find_repo_root():
    path = Path(__file__).resolve()
    while path != path.parent:
        if (path / 'space_robotics_gz_envs').is_dir():
            return str(path)
        path = path.parent
    # Fallback: assume standard repo layout
    return str(Path.home() / 'Desktop' / 'SES598' / 'FinalProject' /
               'multirobotSLAM-ses598-finalproject')

_REPO_ROOT = _find_repo_root()
_ASSETS_CACHE_DIR = os.path.join(_REPO_ROOT, 'space_robotics_gz_envs', 'assets', 'cache')


def generate_launch_description():
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')
    bringup_dir = get_package_share_directory('multirobot_bringup')

    world = LaunchConfiguration('world')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(bringup_dir, 'worlds', 'mars_exploration.sdf'),
        description='Full path to world SDF file (.sdf or .sdf.xacro)',
    )

    # Nav2 TB3 models (needed for robot spawning)
    set_env_nav2_models = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(sim_dir, 'models'))
    set_env_nav2_parent = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        str(Path(sim_dir).parent.resolve()))

    # Procgen assets from space_robotics_gz_envs (martian_rock0, martian_surface0, etc.)
    set_env_space_assets = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', _ASSETS_CACHE_DIR)

    # Custom worlds shipped with this package
    set_env_bringup_worlds = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(bringup_dir, 'worlds'))

    # Process xacro to SDF (needed for .sdf.xacro worlds like tb3_sandbox)
    world_sdf = tempfile.mktemp(prefix='multirobot_', suffix='.sdf')
    world_sdf_xacro = ExecuteProcess(
        cmd=['xacro', '-o', world_sdf, ['headless:=', 'False'], world])

    # Start Gazebo server
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_sdf],
        output='screen',
    )

    # Start Gazebo GUI
    start_gazebo_gui_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen',
    )

    # Clean up temp SDF on shutdown
    remove_temp_sdf_file = RegisterEventHandler(event_handler=OnShutdown(
        on_shutdown=[
            OpaqueFunction(function=lambda _: os.remove(world_sdf)
                           if os.path.exists(world_sdf) else None)
        ]))

    ld = LaunchDescription()
    ld.add_action(set_env_nav2_models)
    ld.add_action(set_env_nav2_parent)
    ld.add_action(set_env_space_assets)
    ld.add_action(set_env_bringup_worlds)
    ld.add_action(declare_world_cmd)
    ld.add_action(world_sdf_xacro)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_gazebo_gui_cmd)
    ld.add_action(remove_temp_sdf_file)

    return ld
