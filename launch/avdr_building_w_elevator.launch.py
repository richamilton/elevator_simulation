#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def launch_setup(context):
    gz_headless_mode = LaunchConfiguration("gz_headless_mode").perform(context)
    gz_log_level = LaunchConfiguration("gz_log_level").perform(context)

    package_dir = get_package_share_directory('avdr_gz_worlds')
    world_sdf_path = os.path.join(package_dir, 'worlds', 'avdr_building_w_elevator.sdf')

    # Launch Classic Gazebo with the building world loaded at startup
    gazebo_launch_file = "gzserver.launch.py" if eval(gz_headless_mode) else "gazebo.launch.py"
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", gazebo_launch_file])
        ),
        launch_arguments={
            "world": world_sdf_path,
            "verbose": "true" if int(gz_log_level) >= 3 else "false",
            "pause": "false",
        }.items()
    )

    # Start elevator scheduler after building + elevators are fully spawned
    elevator_scheduler = TimerAction(
        period=16.0,
        actions=[
            Node(
                package='avdr_gz_worlds',
                executable='elevator_scheduler.py',
                name='elevator_scheduler',
                output='screen',
            )
        ]
    )

    return [gz_sim, elevator_scheduler]


def generate_launch_description():
    plugin_lib_path = os.path.join(
        get_package_share_directory('avdr_gz_worlds'), '..', '..', 'lib', 'avdr_gz_worlds'
    )
    set_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=[plugin_lib_path, ':', os.environ.get('GAZEBO_PLUGIN_PATH', '')]
    )

    declare_gz_headless_mode = DeclareLaunchArgument(
        "gz_headless_mode",
        default_value="False",
        description="Run the simulation in headless mode.",
        choices=["True", "False"],
    )

    declare_gz_log_level = DeclareLaunchArgument(
        "gz_log_level",
        default_value="2",
        description="Adjust the level of console output.",
        choices=["0", "1", "2", "3", "4"],
    )

    return LaunchDescription(
        [
            set_plugin_path,
            declare_gz_headless_mode,
            declare_gz_log_level,
            OpaqueFunction(function=launch_setup)
        ]
    )
