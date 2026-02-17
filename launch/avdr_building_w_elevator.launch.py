#!/usr/bin/env python3

# Copyright 2026 ThynkSpace.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
    gazebo_launch_file = "gzserver.launch.py" if gz_headless_mode.lower() == "true" else "gazebo.launch.py"
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
            declare_gz_headless_mode,
            declare_gz_log_level,
            OpaqueFunction(function=launch_setup)
        ]
    )
