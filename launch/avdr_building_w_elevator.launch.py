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
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def launch_setup(context):
    gz_headless_mode = LaunchConfiguration("gz_headless_mode").perform(context)
    gz_log_level = LaunchConfiguration("gz_log_level").perform(context)
    gz_gui = LaunchConfiguration("gz_gui").perform(context)

    package_dir = get_package_share_directory('avdr_gz_worlds')
    world_sdf_path = os.path.join(package_dir, 'worlds', 'avdr_building_w_elevator.sdf')

    gz_args = f"-r -v {gz_log_level} {world_sdf_path}"
    if eval(gz_headless_mode):
        gz_args = "--headless-rendering -s " + gz_args
    if gz_gui:
        gz_args = f"--gui-config {gz_gui} " + gz_args

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items()
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
    declare_gz_gui = DeclareLaunchArgument(
        "gz_gui",
        default_value=PathJoinSubstitution(
            [FindPackageShare("husarion_gz_worlds"), "config", "teleop.config"]
        ),
        description="Run simulation with specific GUI layout.",
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
            declare_gz_gui,
            declare_gz_headless_mode,
            declare_gz_log_level,
            OpaqueFunction(function=launch_setup)
        ]
    )
