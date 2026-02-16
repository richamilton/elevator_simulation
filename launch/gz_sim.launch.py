#!/usr/bin/env python3

# Copyright 2024 Husarion sp. z o.o.
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

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def launch_setup(context):
    gz_headless_mode = LaunchConfiguration("gz_headless_mode").perform(context)
    gz_log_level = LaunchConfiguration("gz_log_level").perform(context)
    world_name = LaunchConfiguration("world_name").perform(context)

    # Include the world-specific launch file, passing through Gazebo args
    # This pattern allows us to have a single gz_sim.launch.py that can launch any world by name, 
    #   as long as it has a corresponding <world_name>.launch.py in the launch/ directory
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("avdr_gz_worlds"), "launch", f"{world_name}.launch.py"]
            )
        ),
        launch_arguments={
            "gz_headless_mode": gz_headless_mode,
            "gz_log_level": gz_log_level,
        }.items()
    )

    return [world_launch]


def generate_launch_description():
    plugin_lib_path = os.path.join(
        get_package_share_directory('avdr_gz_worlds'), '..', '..', 'lib', 'avdr_gz_worlds'
    )
    set_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=[plugin_lib_path, ':', os.environ.get('GAZEBO_PLUGIN_PATH', '')]
    )

    declare_world_name = DeclareLaunchArgument(
        "world_name",
        default_value="avdr_building_w_elevator",
        description="Name of the world to launch. Must match a <world_name>.launch.py in the launch/ directory.",
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
            declare_world_name,
            declare_gz_headless_mode,
            declare_gz_log_level,
            OpaqueFunction(function=launch_setup)
        ]
    )
