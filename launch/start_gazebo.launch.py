import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    package_name='elevator_simulation'

    # Declare launch arguments
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='true',
        description='Enable verbose Gazebo output'
    )

    # Start Gazebo
    start_gazebo = ExecuteProcess(
        cmd=['ros2', 'launch', 'gazebo_ros', 'gazebo.launch.py', f'verbose:={LaunchConfiguration("verbose")}'],
        output='screen',
        name='gazebo'
    )

    launch_elevator_simulation = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','launch.py'
                )])
    )

    # Launch them all!
    return LaunchDescription([
        verbose_arg,
        start_gazebo,
        launch_elevator_simulation
    ])
