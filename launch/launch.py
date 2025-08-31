import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():

    package_name='elevator_simulation'

    # Get package directory
    elevator_simulation_dir = get_package_share_directory(package_name)
    
    # Model paths
    building_sdf_path = os.path.join(elevator_simulation_dir, 'models', 'building', 'model.sdf')

    spawn_building = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                     '-entity', ['building'],
                     '-file', building_sdf_path,
                     '-x', '0.0',
                     '-y', '0.0',
                     '-z', '0.0',
                     '-R', '0.0',
                     '-P', '0.0',
                     '-Y', '0.0'
                ],
                output='screen',
                name='spawn_building'
            )
        ]
    )

    spawn_elevators = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','spawn_elevators.launch.py'
                )])
    )

    elevator_scheduler = TimerAction(
        period=16.0,  # Increased from 12.0 to 16.0 seconds to ensure all elevators are fully spawned before starting the scheduler.
        # This accounts for possible delays in the elevator spawning process and avoids race conditions.
        actions=[
            Node(
                package='elevator_simulation',
                executable='elevator_scheduler.py',
                name='elevator_scheduler',
                output='screen'
            )
        ]
    )

    # Launch them all!
    return LaunchDescription([
        spawn_building,
        spawn_elevators,
        elevator_scheduler
    ])
