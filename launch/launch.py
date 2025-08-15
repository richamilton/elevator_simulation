import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():

    package_name='elevator_simulation'

    # Declare launch arguments
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='true',
        description='Enable verbose Gazebo output'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.world',
        description='Gazebo world file to load'
    )
    

    # Get package directory
    elevator_simulation_dir = get_package_share_directory(package_name)
    
    # Model paths
    building_sdf_path = os.path.join(elevator_simulation_dir, 'models', 'building', 'model.sdf')

    # Start Gazebo
    start_gazebo = ExecuteProcess(
        cmd=['ros2', 'launch', 'gazebo_ros', 'gazebo.launch.py', 'verbose:=true'],
        output='screen',
        name='gazebo'
    )

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
        period=12.0,  # Start scheduler after all elevators are spawned
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
        start_gazebo,
        spawn_building,
        spawn_elevators,
        elevator_scheduler
    ])
