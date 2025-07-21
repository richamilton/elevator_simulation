#!/usr/bin/env python3
"""
Simple launch file for testing a single elevator.
Useful for debugging and development.
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Generate launch description for single elevator testing."""
    
    # Declare launch arguments
    elevator_id_arg = DeclareLaunchArgument(
        'elevator_id',
        default_value='1',
        description='ID of the elevator to spawn'
    )
    
    x_pos_arg = DeclareLaunchArgument(
        'x_pos',
        default_value='0.0',
        description='X position of elevator'
    )
    
    y_pos_arg = DeclareLaunchArgument(
        'y_pos',
        default_value='0.0',
        description='Y position of elevator'
    )
    
    initial_floor_arg = DeclareLaunchArgument(
        'initial_floor',
        default_value='0',
        description='Initial floor (0-3)'
    )
    
    movement_speed_arg = DeclareLaunchArgument(
        'movement_speed',
        default_value='1.0',
        description='Elevator movement speed in m/s'
    )
    
    # Get package directory
    rexbot_dir = get_package_share_directory('rexbot')
    
    # Model paths
    elevator_car_model = os.path.join(rexbot_dir, 'models', 'elevator_car', 'model.sdf')
    elevator_shaft_model = os.path.join(rexbot_dir, 'models', 'elevator_shaft', 'model.sdf')
    
    # Start Gazebo
    start_gazebo = ExecuteProcess(
        cmd=['ros2', 'launch', 'gazebo_ros', 'gazebo.launch.py', 'verbose:=true'],
        output='screen',
        name='gazebo'
    )
    
    # Set elevator parameters
    set_param_speed = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'param', 'set', ['/elevator_controller_', LaunchConfiguration('elevator_id')],
                     'movement_speed', LaunchConfiguration('movement_speed')],
                output='screen',
                name='set_param_speed'
            )
        ]
    )
    
    set_param_floor = TimerAction(
        period=3.1,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'param', 'set', ['/elevator_controller_', LaunchConfiguration('elevator_id')],
                     'initial_floor', LaunchConfiguration('initial_floor')],
                output='screen',
                name='set_param_floor'
            )
        ]
    )
    
    set_param_stop_min = TimerAction(
        period=3.2,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'param', 'set', ['/elevator_controller_', LaunchConfiguration('elevator_id')],
                     'stop_duration_min', '10.0'],
                output='screen',
                name='set_param_stop_min'
            )
        ]
    )
    
    set_param_stop_max = TimerAction(
        period=3.3,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'param', 'set', ['/elevator_controller_', LaunchConfiguration('elevator_id')],
                     'stop_duration_max', '15.0'],
                output='screen',
                name='set_param_stop_max'
            )
        ]
    )
    
    set_param_door_time = TimerAction(
        period=3.4,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'param', 'set', ['/elevator_controller_', LaunchConfiguration('elevator_id')],
                     'door_operation_time', '2.0'],
                output='screen',
                name='set_param_door_time'
            )
        ]
    )
    
    set_param_doors_open = TimerAction(
        period=3.5,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'param', 'set', ['/elevator_controller_', LaunchConfiguration('elevator_id')],
                     'initial_doors_open', 'true'],
                output='screen',
                name='set_param_doors_open'
            )
        ]
    )
    
    # Spawn elevator car
    spawn_elevator = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                     '-entity', ['elevator_', LaunchConfiguration('elevator_id')],
                     '-file', elevator_car_model,
                     '-x', LaunchConfiguration('x_pos'),
                     '-y', LaunchConfiguration('y_pos'),
                     '-z', '0.0'],  # Will be adjusted by plugin based on initial_floor
                output='screen',
                name='spawn_elevator'
            )
        ]
    )
    
    # Monitor elevator status
    monitor_elevator = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'list'],
                output='screen',
                name='list_topics'
            )
        ]
    )
    
    return LaunchDescription([
        elevator_id_arg,
        x_pos_arg,
        y_pos_arg,
        initial_floor_arg,
        movement_speed_arg,
        start_gazebo,
        set_param_speed,
        set_param_floor,
        set_param_stop_min,
        set_param_stop_max,
        set_param_door_time,
        set_param_doors_open,
        spawn_elevator,
        monitor_elevator
    ])