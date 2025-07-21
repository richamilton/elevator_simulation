#!/usr/bin/env python3
"""
Launch file for spawning multiple elevators with configuration.
This file spawns 4 elevators at different positions with individual settings.
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Generate launch description for multiple elevator simulation."""
    
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
    
    num_elevators_arg = DeclareLaunchArgument(
        'num_elevators',
        default_value='4',
        description='Number of elevators to spawn'
    )
    
    # Get package directories
    rexbot_dir = get_package_share_directory('rexbot')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    
    # Model and config paths
    elevator_car_model = os.path.join(rexbot_dir, 'models', 'elevator_car', 'model.sdf')
    elevator_shaft_model = os.path.join(rexbot_dir, 'models', 'elevator_shaft', 'model.sdf')
    config_file = os.path.join(rexbot_dir, 'config', 'elevator_schedule.yaml')
    
    # Start Gazebo
    start_gazebo = ExecuteProcess(
        cmd=['ros2', 'launch', 'gazebo_ros', 'gazebo.launch.py', 'verbose:=true'],
        output='screen',
        name='gazebo'
    )
    
    # Load configuration
    elevator_configs = []
    try:
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
            
        # Extract elevator configurations
        building_config = config.get('building', {})
        defaults = config.get('elevator_defaults', {})
        elevators = config.get('elevators', {})
        
        for elevator_name, elevator_config in elevators.items():
            elevator_id = elevator_config.get('id', 1)
            position = elevator_config.get('position', {'x': 0.0, 'y': 0.0, 'z': 0.0})
            
            # Merge defaults with individual config
            merged_config = {
                'id': elevator_id,
                'position': position,
                'initial_floor': elevator_config.get('initial_floor', 0),
                'initial_doors_open': elevator_config.get('initial_doors_open', True),
                'movement_speed': defaults.get('movement_speed', 1.0),
                'door_operation_time': defaults.get('door_operation_time', 2.0),
                'stop_duration_min': defaults.get('stop_duration', {}).get('min', 10.0),
                'stop_duration_max': defaults.get('stop_duration', {}).get('max', 15.0),
                'stop_duration_multiplier': elevator_config.get('schedule', {}).get('stop_duration_multiplier', 1.0),
                'floor_heights': building_config.get('floor_heights', [0.0, 2.5, 5.0, 7.5])
            }
            elevator_configs.append(merged_config)
            
    except (FileNotFoundError, yaml.YAMLError) as e:
        print(f"Warning: Could not load config file {config_file}: {e}")
        print("Using default elevator configurations")
        
        # Default configurations if config file not found
        elevator_configs = [
            {'id': 1, 'position': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'initial_floor': 0, 'movement_speed': 1.0},
            {'id': 2, 'position': {'x': 5.0, 'y': 0.0, 'z': 0.0}, 'initial_floor': 1, 'movement_speed': 1.0},
            {'id': 3, 'position': {'x': 10.0, 'y': 0.0, 'z': 0.0}, 'initial_floor': 2, 'movement_speed': 1.0},
            {'id': 4, 'position': {'x': 15.0, 'y': 0.0, 'z': 0.0}, 'initial_floor': 3, 'movement_speed': 1.0}
        ]
    
    # Create launch actions list
    launch_actions = [verbose_arg, world_arg, num_elevators_arg, start_gazebo]

    # Read the base model file
    with open(elevator_car_model, 'r') as f:
        base_sdf_content = f.read()
    
    # Spawn elevator cars with plugins
    for i, config in enumerate(elevator_configs):
        position = config['position']
        initial_height = config.get('floor_heights', [0.0, 2.5, 5.0, 7.5])[config.get('initial_floor', 0)]
        
        # Create parameter setting commands
        param_commands = []
        param_node = f"/elevator_controller_{config['id']}"
        
        param_commands.extend([
            ['ros2', 'param', 'set', param_node, 'movement_speed', str(config.get('movement_speed', 1.0))],
            ['ros2', 'param', 'set', param_node, 'door_operation_time', str(config.get('door_operation_time', 2.0))],
            ['ros2', 'param', 'set', param_node, 'stop_duration_min', str(config.get('stop_duration_min', 10.0))],
            ['ros2', 'param', 'set', param_node, 'stop_duration_max', str(config.get('stop_duration_max', 15.0))],
            ['ros2', 'param', 'set', param_node, 'stop_duration_multiplier', str(config.get('stop_duration_multiplier', 1.0))],
            ['ros2', 'param', 'set', param_node, 'initial_floor', str(config.get('initial_floor', 0))],
            ['ros2', 'param', 'set', param_node, 'initial_doors_open', str(config.get('initial_doors_open', True)).lower()],
            ['ros2', 'param', 'set', param_node, 'floor_heights', str(config.get('floor_heights', [0.0, 2.5, 5.0, 7.5]))]
        ])
        
        # Set parameters first
        for j, param_cmd in enumerate(param_commands):
            set_param = TimerAction(
                period=5.0 + i * 2.0 + j * 0.1,  # Stagger parameter setting
                actions=[
                    ExecuteProcess(
                        cmd=param_cmd,
                        output='screen',
                        name=f'set_param_{config["id"]}_{j}'
                    )
                ]
            )
            launch_actions.append(set_param)
        
        # Create modified SDF with plugin for this elevator
        plugin_xml = f'''
    <plugin name="elevator_controller" filename="libelevator_controller.so">
        <elevator_id>{config["id"]}</elevator_id>
    </plugin>
</model>'''
        
        # Insert plugin before closing </model> tag
        modified_sdf = base_sdf_content.replace('</model>', plugin_xml)
        
        # Write temporary file for this elevator
        temp_sdf_path = f"/tmp/elevator_{config['id']}_model.sdf"
        with open(temp_sdf_path, 'w') as f:
            f.write(modified_sdf)
        
        # Spawn elevator car with plugin
        spawn_elevator = TimerAction(
            period=6.0 + i * 2.0,  # Spawn after parameters are set
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                         '-entity', f'elevator_{config["id"]}',
                         '-file', temp_sdf_path,
                         '-x', str(position['x']),
                         '-y', str(position['y']),
                         '-z', str(position['z'] + initial_height)],
                    output='screen',
                    name=f'spawn_elevator_{config["id"]}'
                )
            ]
        )
        launch_actions.append(spawn_elevator)
    
    # Add status monitoring
    monitor_elevators = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c', 'echo "Elevator simulation started. Monitoring topics..." && ros2 topic list | grep elevator'],
                output='screen',
                name='monitor_elevators'
            )
        ]
    )
    launch_actions.append(monitor_elevators)
    
    return LaunchDescription(launch_actions)


def main():
    """For testing launch file directly."""
    import sys
    from launch import LaunchService
    
    # Create launch service
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    
    try:
        ls.run()
    except KeyboardInterrupt:
        print("\nShutting down elevator simulation...")
    finally:
        ls.shutdown()


if __name__ == '__main__':
    main()