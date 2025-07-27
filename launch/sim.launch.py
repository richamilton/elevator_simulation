import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():

    package_name='rexbot'

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
    rexbot_dir = get_package_share_directory(package_name)
    
    # Model paths
    building_sdf_path = os.path.join(rexbot_dir, 'models', 'building', 'model.sdf')

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

    # twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    # twist_mux = Node(
    #         package="twist_mux",
    #         executable="twist_mux",
    #         parameters=[twist_mux_params, {'use_sim_time': True}],
    #         remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    #     )

    # # Include the Gazebo launch file, provided by the gazebo_ros package
    # gazebo = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    #                 launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_path }.items()
    #          )

    # # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    # spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
    #                     arguments=['-topic', 'robot_description',
    #                                '-entity', 'my_bot'],
    #                     output='screen')
    
    # diff_drive_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["diff_cont"],
    # )

    # joint_broad_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_broad"],
    # )

    # node_goal_publisher = Node(
    #     package='articubot_one',
    #     executable='goal_publisher_node.py',
    #     output='screen'
    # )

    # Launch them all!
    return LaunchDescription([
        start_gazebo,
        spawn_building,
        spawn_elevators,
        # twist_mux,
        # gazebo,
        # spawn_entity,
        # diff_drive_spawner,
        # joint_broad_spawner,
        # node_goal_publisher
    ])
