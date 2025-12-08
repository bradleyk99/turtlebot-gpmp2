#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

def generate_launch_description():
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='world')
    
    # Package directories
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    gpmp2_planner_dir = get_package_share_directory('gpmp2_planner')
    
    # Set TurtleBot3 model
    os.environ['TURTLEBOT3_MODEL'] = 'burger'
    turtlebot3_model = 'burger'
    
    # Map file path (ADDED)
    map_file = os.path.join(gpmp2_planner_dir, 'maps', 'my_map.yaml')
    
    # Check if map exists
    if not os.path.exists(map_file):
        print("\n" + "="*60)
        print("WARNING: No map file found!")
        print(f"Expected at: {map_file}")
        print("\nTo create a map, first run SLAM, then save with:")
        print("  ros2 run nav2_map_server map_saver_cli -f <workspace>/src/turtlebot-gpmp2/maps/my_map")
        print("="*60 + "\n")
    
    # Robot Description (ADDED for robot model visualization)
    urdf_file_path = os.path.join(turtlebot3_gazebo_dir, 'urdf', f'turtlebot3_{turtlebot3_model}.urdf')
    with open(urdf_file_path, 'r') as urdf_file:
        robot_description = urdf_file.read()
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )
    
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Gazebo - house world
    gazebo_house = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_house.launch.py')
        ),
        condition=IfCondition(PythonExpression(["'", world, "' == 'house'"]))
    )
    
    # Gazebo - standard world
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        ),
        condition=IfCondition(PythonExpression(["'", world, "' == 'world'"]))
    )
    
    # Gazebo - empty world
    gazebo_empty = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'empty_world.launch.py')
        ),
        condition=IfCondition(PythonExpression(["'", world, "' == 'empty'"]))
    )
    
    # Map Server (REPLACES Cartographer)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'use_sim_time': False
        }]
    )
    
    # Lifecycle Manager (ADDED - activates map server)
    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server'],
            'use_sim_time': False
        }]
    )
    
    # Static TF for localization (ADDED - perfect localization from Gazebo)
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # GPMP2 service node (C++)
    gpmp2_service_node = Node(
        package='gpmp2_planner',
        executable='gpmp2_service_node',
        name='gpmp2_service_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Planner node (Python)
    planner_node = Node(
        package='gpmp2_planner',
        executable='planner_node.py',
        name='planner_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Path follower node (Python)
    path_follower_node = Node(
        package='gpmp2_planner',
        executable='path_follower.py',
        name='path_follower',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(gpmp2_planner_dir, 'config', 'gpmp2.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Delay rviz so the map server has time to open
    rviz_delayed = TimerAction(
        period = 5.0,
        actions=[rviz_node]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='world',
                              description='World to load: house, world, or empty'),
        gazebo_house,
        gazebo_world,
        gazebo_empty,
        robot_state_publisher,      # ADDED
        joint_state_publisher,      # ADDED
        static_tf_map_odom,         # ADDED
        gpmp2_service_node,
        planner_node,
        path_follower_node,
        rviz_node,
    ])