#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='world')
    
    # Package directories
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_cartographer_dir = get_package_share_directory('turtlebot3_cartographer')
    gpmp2_planner_dir = get_package_share_directory('gpmp2_planner')
    
    # Set TurtleBot3 model
    os.environ['TURTLEBOT3_MODEL'] = 'burger'
    
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
    
    # Cartographer SLAM
    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_cartographer_dir, 'launch', 'cartographer.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
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
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='house',
                              description='World to load: house, world, or empty'),
        gazebo_house,
        gazebo_world,
        gazebo_empty,
        cartographer_launch,
        gpmp2_service_node,
        planner_node,
        path_follower_node,
    ])