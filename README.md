# Dependencies
- GPMP2 Library (Requires GTSAM Library which is also used)
- ROS2 Jazzy

# Launch Command
Following sourcing the directory to launch the entire simulation in Gazebo and RVIZ:
`ros2 launch gpmp2_planner gpmp2_planning.launch.py`

The specific simulation world can be selected from one of the three built in Turtlebot3 simulation world through
`ros2 launch gpm2_planner gpmp2_planning.launch.py world:="house"`
"house", "world", and "empty" are available

Within RVIZ, goal pose can be set to direct the robot to use GPMP2 to plan a path to the goal pose. To view the path trajectory, add it by topic in RVIZ to view.
