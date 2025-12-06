cmake_minimum_required(VERSION 3.8)
project(gpmp2_planner)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# GPMP2/GTSAM/OpenCV
find_package(GTSAM REQUIRED)
find_package(gpmp2 REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories for GTSAM and OpenCV
include_directories(
  ${GTSAM_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  /usr/local/include
)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanPath.srv"
  DEPENDENCIES nav_msgs geometry_msgs
)

# C++ service node
add_executable(gpmp2_service_node src/gpmp2_service_node.cpp)

ament_target_dependencies(gpmp2_service_node
  rclcpp
  nav_msgs
  geometry_msgs
)

# Link to generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(gpmp2_service_node 
  "${cpp_typesupport_target}" 
  gtsam 
  gpmp2 
  ${OpenCV_LIBS}
)

# Install C++ executables
install(TARGETS
  gpmp2_service_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/planner_node.py
  scripts/path_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config directories
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
