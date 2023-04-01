cmake_minimum_required(VERSION 3.8)
project(human_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(soloco_interfaces REQUIRED)
find_package(zed_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

include_directories(${GSL_INCLUDE_DIRS})

add_executable(human_track_publisher src/human_track_publisher.cpp)
ament_target_dependencies(human_track_publisher
        rclcpp
        visualization_msgs
        geometry_msgs
        nav_msgs
        soloco_interfaces
        zed_interfaces
        tf2_ros
        tf2_sensor_msgs 
        tf2_geometry_msgs
        Eigen3)
        
add_executable(robot_track_publisher src/robot_track_publisher.cpp)
ament_target_dependencies(robot_track_publisher
        rclcpp
        visualization_msgs
        geometry_msgs
        nav_msgs
        soloco_interfaces
        zed_interfaces
        tf2_ros
        tf2_sensor_msgs 
        tf2_geometry_msgs)
                
add_executable(ego_trajectory_visualizer src/ego_trajectory_visualizer.cpp)
ament_target_dependencies(ego_trajectory_visualizer
        rclcpp
        visualization_msgs
        geometry_msgs
        nav_msgs
        soloco_interfaces
        zed_interfaces
        tf2_ros
        tf2_sensor_msgs 
        tf2_geometry_msgs)
                
install(TARGETS human_track_publisher robot_track_publisher ego_trajectory_visualizer
  DESTINATION lib/${PROJECT_NAME})

# Install launch files and directories.
install(DIRECTORY
        src launch include config
        DESTINATION share/${PROJECT_NAME}
        )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
