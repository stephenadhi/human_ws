cmake_minimum_required(VERSION 3.8)
project(social_motion_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(soloco_interfaces REQUIRED)
find_package(zed_interfaces REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)

#add_executable(laserscan_filter_node src/laserscan_filter_node.cpp)
#ament_target_dependencies(laserscan_filter_node
#        rclcpp
#        visualization_msgs
#        geometry_msgs
#        nav_msgs
#        sensor_msgs
#        soloco_interfaces
#        zed_interfaces
#        tf2_ros
#        tf2_sensor_msgs 
#        tf2_geometry_msgs)

#install(TARGETS laserscan_filter_node
#        DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install path_calculator executables
install(PROGRAMS
        ${PROJECT_NAME}/neural_motion_planner.py
        DESTINATION lib/${PROJECT_NAME}
        )

# Install launch files and directories.
install(DIRECTORY
        src launch include config ${PROJECT_NAME}
        DESTINATION share/${PROJECT_NAME}
        )
        
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
