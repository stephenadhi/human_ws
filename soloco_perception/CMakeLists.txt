cmake_minimum_required(VERSION 3.8)
project(soloco_human_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy  REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(zed_interfaces REQUIRED)
find_package(soloco_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies
    rclcpp
    soloco_interfaces
    tf2_geometry_msgs
    tf2_ros
    tf2
)

include_directories(
include
${GSL_INCLUDE_DIRS}
)

add_executable(human_tf2_publisher src/human_tf2_publisher.cpp)
ament_target_dependencies(human_tf2_publisher ${dependencies})

install(TARGETS
  human_tf2_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install python executables
install(PROGRAMS
        ${PROJECT_NAME}/human_tracks.py
        DESTINATION lib/${PROJECT_NAME}
        )

install(PROGRAMS
        ${PROJECT_NAME}/robot_track.py
        DESTINATION lib/${PROJECT_NAME}
        )

install(PROGRAMS
        ${PROJECT_NAME}/multi_track_visualizer.py
        DESTINATION lib/${PROJECT_NAME}
        )

# Install launch files and directories.
install(DIRECTORY
        src launch include config rviz ${PROJECT_NAME}
        DESTINATION share/${PROJECT_NAME}
        )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()