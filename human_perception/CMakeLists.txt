cmake_minimum_required(VERSION 3.8)
project(human_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(soloco_interfaces REQUIRED)
find_package(zed_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(human_track_publisher src/human_track_publisher.cpp)
ament_target_dependencies(human_track_publisher
        rclcpp
        visualization_msgs
        nav_msgs
        soloco_interfaces
        zed_interfaces
        tf2_ros
        tf2_sensor_msgs
        tf2_geometry_msgs)
target_include_directories(human_track_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(human_track_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS human_track_publisher
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
