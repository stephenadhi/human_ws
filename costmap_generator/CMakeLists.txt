cmake_minimum_required(VERSION 3.8)
project(costmap_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(soloco_interfaces REQUIRED)
find_package(zed_interfaces REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)

add_executable(laserscan_box_filter src/laserscan_box_filter.cpp)
ament_target_dependencies(laserscan_filter_node
       rclcpp
       geometry_msgs
       nav_msgs
       sensor_msgs
       soloco_interfaces
       zed_interfaces
       tf2_ros
       tf2_sensor_msgs 
       tf2_geometry_msgs)
target_include_directories(laserscan_box_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(laserscan_box_filter PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS laserscan_box_filter
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
