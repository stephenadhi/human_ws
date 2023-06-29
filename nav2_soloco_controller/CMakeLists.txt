cmake_minimum_required(VERSION 3.8)
project(nav2_soloco_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(soloco_interfaces REQUIRED)
find_package(zed_interfaces REQUIRED)

include_directories(include)

set(dependencies_pkgs
  rclcpp
  rclpy
  nav2_common
  pluginlib
  tf2
  geometry_msgs
  sensor_msgs
  visualization_msgs
  action_msgs
  std_msgs
  nav_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  tf2_geometry_msgs
  tf2_eigen
  tf2_ros
  soloco_interfaces
)

foreach(pkg IN LISTS dependencies_pkgs)
  find_package(${pkg} REQUIRED)
endforeach()

nav2_package()

include(CheckCXXCompilerFlag)

check_cxx_compiler_flag("-mavx2" COMPILER_SUPPORTS_AVX2)
check_cxx_compiler_flag("-mfma" COMPILER_SUPPORTS_FMA)

if(COMPILER_SUPPORTS_AVX2)
  add_compile_options(-mavx2)
endif()

if(COMPILER_SUPPORTS_FMA)
  add_compile_options(-mfma)
endif()

add_compile_options(-O3 -finline-limit=1000000 -ffp-contract=fast -ffast-math)

add_library(soloco_controller SHARED
  src/controller.cpp
#   src/optimizer.cpp
#   src/critic_manager.cpp
#   src/trajectory_visualizer.cpp
#   src/path_handler.cpp
#   src/parameters_handler.cpp
#   src/noise_generator.cpp
)

set(libraries soloco_controller)

foreach(lib IN LISTS libraries)
  target_compile_options(${lib} PUBLIC -fconcepts)
  ament_target_dependencies(${lib} ${dependencies_pkgs})
endforeach()

install(TARGETS soloco_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

add_executable(subgoal_publisher src/subgoal_publisher.cpp)
ament_target_dependencies(subgoal_publisher
       rclcpp
       geometry_msgs
       nav_msgs
       visualization_msgs
       )
install(TARGETS subgoal_publisher
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install python executables
install(PROGRAMS
        ${PROJECT_NAME}/global_nav2_client.py
        DESTINATION lib/${PROJECT_NAME}
        )
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
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(${libraries})
ament_export_dependencies(${dependencies_pkgs})
ament_export_include_directories(include)
pluginlib_export_plugin_description_file(nav2_core solococ.xml)

ament_package()
