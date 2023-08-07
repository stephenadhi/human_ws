cmake_minimum_required(VERSION 3.8)
project(soloco_evaluator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy  REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(soloco_interfaces REQUIRED)

set(dependencies
    rclcpp
    soloco_interfaces
    geometry_msgs
    nav_msgs
)

include_directories(include)
add_executable(prediction_evaluator src/prediction_evaluator.cpp)
ament_target_dependencies(prediction_evaluator ${dependencies})

install(TARGETS prediction_evaluator
  DESTINATION lib/${PROJECT_NAME})


# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install python executables
install(PROGRAMS
        ${PROJECT_NAME}/soloco_evaluator_node.py
        DESTINATION lib/${PROJECT_NAME}
        )

# Install launch files and directories.
install(DIRECTORY
        src include launch config ${PROJECT_NAME}
        DESTINATION share/${PROJECT_NAME}
        )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
