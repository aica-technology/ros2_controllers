cmake_minimum_required(VERSION 3.5)
project(position_controllers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(forward_command_controllers REQUIRED)

add_library(${PROJECT_NAME}
  SHARED
  src/joint_position_controller.cpp
  src/joint_group_position_controller.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME}
  builtin_interfaces
  hardware_interface
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcutils
  std_msgs
  forward_command_controllers
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "POSITION_CONTROLLERS_BUILDING_DLL")

pluginlib_export_plugin_description_file(controller_interface position_controllers_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(test_robot_hardware REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_joint_position_controller test/test_joint_position_controller.cpp)
  target_include_directories(test_joint_position_controller PRIVATE include)
  ament_target_dependencies(test_joint_position_controller
    controller_manager
    test_robot_hardware
    forward_command_controllers
  )

  ament_add_gtest(test_joint_group_position_controller test/test_joint_group_position_controller.cpp)
  target_include_directories(test_joint_group_position_controller PRIVATE include)
  ament_target_dependencies(test_joint_group_position_controller
    controller_manager
    test_robot_hardware
    forward_command_controllers
  )
endif()

ament_export_dependencies(
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  std_msgs
  forward_command_controllers
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()