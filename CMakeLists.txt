cmake_minimum_required(VERSION 3.8)
project(roberto_hardware_interface)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(roberto_hardware_interface SHARED
  src/tendon.cpp
  src/roberto_hardware_interface.cpp
)

target_include_directories(roberto_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/roberto_hardware_interface>
  $<INSTALL_INTERFACE:include/roberto_hardware_interface>
)

ament_target_dependencies(roberto_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  trajectory_msgs
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface roberto_hardware_interface.xml)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(TARGETS roberto_hardware_interface
    DESTINATION lib/${PROJECT_NAME}
)

# install(FILES plugin_description.xml
#     DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
