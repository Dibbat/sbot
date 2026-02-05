cmake_minimum_required(VERSION 3.5)
project(imav_2017)

set(CMAKE_CXX_STANDARD 14)

# Find ament
find_package(ament_cmake REQUIRED)

# Optionally build Gazebo plugins (disabled by default)
option(BUILD_GAZEBO_PLUGINS "Build Gazebo plugins" OFF)

if(BUILD_GAZEBO_PLUGINS)
  find_package(gazebo REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)

  message(STATUS "Building Gazebo plugins")
  
  include_directories(
    plugins/include
    ${GAZEBO_INCLUDE_DIRS}
  )
  
  add_library(quadrotor_simple_controller SHARED
    plugins/src/quadrotor_simple_controller.cpp
  )
  
  target_link_libraries(quadrotor_simple_controller
    ${GAZEBO_LIBRARIES}
    rclcpp::rclcpp
    geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
    sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
    nav_msgs::nav_msgs__rosidl_typesupport_cpp
  )
  
  install(TARGETS quadrotor_simple_controller
    LIBRARY DESTINATION lib
  )
else()
  message(STATUS "Gazebo plugin build disabled")
endif()

# Install directories
install(DIRECTORY
  launch
  urdf
  worlds
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

# Install scripts
install(PROGRAMS
  scripts/ps2cmd_vel.py
  DESTINATION lib/${PROJECT_NAME}
)

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/quadrotor_controller.py
  ${CMAKE_CURRENT_BINARY_DIR}/quadrotor_controller.py
  @ONLY
  NEWLINE_STYLE UNIX
)

install(PROGRAMS
  ${CMAKE_CURRENT_BINARY_DIR}/quadrotor_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME quadrotor_controller
)

ament_package()
