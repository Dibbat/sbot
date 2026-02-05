cmake_minimum_required(VERSION 3.5)

# Only build plugin if gazebo and rclcpp are available
if(NOT gazebo_FOUND OR NOT rclcpp_FOUND)
  message(WARNING "Skipping plugin: Gazebo or rclcpp not found")
  return()
endif()

# Find ROS2 packages
find_package(gazebo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros QUIET)
find_package(geometry_msgs QUIET)
find_package(sensor_msgs QUIET)
find_package(nav_msgs QUIET)

# Get source files
file(GLOB SOURCES src/*.cpp)

if(NOT SOURCES)
  message(WARNING "No C++ sources found in plugins/src/")
  return()
endif()

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${GAZEBO_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

# Plugin library
set(PLUGIN_NAME quadrotor_simple_controller)
add_library(${PLUGIN_NAME} SHARED ${SOURCES})

# Link libraries
target_link_libraries(${PLUGIN_NAME}
  rclcpp::rclcpp
  ${GAZEBO_LIBRARIES}
)

if(geometry_msgs_FOUND)
  target_link_libraries(${PLUGIN_NAME} ${geometry_msgs_LIBRARIES})
endif()

if(sensor_msgs_FOUND)
  target_link_libraries(${PLUGIN_NAME} ${sensor_msgs_LIBRARIES})
endif()

if(nav_msgs_FOUND)
  target_link_libraries(${PLUGIN_NAME} ${nav_msgs_LIBRARIES})
endif()

# Target includes
target_include_directories(${PLUGIN_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install
install(TARGETS ${PLUGIN_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


















