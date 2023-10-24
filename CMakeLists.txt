cmake_minimum_required(VERSION 3.5)
project(ros2_viz_tools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED) 
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
)

set(DEPENDENCIES
  "geometry_msgs"
  "tf2"
  "rclcpp"
  "tf2_ros"
  "visualization_msgs"
  "std_msgs"
  "tf2_geometry_msgs"
)

# library
add_library(${PROJECT_NAME}_lib src/ros2_viz_tools.cpp src/color.cpp)
ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})

add_executable(demo_node src/demo_node.cpp)
target_link_libraries(demo_node ${PROJECT_NAME}_lib)
ament_target_dependencies(demo_node ${DEPENDENCIES})

install(TARGETS demo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch images
  DESTINATION share/${PROJECT_NAME}
)


ament_export_include_directories(include)
ament_package()