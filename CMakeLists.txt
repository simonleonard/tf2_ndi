cmake_minimum_required(VERSION 3.8)
project(tf2_ndi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ndi_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable( tf2_broadcaster src/tf2_broadcaster_node.cpp )
add_executable( point_publisher src/point_publisher.cpp )
ament_target_dependencies( tf2_broadcaster tf2_ros ndi_msgs rclcpp tf2_geometry_msgs )
ament_target_dependencies( point_publisher tf2_ros ndi_msgs rclcpp tf2_geometry_msgs )

install( TARGETS tf2_broadcaster point_publisher DESTINATION lib/${PROJECT_NAME} )

ament_package()
