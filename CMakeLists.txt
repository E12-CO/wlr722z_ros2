cmake_minimum_required(VERSION 3.8)
project(wlr722z_ros2)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra -Wpedantic -g")

find_package(ament_cmake REQUIRED)
# ROS2 dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

#PCL
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

set(wlr_dependencies
	rclcpp
	rclcpp_components
	sensor_msgs
	pcl_msgs
	pcl_conversions
)

include_directories(
  ${PCL_INCLUDE_DIRS}
)
link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_executable(wlr_node
	src/wlr_ros2.cpp
)

target_link_libraries(
	wlr_node
	${PCL_LIBRARIES} 
)

ament_target_dependencies(
	wlr_node
	${wlr_dependencies}
)

install(
	TARGETS wlr_node
	DESTINATION lib/${PROJECT_NAME}
)

install(
	DIRECTORY params launch
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
