cmake_minimum_required(VERSION 2.8.3)
project(obstacle_tracker)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
					roscpp 
					car_msgs
					geometry_msgs
					pcl_msgs
					pcl_conversions
					visualization_msgs
					pcl_ros 
					tf
					dynamic_reconfigure
					#message_generation
					#${msg_dependencies}
)

generate_dynamic_reconfigure_options(
  cfg/Tracker.cfg
  #...
)

## Define catkin package
catkin_package()

## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${pcl_msgs_INCLUDE_DIRS})
include_directories(include ${pcl_ros_INCLUDE_DIRS})
include_directories(include ${/include}) 


# Declare executables
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

# Add dependencies
add_dependencies(obstacle_tracker_node ${PROJECT_NAME}_gencfg)

# Link libraries
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node ${pcl_msgs_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node ${pcl_ros_LIBRARIES})

