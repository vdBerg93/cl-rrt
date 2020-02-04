cmake_minimum_required(VERSION 2.8.12)
project(car_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  vision_msgs
  std_msgs
  rospy
)

add_message_files(
  FILES
  Obstacle2D.msg
  State.msg
  Trajectory.msg
  Reference.msg
  MotionRequest.msg
  MotionResponse.msg
  MotionPlan.msg
  LaneDet.msg
  ControlErrors.msg
)

add_service_files(
  FILES
  getobstacles.srv
  resetplanner.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  vision_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
