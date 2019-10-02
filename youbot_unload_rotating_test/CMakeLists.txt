cmake_minimum_required(VERSION 2.8.3)
project(youbot_unload_object_table)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation dynamic_reconfigure actionlib actionlib_msgs geometry_msgs std_msgs aricc_utils aricc_pid aricc_vision_msgs eigen_conversions)
find_package(Boost REQUIRED COMPONENTS system)

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(
  cfg/Controller.cfg
  cfg/Server.cfg
)

add_action_files(
  DIRECTORY action
  FILES Unload.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_server src/main.cpp)
target_link_libraries( ${PROJECT_NAME}_server 
  ${catkin_LIBRARIES} 
  ${Eigen_LIBRARIES})

add_executable( ${PROJECT_NAME}_controller src/controller.cpp )
target_link_libraries( ${PROJECT_NAME}_controller 
  ${catkin_LIBRARIES} 
  ${Eigen_LIBRARIES})
