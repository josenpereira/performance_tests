cmake_minimum_required(VERSION 2.8.3)
project(performance_tests)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
   FILES
   SuperAwesome.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES performance_tests
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(cpp_pub src/cpp_pub.cpp)
target_link_libraries(cpp_pub ${catkin_LIBRARIES})
add_dependencies(cpp_pub performance_tests_generate_messages_cpp)

add_executable(cpp_sub src/cpp_sub.cpp)
target_link_libraries(cpp_sub ${catkin_LIBRARIES})
add_dependencies(cpp_sub performance_tests_generate_messages_cpp)