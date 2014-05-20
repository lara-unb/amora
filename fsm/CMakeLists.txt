cmake_minimum_required(VERSION 2.8.3)
project(fsm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  genmsg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include/
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/local/include/sphinxbase
  /usr/local/include/pocketsphinx
  /usr/local/include/EST
  /usr/local/include/festival
  /usr/local/include
  /usr/local/lib
)


## Declare a cpp library
add_library(continuous
  src/continuous.cpp
)

add_library(filters
  src/filters.cpp
)

add_library(class_fsm
  src/class_fsm.cpp
  src/filters.cpp
)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES} continuous pocketsphinx sphinxbase sphinxad)

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES} Festival estools eststring estbase asound ncurses)

add_executable(fsm src/fsm.cpp)
target_link_libraries(fsm ${catkin_LIBRARIES} class_fsm)

add_dependencies(listener beginner_tutorials_generate_message_cpp)
