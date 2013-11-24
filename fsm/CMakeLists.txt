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
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${pocketsphinx_INCLUDE_DIRS}
  /usr/local/include/sphinxbase
  /usr/local/include/pocketsphinx
  /usr/local/lib
  /home/gastd/Downloads/festival/src/include
  /home/gastd/Downloads/speech_tools/include
  /home/gastd/Downloads/speech_tools/lib
  /home/gastd/Downloads/festival/src/lib/
)

## Declare a cpp library
add_library(continuous
#   src/${PROJECT_NAME}/fsm.cpp
  src/continuous.cpp
)
add_library(filters
  src/filters.cpp
)
add_library(fsm
  src/fsm.cpp
)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES} ${pocketsphinx_LIBRARIES} filters continuous pocketsphinx sphinxbase sphinxad /home/gastd/Downloads/festival/src/lib/libFestival.a /home/gastd/Downloads/speech_tools/lib/libestools.a /home/gastd/Downloads/speech_tools/lib/libeststring.a /home/gastd/Downloads/speech_tools/lib/libestbase.a asound ncurses)

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES} ${pocketsphinx_LIBRARIES} fsm /home/gastd/Downloads/festival/src/lib/libFestival.a /home/gastd/Downloads/speech_tools/lib/libestools.a /home/gastd/Downloads/speech_tools/lib/libeststring.a /home/gastd/Downloads/speech_tools/lib/libestbase.a asound ncurses)

add_dependencies(listener beginner_tutorials_generate_message_cpp)
