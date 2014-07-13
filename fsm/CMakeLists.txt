cmake_minimum_required(VERSION 2.8.3)
project(fsm)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

find_package(ALSA REQUIRED)
find_package(Curses REQUIRED)
find_package(Festival REQUIRED)
find_package(Speechtools REQUIRED)
find_package(SphinxBase 0.8 REQUIRED)
find_package(PocketSphinx 0.8 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include/
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(include)
include_directories(
  ${ALSA_INCLUDE_DIR}
  ${CURSES_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${FESTIVAL_INCLUDE_DIR}
  ${SPEECHTOOLS_INCLUDE_DIR}
  ${SPHINXBASE_INCLUDE_DIRS}
  ${POCKETSPHINX_INCLUDE_DIRS}
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
target_link_libraries(listener ${catkin_LIBRARIES} continuous ${POCKETSPHINX_LIBRARIES} ${SPHINXBASE_LIBRARIES} ${SPHINXAD_LIBRARIES})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES} ${FESTIVAL_LIBRARIES} ${SPEECHTOOLS_LIBRARIES} ${ALSA_LIBRARY} ${CURSES_LIBRARY})

add_executable(fsm src/fsm.cpp)
target_link_libraries(fsm ${catkin_LIBRARIES} class_fsm)

add_dependencies(listener beginner_tutorials_generate_message_cpp)
