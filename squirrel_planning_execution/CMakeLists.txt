cmake_minimum_required(VERSION 2.8.3)
project(squirrel_planning_execution)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  rosplan_knowledge_msgs
  rosplan_dispatch_msgs
  rosplan_planning_system
  nav_msgs
  mongodb_store
  geometry_msgs
  diagnostic_msgs
  visualization_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS
  filesystem
)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp rospy std_msgs actionlib rosplan_knowledge_msgs rosplan_planning_system nav_msgs mongodb_store geometry_msgs diagnostic_msgs visualization_msgs tf
  DEPENDS
)

###########
## Build ##
###########

## include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Pointing service
set(tidyroom_SOURCES
	src/TidyRoom.cpp)

## map sources
set(rpsquirrelroadmap_SOURCES
  src/RPSquirrelRoadmap.cpp
  src/RPSimpleMapVisualization.cpp)

## recurse sources
set(rpsquirrelRecursion_SOURCES
  src/RPSquirrelRecursion.cpp
  src/ContingentTacticalClassifyPDDLGenerator.cpp
  src/ContingentStrategicClassifyPDDLGenerator.cpp)

## Declare cpp executables
add_executable(tidyroom ${tidyroom_SOURCES})
add_executable(simpledemo src/TidyRoomGrasping.cpp)
add_executable(rpsquirrelRoadmap ${rpsquirrelroadmap_SOURCES})
add_executable(rpsquirrelRecursion ${rpsquirrelRecursion_SOURCES})

add_dependencies(tidyroom ${catkin_EXPORTED_TARGETS})
add_dependencies(simpledemo ${catkin_EXPORTED_TARGETS})
add_dependencies(rpsquirrelRoadmap ${catkin_EXPORTED_TARGETS})
add_dependencies(rpsquirrelRecursion ${catkin_EXPORTED_TARGETS})

target_link_libraries(tidyroom ${catkin_LIBRARIES})
target_link_libraries(simpledemo ${catkin_LIBRARIES})
target_link_libraries(rpsquirrelRoadmap ${catkin_LIBRARIES})
target_link_libraries(rpsquirrelRecursion ${catkin_LIBRARIES})

##########
## Test ##
##########
# please do not use add_rosttest_gtest (seems to be interfering with qtcreator and cmake)
# see test documentation: http://wiki.ros.org/gtest
