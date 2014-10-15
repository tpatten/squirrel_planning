cmake_minimum_required(VERSION 2.8.3)
project(squirrel_knowledge_base)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  squirrel_planning_knowledge_msgs
)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  LIBRARIES squirrel_knowledge_base
  CATKIN_DEPENDS roscpp rospy std_msgs squirrel_planning_knowledge_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## val sources
set(KNOWLEDGE_SOURCES
	src/KnowledgeBase.h
	src/KnowledgeBase.cpp
	src/PlanningFilter.cpp)

## Declare cpp executables
add_executable(knowledgeBase ${KNOWLEDGE_SOURCES})

## Specify libraries against which to link a library or executable target
target_link_libraries(knowledgeBase
  ${catkin_LIBRARIES}
)
