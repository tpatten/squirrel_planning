cmake_minimum_required(VERSION 2.8.3)
project(squirrel_knowledge_base)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  squirrel_planning_knowledge_msgs
  nav_msgs
  mongodb_store
  geometry_msgs
  diagnostic_msgs
  visualization_msgs
)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  LIBRARIES squirrel_knowledge_base
  CATKIN_DEPENDS roscpp rospy std_msgs squirrel_planning_knowledge_msgs nav_msgs mongodb_store geometry_msgs diagnostic_msgs visualization_msgs
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

## knowledge sources
set(KNOWLEDGE_SOURCES
	src/KnowledgeBase.cpp
	src/PlanningFilter.cpp)

## knowledge sources
set(RPROADMAP_SOURCES
	src/rproadmap/RPRoadmapServer.cpp
	src/rproadmap/RPRoadmapVisualization.cpp)

## Declare cpp executables
add_executable(knowledgeBase ${KNOWLEDGE_SOURCES})
add_executable(rproadmapServer ${RPROADMAP_SOURCES})

## Specify libraries against which to link a library or executable target
target_link_libraries(knowledgeBase ${catkin_LIBRARIES})
target_link_libraries(rproadmapServer ${catkin_LIBRARIES})

##########
## Test ##
##########

## using ROSTest
find_package(rostest REQUIRED)
find_package(gtest)

add_rostest_gtest(testKnowledgeBase test/test_launch.launch test/utest.cpp)
target_link_libraries(testKnowledgeBase ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
