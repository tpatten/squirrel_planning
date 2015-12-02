cmake_minimum_required(VERSION 2.8.3)
project(squirrel_recommender)

# set(Python_ADDITIONAL_VERSIONS 3.4)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  squirrel_prediction_msgs
  rosplan_knowledge_msgs
)
find_package(cmake_modules REQUIRED)

find_package(Boost REQUIRED COMPONENTS
  filesystem
)
find_package(PythonLibs 3.4 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
   roscpp
   rospy
   std_msgs
   squirrel_prediction_msgs
   rosplan_knowledge_msgs
)

###########
## Build ##
###########

## include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include/python3.4m
  include
)

#include_directories(${PYTHON_INCLUDE_DIRS})

## python definitions
link_directories(/usr/lib/python3.4/config-3.4m-x86_64-linux-gnu /usr/lib)

add_executable(recommender src/RecommenderNode.cpp)
target_link_libraries(recommender ${catkin_LIBRARIES} python3.4m)

add_executable(rprecommender src/RPRecommender.cpp)
add_dependencies(rprecommender ${catkin_EXPORTED_TARGETS})
target_link_libraries(rprecommender ${catkin_LIBRARIES})





