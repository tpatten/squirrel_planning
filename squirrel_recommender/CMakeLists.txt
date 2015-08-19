cmake_minimum_required(VERSION 2.8.3)
project(squirrel_recommender)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  squirrel_prediction_msgs
)

find_package(Boost REQUIRED COMPONENTS
  filesystem
)

catkin_package(
  CATKIN_DEPENDS
   roscpp
   rospy
   std_msgs
   squirrel_prediction_msgs
)

###########
## Build ##
###########

## include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)


add_executable(recommender src/RecommenderNode.cpp)
target_link_libraries(recommender ${catkin_LIBRARIES})






