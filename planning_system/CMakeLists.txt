cmake_minimum_required(VERSION 2.8.3)
project(planning_system)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  planning_dispatch_msgs
  planning_knowledge_msgs
  perception_srv_definitions
)

## needed for perception interface, should eventually go
find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  LIBRARIES planning_system
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs planning_dispatch_msgs planning_knowledge_msgs perception_srv_definitions perception_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## needed for perception interface, should eventually go
include_directories(include
  ${PCL_INCLUDE_DIRS}
)
  
## visualisation
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})

## val sources
set(VAL_SOURCES
	src/VALfiles/DebugWriteController.cpp
	src/VALfiles/FastEnvironment.cpp
	src/VALfiles/FuncAnalysis.cpp
	src/VALfiles/SASActions.cpp
	src/VALfiles/SimpleEval.cpp
	src/VALfiles/TIM.cpp
	src/VALfiles/TimSupport.cpp
	src/VALfiles/ToFunction.cpp
	src/VALfiles/TypedAnalyser.cpp
	src/VALfiles/ValidatorAPI.cpp
	src/VALfiles/instantiation.cpp
	src/VALfiles/pddl+.cpp
	src/VALfiles/ptree.cpp
	src/VALfiles/typecheck.cpp
	src/VALfiles/Validator.cpp
	src/VALfiles/Action.cpp
	src/VALfiles/Proposition.cpp
	src/VALfiles/FuncExp.cpp
	src/VALfiles/RepairAdvice.cpp
	src/VALfiles/State.cpp
	src/VALfiles/Plan.cpp
	src/VALfiles/Ownership.cpp
	src/VALfiles/Environment.cpp
	src/VALfiles/Events.cpp
	src/VALfiles/Polynomial.cpp
	src/VALfiles/TrajectoryConstraints.cpp
	src/VALfiles/RobustAnalyse.cpp
	src/VALfiles/PrettyPrinter.cpp
	src/VALfiles/random.cpp
	src/VALfiles/LaTeXSupport.cpp
	src/VALfiles/Utils.cpp)

## squirrel interface sources
set(SQUIRREL_SOURCES
	squirrel/PerceptionInterface.cpp
	squirrel/ExploreAction.cpp)

## Declare cpp executables
add_executable(planner src/PlanningLoop.cpp	${VAL_SOURCES})

# some squirrel executables
add_executable(squirrelVis squirrel/PlanningVis.cpp)
add_executable(perceptionInterface ${SQUIRREL_SOURCES})

## Specify libraries against which to link a library or executable target
target_link_libraries(planner
  ${catkin_LIBRARIES}
)

target_link_libraries(squirrelVis
  ${catkin_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${OPENGL_LIBRARIES}
)

target_link_libraries(perceptionInterface
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(exploreAction
  ${catkin_LIBRARIES}
)
