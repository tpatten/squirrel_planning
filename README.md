squirrel_planning
=================

Technical Maintainer: m312z (Michael Cashmore, King's College London)

Repository for planning related SQUIRREL packages.

Installation
============

You need the ROSPlan repository
(follow the installation instructions here: https://github.com/KCL-Planning/ROSPlan/wiki/a.-Installation )
```
git clone https://github.com/clearpathrobotics/occupancy_grid_utils
git clone -b squirrel https://github.com/kcl-planning/ROSPlan.git
```
Additionally you will require Flex and MongoDB.
```
sudo apt-get install flex
sudo apt-get install mongodb
sudo apt-get install ros-indigo-mongodb-store
```
Finally check the requirements in squirrel_recommender.

Running everything
==================

First, disable the MongoDB service, if it is started automatically by your system:
```
sudo service mongodb stop
```
To launch the planning system, use:
```
roslaunch squirrel_planning_launch squirrel_planning_system.launch
```
To begin the mission, use:
```
rosrun squirrel_planning_execution tidyroom
```

This branch is for 3rd year review in Freiburg and is initialised from a working planning system on 14.03.2017
