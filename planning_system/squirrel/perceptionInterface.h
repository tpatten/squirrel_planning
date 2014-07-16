#include "ros/ros.h"

#include "planning_dispatch_msgs/ActionDispatch.h"
#include "planning_dispatch_msgs/ActionFeedback.h"

#include <iostream>
#include <string>
#include <stdio.h>
#include <string.h>

#ifndef SQUIRREL_perceptionInterface
#define SQUIRREL_perceptionInterface

/**
 * ROS node for SQUIRREL Summer School.
 * Contains interface for calling various controllers depending on action dispatch.
 * A class like this could service all actions implemented by a particular group.
 * It should call the relevant controllers using actionlib.
 */
namespace SQUIRREL_summerschool_perception {

	class PerceptionInterface
	{
		private:

		// action_id -> cancelled
		std::map<int,bool> actionCancelled;

		// action execution methods
		void executeObserve(const planning_dispatch_msgs::ActionDispatch::ConstPtr& msg);
		void executeClassify(const planning_dispatch_msgs::ActionDispatch::ConstPtr& msg);

		public:

		// ROS publisher and subscriber
		void dispatchCallback(const planning_dispatch_msgs::ActionDispatch::ConstPtr& msg);
		ros::Publisher feedbackPub;
	};
} // close namespace

#endif
