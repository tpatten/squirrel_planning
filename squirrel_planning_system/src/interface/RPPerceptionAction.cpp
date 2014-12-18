#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>
#include <actionlib/client/simple_action_client.h>
#include "squirrel_planning_dispatch_msgs/ActionDispatch.h"
#include "squirrel_planning_dispatch_msgs/ActionFeedback.h"
#include "squirrel_object_perception_msgs/LookForObjectsAction.h"
#include "squirrel_planning_system/RPPerceptionAction.h"
#include "mongodb_store/message_store.h"
#include "geometry_msgs/PoseStamped.h"

/* The implementation of RPMoveBase.h */
namespace KCL_rosplan {

	/* constructor */
	RPPerceptionAction::RPPerceptionAction(ros::NodeHandle &nh, std::string &actionserver)
	 : message_store(nh), action_client(actionserver, true) {
		
		// create the action clients
		ROS_INFO("KCL: (PerceptionAction) waiting for action server to start on %s", actionserver.c_str());
		action_client.waitForServer();
		
		// create the action feedback publisher
		action_feedback_pub = nh.advertise<squirrel_planning_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
	}

	/* action dispatch callback; parameters (?v - robot ?wp - waypoint) */
	void RPPerceptionAction::dispatchCallback(const squirrel_planning_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// ignore non-goto-waypoint actions
		if(0!=msg->name.compare("explore_waypoint")) return;

		ROS_INFO("KCL: (PerceptionAction) action recieved");

		// get waypoint ID from action dispatch
		std::string wpID;
		bool found = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			if(0==msg->parameters[i].key.compare("wp")) {
				wpID = msg->parameters[i].value;
				found = true;
			}
		}
		if(!found) {
			ROS_INFO("KCL: (PerceptionAction) aborting action dispatch; malformed parameters");
			return;
		}

		// dispatch Perception action
		squirrel_object_perception_msgs::LookForObjectsGoal goal;
		goal.look_for_object = squirrel_object_perception_msgs::LookForObjectsGoal::EXPLORE;
		action_client.sendGoal(goal);

		// publish feedback (enabled)
		 squirrel_planning_dispatch_msgs::ActionFeedback fb;
		fb.action_id = msg->action_id;
		fb.status = "action enabled";
		action_feedback_pub.publish(fb);

		bool finished_before_timeout = action_client.waitForResult(ros::Duration(msg->duration));
		if (finished_before_timeout) {

			actionlib::SimpleClientGoalState state = action_client.getState();
			ROS_INFO("KCL: (PerceptionAction) action finished: %s", state.toString().c_str());
			
			// publish feedback (achieved)
			 squirrel_planning_dispatch_msgs::ActionFeedback fb;
			fb.action_id = msg->action_id;
			fb.status = "action achieved";
			action_feedback_pub.publish(fb);

		} else {

			ROS_INFO("KCL: (PerceptionAction) action timed out");

			// publish feedback (failed)
			 squirrel_planning_dispatch_msgs::ActionFeedback fb;
			fb.action_id = msg->action_id;
			fb.status = "action failed";
			action_feedback_pub.publish(fb);
		}
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_perception");
		ros::NodeHandle nh;

		std::string actionserver;
		nh.param("action_server", actionserver, std::string("/squirrel_object_perception"));

		// create PDDL action subscriber
		KCL_rosplan::RPPerceptionAction rppa(nh, actionserver);
	
		// listen for action dispatch
		ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPPerceptionAction::dispatchCallback, &rppa);
		ROS_INFO("KCL: (PerceptionAction) Ready to receive");

		ros::spin();
		return 0;
	}
