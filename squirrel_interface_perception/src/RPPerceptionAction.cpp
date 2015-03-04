#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>
#include <actionlib/client/simple_action_client.h>
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "squirrel_object_perception_msgs/LookForObjectsAction.h"
#include "squirrel_interface_perception/RPPerceptionAction.h"
#include "mongodb_store/message_store.h"
#include "geometry_msgs/PoseStamped.h"

/* The implementation of RPMoveBase.h */
namespace KCL_rosplan {

	/* constructor */
	RPPerceptionAction::RPPerceptionAction(ros::NodeHandle &nh, std::string &actionserver, bool simulate)
	 : message_store(nh), action_client(actionserver, true), simulate_(simulate) {
		
		// create the action clients
		ROS_INFO("KCL: (PerceptionAction) waiting for action server to start on %s", actionserver.c_str());
		action_client.waitForServer();
		
		// create the action feedback publisher
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
	}

	/* action dispatch callback; parameters (?v - robot ?wp - waypoint) */
	void RPPerceptionAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

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

		if(!simulate_) {
			// dispatch Perception action
			squirrel_object_perception_msgs::LookForObjectsGoal goal;
			goal.look_for_object = squirrel_object_perception_msgs::LookForObjectsGoal::EXPLORE;
			action_client.sendGoal(goal);
		}

		// publish feedback (enabled)
		 rosplan_dispatch_msgs::ActionFeedback fb;
		fb.action_id = msg->action_id;
		fb.status = "action enabled";
		action_feedback_pub.publish(fb);

		if(!simulate_) {
			bool finished_before_timeout = action_client.waitForResult(ros::Duration(msg->duration));
			if (finished_before_timeout) {

				actionlib::SimpleClientGoalState state = action_client.getState();
				ROS_INFO("KCL: (PerceptionAction) action finished: %s", state.toString().c_str());
			
				// publish feedback (achieved)
				 rosplan_dispatch_msgs::ActionFeedback fb;
				fb.action_id = msg->action_id;
				fb.status = "action achieved";
				action_feedback_pub.publish(fb);

			} else {

				ROS_INFO("KCL: (PerceptionAction) action timed out");

				// publish feedback (failed)
				 rosplan_dispatch_msgs::ActionFeedback fb;
				fb.action_id = msg->action_id;
				fb.status = "action failed";
				action_feedback_pub.publish(fb);
			}
		} else {
			// publish feedback (achieved)
			ROS_INFO("KCL: (PerceptionAction) simulated action finished");
			rosplan_dispatch_msgs::ActionFeedback fb;
			fb.action_id = msg->action_id;
			fb.status = "action achieved";
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

		bool simulate;
		nh.param("simulate", simulate, false);

		std::string actionserver;
		nh.param("action_server", actionserver, std::string("/squirrel_object_perception"));

		// create PDDL action subscriber
		KCL_rosplan::RPPerceptionAction rppa(nh, actionserver, simulate);
	
		// listen for action dispatch
		ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPPerceptionAction::dispatchCallback, &rppa);
		ROS_INFO("KCL: (PerceptionAction) Ready to receive");

		ros::spin();
		return 0;
	}
