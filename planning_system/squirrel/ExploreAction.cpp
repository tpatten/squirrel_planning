#include "ros/ros.h"

#include "planning_dispatch_msgs/ActionDispatch.h"
#include "planning_dispatch_msgs/ActionFeedback.h"

#include <iostream>
#include <string>
#include <stdio.h>
#include <string.h>

/**
 * ROS node stub for SQUIRREL Summer School: EXPLORE
 * (this code is just a suggestion)
 */
namespace SQUIRREL_summerschool_explore {

	/* true is the action has been dispatched */
	bool actionActive;

	/* the ID of the action currently being executed */
	int actionID;

	/* true if the action has completed */
	bool actionCompleted;

	/* true if there has been a request to cancel the action */
	bool actionCancelled;

	/* the remaining duration of the current action */
	float exploreDuration;

	ros::Publisher feedbackPub;

	/*-----------------*/
	/* ROS subscribers */
	/*-----------------*/

	/**
	 * listens to the "/kcl_rosplan/action_dispatch" topic.
	 */
	void dispatchCallback(const planning_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		
		if(0 == msg->name.compare("explore")) {
			// prep new explore action
			actionID = msg->action_id;
			actionActive = true;
			actionCompleted = false;
			actionCancelled = false;
			exploreDuration = msg->duration;

			planning_dispatch_msgs::ActionFeedback msg;
			msg.action_id = actionID;
			msg.status = "action enabled";
			feedbackPub.publish(msg);
		}

		if(0 == msg->name.compare("cancel_action") && msg->action_id == actionID) {
			// cancel the current action
			actionCancelled = true;
		}
	}

	/*-----------*/
	/* main loop */
	/*-----------*/

	/**
	 * While the action is active, explore the room!
	 */
	void startAction() {

		// set up ROS
		ros::NodeHandle nh("~");
		ros::Subscriber dispatchSub = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, dispatchCallback);
		feedbackPub = nh.advertise<planning_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 1000, true);
		ros::Rate loop_rate(10);

		// main loop
		std::clock_t timer = std::clock();
		while(ros::ok()) {

			ros::spinOnce();

			if(actionCancelled) {

				// TODO cancel the current movement

				actionCompleted = true;
				actionActive = false;
			}

			if(actionActive) {
				

				// TODO perform an explore action

				// explored for long enough
				exploreDuration = exploreDuration - (std::clock() - timer)/CLOCKS_PER_SEC;
				timer = std::clock();
				if(exploreDuration <= 0)
					actionCompleted = true;
			}

			// send feedback msg
			if(actionCompleted) {
				planning_dispatch_msgs::ActionFeedback msg;
				msg.action_id = actionID;
				msg.status = "action achieved";
				feedbackPub.publish(msg);
			}

			loop_rate.sleep();
		}		
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc,argv,"squirrel_action_explore");
		SQUIRREL_summerschool_explore::startAction();
		return 0;
	}
