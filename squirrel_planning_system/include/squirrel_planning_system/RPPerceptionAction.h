#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <actionlib/client/simple_action_client.h>
#include "squirrel_planning_dispatch_msgs/ActionDispatch.h"
#include "squirrel_planning_dispatch_msgs/ActionFeedback.h"
#include "mongodb_store/message_store.h"

#ifndef KCL_perception
#define KCL_perception

/**
 * This file defines the RPPerceptionAction class.
 * RPPerceptionAction is used to connect ROSPlan to the object perception in SQUIRREL
 */
namespace KCL_rosplan {

	class RPPerceptionAction
	{

	private:

		mongodb_store::MessageStoreProxy message_store;
		// actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client;
		ros::Publisher action_feedback_pub;

	public:

		/* constructor */
		RPPerceptionAction(ros::NodeHandle &nh, std::string &actionserver);

		/* listen to and process action_dispatch topic */
		void dispatchCallback(const squirrel_planning_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif
