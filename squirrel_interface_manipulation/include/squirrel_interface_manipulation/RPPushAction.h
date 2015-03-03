#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <actionlib/client/simple_action_client.h>
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "squirrel_manipulation_msgs/PushAction.h"
#include "mongodb_store/message_store.h"
#include "geometry_msgs/PoseStamped.h"

#ifndef KCL_pushaction
#define KCL_pushaction

/**
 * This file defines the RPPushAction class.
 * RPPushAction is used by SQUIRREL to push objects about with the robotino.
 * PDDL "push_object" actions become "squirrel_manipulation_msgs::Push" actions.
 * Waypoint goals are fetched by name from the SceneDB (implemented by mongoDB).
 */
namespace KCL_rosplan {

	class RPPushAction
	{

	private:

		mongodb_store::MessageStoreProxy message_store;
		actionlib::SimpleActionClient<squirrel_manipulation_msgs::PushAction> action_client;
		ros::Publisher action_feedback_pub;

	public:

		/* constructor */
		RPPushAction(ros::NodeHandle &nh, std::string &actionserver);

		/* listen to and process action_dispatch topic */
		void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif
