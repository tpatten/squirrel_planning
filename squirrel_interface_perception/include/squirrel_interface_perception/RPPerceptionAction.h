#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <actionlib/client/simple_action_client.h>
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "squirrel_planning_knowledge_msgs/AddObjectService.h"
#include "move_base_msgs/MoveBaseAction.h"
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

		bool simulate_;
		mongodb_store::MessageStoreProxy message_store;
		actionlib::SimpleActionClient<squirrel_object_perception_msgs::LookForObjectsAction> action_client;
		actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> movebase_client;
		ros::ServiceClient add_object_client;
		ros::Publisher action_feedback_pub;
		void publishFeedback(int action_id, std::string feedback);

	public:

		/* constructor */
		RPPerceptionAction(ros::NodeHandle &nh, std::string &actionserver, bool simulate);

		/* listen to and process action_dispatch topic */
		void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif
