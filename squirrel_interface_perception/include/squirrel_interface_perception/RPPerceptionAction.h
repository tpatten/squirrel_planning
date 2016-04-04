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

		mongodb_store::MessageStoreProxy message_store;

		actionlib::SimpleActionClient<squirrel_object_perception_msgs::LookForObjectsAction> examine_action_client;
		ros::ServiceClient find_dynamic_objects_client;
		ros::ServiceClient add_object_client;
		ros::ServiceClient update_knowledge_client;

		ros::Publisher action_feedback_pub;

		std::map<std::string,std::string> db_name_map;

		void publishFeedback(int action_id, std::string feedback);

	public:

		/* constructor */
		RPPerceptionAction(ros::NodeHandle &nh, std::string &actionserver);

		/* listen to and process action_dispatch topic */
		void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif
