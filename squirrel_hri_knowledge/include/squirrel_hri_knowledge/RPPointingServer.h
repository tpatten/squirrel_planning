#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "squirrel_planning_knowledge_msgs/KnowledgeItem.h"
#include "squirrel_planning_knowledge_msgs/AddObjectService.h"
#include "squirrel_planning_knowledge_msgs/RemoveObjectService.h"
#include "squirrel_planning_knowledge_msgs/UpdateObjectService.h"
#include "squirrel_planning_dispatch_msgs/ActionFeedback.h"
#include "squirrel_planning_dispatch_msgs/ActionDispatch.h"
#include "mongodb_store/message_store.h"

#include <geometry_msgs/PoseStamped.h>

#ifndef KCL_pointing_server
#define KCL_pointing_server

/**
 * This file defines the RPObjectPerception class.
 * RPObjectPerception is used to convert between real and
 * symbolic representations of toy objects for SQUIRREL.
 * Symbols are stored in the Knoweldge Base.
 * real data are stored in the SceneDB (implemented by mongoDB).
 */
namespace KCL_rosplan {

	class RPPointingServer
	{

	private:
		
		// Scene database
		mongodb_store::MessageStoreProxy message_store;

		// Knowledge base
		ros::Publisher add_knowledge_pub;
		ros::Publisher remove_knowledge_pub;
		
		ros::Publisher action_feedback_pub;
		
		ros::Subscriber pointing_pose_sub;

		// Callback of the pointing pose topic.
		void receivePointLocation(const geometry_msgs::PoseStamped::ConstPtr& ptr);
		
		geometry_msgs::PoseStamped received_point_;
		bool has_received_point_;
	public:

		/* constructor */
		RPPointingServer(ros::NodeHandle &nh);

		void dispatchCallback(const squirrel_planning_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif
