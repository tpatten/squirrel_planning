#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
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
		ros::ServiceClient knowledgeInterface;
		
		// action topics
		ros::Publisher action_feedback_pub;
		ros::Subscriber pointing_pose_sub;

		// Callback of the pointing pose topic.
		void receivePointLocation(const geometry_msgs::PoseStamped::ConstPtr& ptr);

		geometry_msgs::PoseStamped received_point_;
		bool has_received_point_;
	public:

		/* constructor */
		RPPointingServer(ros::NodeHandle &nh);

		void dispatchCallback(const plan_ispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif
