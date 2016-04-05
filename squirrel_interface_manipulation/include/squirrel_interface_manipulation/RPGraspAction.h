#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>
#include <actionlib/client/simple_action_client.h>
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "mongodb_store/message_store.h"
#include "geometry_msgs/PoseStamped.h"
#include "squirrel_object_perception_msgs/SceneObject.h"
#include "squirrel_manipulation_msgs/BlindGraspAction.h"

#ifndef KCL_graspaction
#define KCL_graspaction

/**
 * This file defines the RPGraspAction class.
 * RPGraspAction is used by SQUIRREL to grasp and lift objects with the robotino.
 * PDDL "grasp_object" actions become one of the grasp actions.
 * (eg. BlindGrasp)
 */
namespace KCL_rosplan {

	class RPGraspAction
	{

	private:

		mongodb_store::MessageStoreProxy message_store;
		actionlib::SimpleActionClient<squirrel_manipulation_msgs::BlindGraspAction> blind_grasp_action_client;
		ros::Publisher action_feedback_pub;

		/* execute pushing actions */
		void dispatchBlindGraspAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

	public:

		/* constructor */
		RPGraspAction(ros::NodeHandle &nh, std::string &blindGraspActionServer);

		/* listen to and process action_dispatch topic */
		void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif
