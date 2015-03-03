#include "ros/ros.h"
#include "squirrel_hri_knowledge/RPPointingServer.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <stdlib.h> 
#include <algorithm> 
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "mongodb_store/message_store.h"

namespace KCL_rosplan {

	/* constructor */
	RPPointingServer::RPPointingServer(ros::NodeHandle &nh)
	 : message_store(nh), has_received_point_(false) {

		add_knowledge_pub = nh.advertise<rosplan_knowledge_msgs::KnowledgeItem>("/kcl_rosplan/add_knowledge", 10, true);
		remove_knowledge_pub = nh.advertise<rosplan_knowledge_msgs::KnowledgeItem>("/kcl_rosplan/remove_knowledge", 10, true);
		
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
		
		pointing_pose_sub = nh.subscribe("/pointing_pose", 1, &RPPointingServer::receivePointLocation, this);
	}
	
	void RPPointingServer::receivePointLocation(const geometry_msgs::PoseStamped::ConstPtr& ptr) {
		ROS_INFO("Received point: (%f, %f, %f)", ptr->pose.position.x, ptr->pose.position.y, ptr->pose.position.z);
		received_point_ = *ptr;
		has_received_point_ = true;
	}
	
	/* action dispatch callback; parameters (?ob - object) */
	void RPPointingServer::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// ignore non-goto-waypoint actions
		if(0!=msg->name.compare("request_tidy")) return;

		ROS_INFO("KCL: (RPPointingServer) action recieved");

		// publish feedback (enabled)
		rosplan_dispatch_msgs::ActionFeedback fb;
		fb.action_id = msg->action_id;
		fb.status = "action enabled";
		action_feedback_pub.publish(fb);
		
		// get waypoint ID from action dispatch
		std::string obID;
		bool found = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			if(0==msg->parameters[i].key.compare("ob")) {
				obID = msg->parameters[i].value;
				found = true;
			}
		}
		if(!found) {
			ROS_INFO("KCL: (RPPointingServer) aborting action dispatch; malformed parameters");
			return;
		}

		// Wait for a point to be published.
		ros::Rate r(10);
		while (!has_received_point_ && ros::ok()) {
			ros::spinOnce();
			r.sleep();
		}
		has_received_point_ = false;
		
		// Store the found point in the database.
		std::stringstream ss;
		ss << "pointed_location_" << obID;
		std::string id(message_store.insertNamed(ss.str(), received_point_));
		
		// Store it in the knowledge base.
		rosplan_knowledge_msgs::KnowledgeItem addWP;
		addWP.knowledge_type = squirrel_planning_knowledge_msgs::KnowledgeItem::INSTANCE;
		addWP.instance_type = "waypoint";
		addWP.instance_name = ss.str();
		add_knowledge_pub.publish(addWP);
		
		rosplan_knowledge_msgs::KnowledgeItem addTL;
		addTL.knowledge_type = squirrel_planning_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE;
		addTL.attribute_name = "tidy_location";
		diagnostic_msgs::KeyValue object;
		object.key = "o";
		object.value = obID;
		addTL.values.push_back(object);
		diagnostic_msgs::KeyValue location;
		location.key = "wp";
		location.value = ss.str();
		addTL.values.push_back(location);
		add_knowledge_pub.publish(addTL);
		
		// publish feedback (achieved)
		fb.action_id = msg->action_id;
		fb.status = "action achieved";
		action_feedback_pub.publish(fb);
	}

} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

	ros::init(argc, argv, "rosplan_pointing_server");
	ros::NodeHandle nh;

	// create PDDL action subscriber
	KCL_rosplan::RPPointingServer rpps(nh);

	// listen for action dispatch
	ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPPointingServer::dispatchCallback, &rpps);
	ROS_INFO("KCL: (RPPointingServer) Ready to receive");

	ros::spin();
	return 0;
}
