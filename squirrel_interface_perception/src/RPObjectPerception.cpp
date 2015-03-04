#include "ros/ros.h"
#include "squirrel_knowledge_base/RPObjectPerception.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <stdlib.h> 
#include <algorithm> 
#include "squirrel_planning_knowledge_msgs/KnowledgeItem.h"
#include "mongodb_store/message_store.h"

namespace KCL_rosplan {

	/* constructor */
	RPObjectPerception::RPObjectPerception(ros::NodeHandle &nh, std::string &dp)
	 : message_store(nh), dataPath(dp) {

		add_knowledge_pub = nh.advertise<squirrel_planning_knowledge_msgs::KnowledgeItem>("/kcl_rosplan/add_knowledge", 10, true);
		remove_knowledge_pub = nh.advertise<squirrel_planning_knowledge_msgs::KnowledgeItem>("/kcl_rosplan/remove_knowledge", 10, true);
	}

	/* add new object to knowledge base and scene database */
	bool RPObjectPerception::addObjects(
		squirrel_planning_knowledge_msgs::AddObjectService::Request &req,
		squirrel_planning_knowledge_msgs::AddObjectService::Response &res) {
	}

	/* remove object from knowledge base and scene database */
	bool RPObjectPerception::removeObjects(
		squirrel_planning_knowledge_msgs::RemoveObjectService::Request &req,
		squirrel_planning_knowledge_msgs::RemoveObjectService::Response &res) {
	}

	/* update object in scene database and recalculate for knowledge base */
	bool RPObjectPerception::updateObjects(
		squirrel_planning_knowledge_msgs::UpdateObjectService::Request &req,
		squirrel_planning_knowledge_msgs::UpdateObjectService::Response &res) {
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		// setup ros
		ros::init(argc, argv, "rosplan_object_perception");
		ros::NodeHandle nh("~");

		// default config
		std::string dataPath = "common/";
		nh.param("data_path", dataPath, dataPath);

		// init services
		KCL_rosplan::RPObjectPerception rms(nh, dataPath);

		ROS_INFO("KCL: (RPObjectPerception) Ready to receive");
		ros::spin();
		return 0;
	}
