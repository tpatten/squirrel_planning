#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "squirrel_planning_knowledge_msgs/KnowledgeItem.h"
#include "squirrel_planning_knowledge_msgs/AddObjectService.h"
#include "squirrel_planning_knowledge_msgs/RemoveObjectService.h"
#include "squirrel_planning_knowledge_msgs/UpdateObjectService.h"
#include "mongodb_store/message_store.h"

#ifndef KCL_object_perception
#define KCL_object_perception

/**
 * This file defines the RPObjectPerception class.
 * RPObjectPerception is used to convert between real and
 * symbolic representations of toy objects for SQUIRREL.
 * Symbols are stored in the Knoweldge Base.
 * real data are stored in the SceneDB (implemented by mongoDB).
 */
namespace KCL_rosplan {

	class RPObjectPerception
	{

	private:
		
		std::string dataPath;

		// Scene database
		mongodb_store::MessageStoreProxy message_store;

		// Knowledge base
		ros::Publisher add_knowledge_pub;
		ros::Publisher remove_knowledge_pub;

	public:

		/* constructor */
		RPObjectPerception(ros::NodeHandle &nh, std::string &dp);

		/* services */
		bool addObjects(squirrel_planning_knowledge_msgs::AddObjectService::Request &req, squirrel_planning_knowledge_msgs::AddObjectService::Response &res);
		bool removeObjects(squirrel_planning_knowledge_msgs::RemoveObjectService::Request &req, squirrel_planning_knowledge_msgs::RemoveObjectService::Response &res);
		bool updateObjects(squirrel_planning_knowledge_msgs::UpdateObjectService::Request &req, squirrel_planning_knowledge_msgs::UpdateObjectService::Response &res);

	};
}
#endif
