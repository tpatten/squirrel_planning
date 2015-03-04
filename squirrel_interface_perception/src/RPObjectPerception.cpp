#include "ros/ros.h"
#include "std_msgs/String.h"
#include "squirrel_interface_perception/RPObjectPerception.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <stdlib.h> 
#include <algorithm> 
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "mongodb_store/message_store.h"

#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"

namespace KCL_rosplan {

	/* constructor */
	RPObjectPerception::RPObjectPerception(ros::NodeHandle &nh, std::string &dp)
	 : message_store(nh), dataPath(dp) {

		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		
		add_object_service = nh.advertiseService("/kcl_rosplan/add_object", &RPObjectPerception::addObjects, this);
		update_object_service = nh.advertiseService("/kcl_rosplan/update_object", &RPObjectPerception::updateObjects, this);
		remove_object_service = nh.advertiseService("/kcl_rosplan/remove_object", &RPObjectPerception::removeObjects, this);
	}

	/* add new object to knowledge base and scene database */
	bool RPObjectPerception::addObjects(
		squirrel_planning_knowledge_msgs::AddObjectService::Request &req,
		squirrel_planning_knowledge_msgs::AddObjectService::Response &res) {
		
		// Store the objects to mongodb.
		std::string id = message_store.insertNamed(req.id, req.cloud);
		mongo_id_mapping.insert(std::make_pair(req.id, id));
		
		std_msgs::String ros_string_category;
		ros_string_category.data = req.category;
		message_store.insertNamed(req.id, ros_string_category);
		mongo_id_mapping.insert(std::make_pair(req.id, id));
		message_store.insertNamed(req.id, req.pose);
		mongo_id_mapping.insert(std::make_pair(req.id, id));
		
		// Store the mappings to the knowledge base.
		rosplan_knowledge_msgs::KnowledgeUpdateService wpSrv;
		wpSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		wpSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
		wpSrv.request.knowledge.instance_type = "object";
		wpSrv.request.knowledge.instance_name = req.id;
		if (!update_knowledge_client.call(wpSrv)) {
			ROS_ERROR("KCL: (ObjectPerception) error adding knowledge");
			res.result = squirrel_planning_knowledge_msgs::AddObjectService::Response::FAILURE;
			return false;
		}

		// Add "tidy object" goal to the knowledge base "(tidy ?o - object)"
		rosplan_knowledge_msgs::KnowledgeUpdateService toSrv;
		toSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
		toSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE;
		toSrv.request.knowledge.attribute_name = "tidy";
		diagnostic_msgs::KeyValue oPair;
		oPair.key = "o";
		oPair.value = req.id;
		toSrv.request.knowledge.values.push_back(oPair);
		if (!update_knowledge_client.call(toSrv)) {
			ROS_ERROR("KCL: (ObjectPerception) error adding tidy goal");
			res.result = squirrel_planning_knowledge_msgs::AddObjectService::Response::FAILURE;
			return false;
		}
		
		res.result = squirrel_planning_knowledge_msgs::AddObjectService::Response::SUCCESS;
		return true;
	}

	/* remove object from knowledge base and scene database */
	bool RPObjectPerception::removeObjects(
		squirrel_planning_knowledge_msgs::RemoveObjectService::Request &req,
		squirrel_planning_knowledge_msgs::RemoveObjectService::Response &res) {
		
		// Remove all data associated with the given id.
		std::pair<std::multimap<std::string, std::string>::iterator, std::multimap<std::string, std::string>::iterator> mm_ci;
		mm_ci = mongo_id_mapping.equal_range(req.id);
		for (std::multimap<std::string, std::string>::const_iterator ci = mm_ci.first; ci != mm_ci.second; ++ci) {
			message_store.deleteID((*ci).second);
		}
		
		// Remove the mappings from knowledge base.
		rosplan_knowledge_msgs::KnowledgeUpdateService wpSrv;
		wpSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
		wpSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
		wpSrv.request.knowledge.instance_type = "object";
		wpSrv.request.knowledge.instance_name = req.id;
		if (!update_knowledge_client.call(wpSrv)) {
			ROS_ERROR("KCL: (ObjectPerception) error removing knowledge");
			res.result = squirrel_planning_knowledge_msgs::RemoveObjectService::Response::FAILURE;
			return false;
		}
		
		res.result = squirrel_planning_knowledge_msgs::RemoveObjectService::Response::SUCCESS;
		return true;
	}

	/* update object in scene database and recalculate for knowledge base */
	bool RPObjectPerception::updateObjects(
		squirrel_planning_knowledge_msgs::UpdateObjectService::Request &req,
		squirrel_planning_knowledge_msgs::UpdateObjectService::Response &res) {
		
		// Update all data associated with the given id -- the types might be swapped, but this should only affect performance.
		std::pair<std::multimap<std::string, std::string>::iterator, std::multimap<std::string, std::string>::iterator> mm_ci;
		mm_ci = mongo_id_mapping.equal_range(req.id);
		
		// We expect exactly three instances in the DB, anything else indicates a failure.
		int found_instances = std::distance(mm_ci.first, mm_ci.second);
		if (found_instances != 3) {
			ROS_ERROR("KCL: (ObjectPerception) error updating, we expected mongodb to contain 3 instances for id %s, instead we found %d.", req.id.c_str(), found_instances);
			res.result = squirrel_planning_knowledge_msgs::UpdateObjectService::Response::FAILURE;
			return false;
		}
		
		std::multimap<std::string, std::string>::const_iterator update_ci = mm_ci.first;
		message_store.updateNamed((*update_ci).second, req.cloud); ++update_ci;

		std_msgs::String ros_string_category;
		ros_string_category.data = req.category;
		message_store.updateNamed((*update_ci).second, ros_string_category); ++update_ci;
		message_store.updateNamed((*update_ci).second, req.pose);
		
		res.result = squirrel_planning_knowledge_msgs::UpdateObjectService::Response::SUCCESS;
		return true;
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
