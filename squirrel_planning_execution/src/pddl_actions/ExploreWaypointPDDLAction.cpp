#include <sstream>
#include <complex>
#include <geometry_msgs/Pose.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_dispatch_msgs/ActionFeedback.h>

#include "ExploreWaypointPDDLAction.h"

namespace KCL_rosplan
{

ExploreWaypointPDDLAction::ExploreWaypointPDDLAction(ros::NodeHandle& node_handle)
{
	// knowledge interface
	update_knowledge_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
	get_instance_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
	get_attribute_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
	action_feedback_pub_ = node_handle.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);

	// Subscribe to the action feedback topic.
	dispatch_sub_ = node_handle.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::ExploreWaypointPDDLAction::dispatchCallback, this);
}

ExploreWaypointPDDLAction::~ExploreWaypointPDDLAction()
{
	
}

void ExploreWaypointPDDLAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
{
	std::string normalised_action_name = msg->name;
	std::transform(normalised_action_name.begin(), normalised_action_name.end(), normalised_action_name.begin(), tolower);
	
	// Check if this action is to be handled by this class.
	if (normalised_action_name != "explore_waypoint" || msg->parameters.size() != 2)
	{
		return;
	}
	
	ROS_INFO("KCL: (ExploreWaypointPDDLAction) Process the action: %s", normalised_action_name.c_str());
	
	// Report this action is enabled and completed successfully.
	rosplan_dispatch_msgs::ActionFeedback fb;
	fb.action_id = msg->action_id;
	fb.status = "action enabled";
	action_feedback_pub_.publish(fb);
	
	// Update the domain.
	const std::string& robot = msg->parameters[0].value;
	const std::string& explored_waypoint = msg->parameters[1].value;
	
	ROS_INFO("KCL: (ExploreWaypointPDDLAction) Process the action: %s, %s explored %s", normalised_action_name.c_str(), robot.c_str(), explored_waypoint.c_str());
	
	// Add the new knowledge.
	rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
	knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
	rosplan_knowledge_msgs::KnowledgeItem kenny_knowledge;
	kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
	kenny_knowledge.attribute_name = "explored";
	kenny_knowledge.is_negative = false;
	
	diagnostic_msgs::KeyValue kv;
	kv.key = "wp";
	kv.value = explored_waypoint;
	kenny_knowledge.values.push_back(kv);
	
	knowledge_update_service.request.knowledge = kenny_knowledge;
	if (!update_knowledge_client_.call(knowledge_update_service)) {
		ROS_ERROR("KCL: (ExploreWaypointPDDLAction) Could not add the explord predicate to the knowledge base.");
		exit(-1);
	}
	ROS_INFO("KCL: (ExploreWaypointPDDLAction) Added the explored predicate to the knowledge base.");
	
	// Get all the objects from the knowledge base, so we can give our objects a unique name.
	// Check if this object has been classified or not.
	rosplan_knowledge_msgs::GetInstanceService get_instance;
	get_instance.request.type_name = "object";
	
	if (!get_instance_client_.call(get_instance))
	{
		ROS_ERROR("KCL: (ExploreWaypointPDDLAction) Could not get the instances of type 'object'.");
		exit(1);
	}
	
	unsigned int object_nr = get_instance.response.instances.size();
	
	// Simulate that we found some (or none!) objects at this waypoint.
	unsigned int new_objects =  rand() % 2;
	for (unsigned int i = 0; i < new_objects; ++i)
	{
		std::stringstream ss;
		ss << "object" << object_nr;
		std::string object_name = ss.str();
		
		ss.str(std::string());
		ss << "waypoint_object" << object_nr;
		std::string waypoint_name = ss.str();
		
		rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
		knowledge_item.instance_type = "object";
		knowledge_item.instance_name = object_name;
		
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client_.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (ExploreWaypointPDDLAction) Could not add the object %s to the knowledge base.", object_name.c_str());
			exit(-1);
		}
		ROS_INFO("KCL: (ExploreWaypointPDDLAction) Added %s to the knowledge base.", object_name.c_str());
		
		knowledge_item.instance_type = "waypoint";
		knowledge_item.instance_name = waypoint_name;
		
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client_.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (ExploreWaypointPDDLAction) Could not add the waypoint %s to the knowledge base.", waypoint_name.c_str());
			exit(-1);
		}
		ROS_INFO("KCL: (ExploreWaypointPDDLAction) Added %s to the knowledge base.", waypoint_name.c_str());
		
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_item.attribute_name = "object_at";
		knowledge_item.is_negative = false;
		
		diagnostic_msgs::KeyValue kv;
		kv.key = "o";
		kv.value = object_name;
		knowledge_item.values.push_back(kv);
		
		kv.key = "wp";
		kv.value = waypoint_name;
		knowledge_item.values.push_back(kv);
		
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client_.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (ExploreWaypointPDDLAction) Could not add the new (object_at %s %s) predicate to the knowledge base.", object_name.c_str(), waypoint_name.c_str());
			exit(-1);
		}
		ROS_INFO("KCL: (ExploreWaypointPDDLAction) Added the new (object_at %s %s)  predicate to the knowledge base.", object_name.c_str(), waypoint_name.c_str());
		++object_nr;
	}
	
	ROS_INFO("KCL: (ExploreWaypointPDDLAction) Added %d new objects!", new_objects);
	
	fb.action_id = msg->action_id;
	fb.status = "action achieved";
	action_feedback_pub_.publish(fb);
}

};
