#include <sstream>
#include <complex>
#include <geometry_msgs/Pose.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_dispatch_msgs/ActionFeedback.h>


#include "NextTurnPDDLAction.h"

namespace KCL_rosplan
{

NextTurnPDDLAction::NextTurnPDDLAction(ros::NodeHandle& node_handle)
{
	// knowledge interface
	update_knowledge_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
	get_instance_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
	get_attribute_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
	action_feedback_pub_ = node_handle.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);

	// Subscribe to the action feedback topic.
	dispatch_sub_ = node_handle.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::NextTurnPDDLAction::dispatchCallback, this);
}

NextTurnPDDLAction::~NextTurnPDDLAction()
{
	
}

void NextTurnPDDLAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
{
	std::string normalised_action_name = msg->name;
	std::transform(normalised_action_name.begin(), normalised_action_name.end(), normalised_action_name.begin(), tolower);
	
	// Check if this action is to be handled by this class.
	if (normalised_action_name != "next_turn" || msg->parameters.size() != 2)
	{
		return;
	}
	
	ROS_INFO("KCL: (NextTurnPDDLAction) Process the action: %s", normalised_action_name.c_str());
	
	// Report this action is enabled and completed successfully.
	rosplan_dispatch_msgs::ActionFeedback fb;
	fb.action_id = msg->action_id;
	fb.status = "action enabled";
	action_feedback_pub_.publish(fb);
	
	// Update the domain.
	const std::string& current_kid = msg->parameters[0].value;
	const std::string& next_kid = msg->parameters[1].value;
	
	ROS_INFO("KCL: (NextTurnPDDLAction) Process the action: %s, Swap the turn from %s to %s", normalised_action_name.c_str(), current_kid.c_str(), next_kid.c_str());
	
	// Remove the old knowledge.
	rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
	knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
	rosplan_knowledge_msgs::KnowledgeItem kenny_knowledge;
	kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
	kenny_knowledge.attribute_name = "current_turn";
	kenny_knowledge.is_negative = false;
	
	diagnostic_msgs::KeyValue kv;
	kv.key = "k";
	kv.value = current_kid;
	kenny_knowledge.values.push_back(kv);
	
	knowledge_update_service.request.knowledge = kenny_knowledge;
	if (!update_knowledge_client_.call(knowledge_update_service)) {
		ROS_ERROR("KCL: (NextTurnPDDLAction) Could not remove (current_turn %s) predicate from the knowledge base.", current_kid.c_str());
		exit(-1);
	}
	ROS_INFO("KCL: (NextTurnPDDLAction) Removed the previous (current_turn %s) predicate from the knowledge base.", current_kid.c_str());
	kenny_knowledge.values.clear();
	
	// Add the new knowledge.
	knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
	
	kv.value = next_kid;
	kenny_knowledge.values.push_back(kv);
	
	knowledge_update_service.request.knowledge = kenny_knowledge;
	if (!update_knowledge_client_.call(knowledge_update_service)) {
		ROS_ERROR("KCL: (NextTurnPDDLAction) Could not add (current_turn %s) predicate to the knowledge base.", next_kid.c_str());
		exit(-1);
	}
	ROS_INFO("KCL: (NextTurnPDDLAction) Added (current_turn %s) predicate to the knowledge base.", next_kid.c_str());
	
	fb.action_id = msg->action_id;
	fb.status = "action achieved";
	action_feedback_pub_.publish(fb);
}

};
