#include <sstream>
#include <complex>
#include <geometry_msgs/Pose.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_dispatch_msgs/ActionFeedback.h>


#include "ClearObjectPDDLAction.h"
#include "squirrel_planning_execution/ViewConeGenerator.h"

namespace KCL_rosplan
{

ClearObjectPDDLAction::ClearObjectPDDLAction(ros::NodeHandle& node_handle)
{
	// knowledge interface
	update_knowledge_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
	get_instance_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
	get_attribute_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
	action_feedback_pub_ = node_handle.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);

	// Subscribe to the action feedback topic.
	dispatch_sub_ = node_handle.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::ClearObjectPDDLAction::dispatchCallback, this);
}

ClearObjectPDDLAction::~ClearObjectPDDLAction()
{
	
}

void ClearObjectPDDLAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
{
	std::string normalised_action_name = msg->name;
	std::transform(normalised_action_name.begin(), normalised_action_name.end(), normalised_action_name.begin(), tolower);
	
	// Check if this action is to be handled by this class.
	if (normalised_action_name != "clear_object" || msg->parameters.size() != 2)
	{
		return;
	}
	
	ROS_INFO("KCL (ClearObjectPDDLAction) Process the action: %s", normalised_action_name.c_str());
	for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = msg->parameters.begin(); ci != msg->parameters.end(); ++ci)
	{
		const std::string& key = (*ci).key;
		const std::string& value = (*ci).value;
		
		ROS_INFO("KCL (ClearObjectPDDLAction) %s -> %s", key.c_str(), value.c_str());
	}
	
	// Report this action is enabled and completed successfully.
	rosplan_dispatch_msgs::ActionFeedback fb;
	fb.action_id = msg->action_id;
	fb.status = "action enabled";
	action_feedback_pub_.publish(fb);
	
	// Update the domain.
	const std::string& object = msg->parameters[0].value;
	const std::string& state = msg->parameters[1].value;
	
	ROS_INFO("KCL (ClearObjectPDDLAction) Process the action: %s, Clear the area around object %s in state %s", normalised_action_name.c_str(), object.c_str(), state.c_str());
	
	// Remove the old knowledge.
	rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
	knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
	rosplan_knowledge_msgs::KnowledgeItem kenny_knowledge;
	kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
	kenny_knowledge.attribute_name = "cleared";
	
	diagnostic_msgs::KeyValue kv;
	kv.key = "o";
	kv.value = object;
	kenny_knowledge.values.push_back(kv);
	
	kv.key = "s";
	kv.value = state;
	kenny_knowledge.values.push_back(kv);
	
	knowledge_update_service.request.knowledge = kenny_knowledge;
	if (!update_knowledge_client_.call(knowledge_update_service)) {
		ROS_ERROR("KCL: (ClearObjectPDDLAction) Could not remove the previous robot_at predicate from the knowledge base.");
		exit(-1);
	}
	ROS_INFO("KCL: (ClearObjectPDDLAction) Removed the previous robot_at predicate from the knowledge base.");
	
	fb.action_id = msg->action_id;
	fb.status = "action achieved";
	action_feedback_pub_.publish(fb);
}

};
