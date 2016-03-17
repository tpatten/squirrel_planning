#include <sstream>
#include <complex>

#include <cstdlib>

#include <geometry_msgs/Pose.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_knowledge_msgs/KnowledgeQueryService.h>
#include <rosplan_dispatch_msgs/ActionFeedback.h>

#include <diagnostic_msgs/KeyValue.h>

#include "ClassifyObjectPDDLAction.h"
#include "squirrel_planning_execution/ViewConeGenerator.h"

namespace KCL_rosplan
{

ClassifyObjectPDDLAction::ClassifyObjectPDDLAction(ros::NodeHandle& node_handle, float classification_probability)
	: classification_probability_(classification_probability)
{
	// knowledge interface
	update_knowledge_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
	get_instance_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
	get_attribute_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
	action_feedback_pub_ = node_handle.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
	//query_knowledge_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>("/kcl_rosplan/query_knowledge_base");

	// Subscribe to the action feedback topic.
	dispatch_sub_ = node_handle.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::ClassifyObjectPDDLAction::dispatchCallback, this);
	
	// Initialise the random number generator with a fixed number so we can reproduce the same results.
	srand (1234);
}

ClassifyObjectPDDLAction::~ClassifyObjectPDDLAction()
{
	
}

void ClassifyObjectPDDLAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
{
	std::string normalised_action_name = msg->name;
	std::transform(normalised_action_name.begin(), normalised_action_name.end(), normalised_action_name.begin(), tolower);
	
	// Check if this action is to be handled by this class.
	if (normalised_action_name != "observe-classifiable_from" && 
	    normalised_action_name != "finalise_classification" &&
	    normalised_action_name != "finalise_classification_nowhere" &&
	    normalised_action_name != "finalise_classification_success" &&
	    normalised_action_name != "finalise_classification_fail")
	{
		return;
	}
	
	ROS_INFO("KCL (ClassifyObjectPDDLAction) Process the action: %s", normalised_action_name.c_str());
	/*
	for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = msg->parameters.begin(); ci != msg->parameters.end(); ++ci)
	{
		const std::string& key = (*ci).key;
		const std::string& value = (*ci).value;
		
		ROS_INFO("KCL (ClassifyObjectPDDLAction) %s -> %s", key.c_str(), value.c_str());
	}
	*/
	
	// Report this action is enabled and completed successfully.
	rosplan_dispatch_msgs::ActionFeedback fb;
	fb.action_id = msg->action_id;
	fb.status = "action enabled";
	action_feedback_pub_.publish(fb);
	
	// We can ignore some set of actions as they only do bookkeeping for the planner.
	if (normalised_action_name == "observe-classifiable_from")
	{
		// Update the domain.
		const std::string& from = msg->parameters[0].value;
		const std::string& view = msg->parameters[1].value;
		const std::string& object = msg->parameters[2].value;
		
		// Add the new knowledge.
		rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_item.attribute_name = "classifiable_from";
		
		knowledge_item.is_negative = (float)rand() / (float)RAND_MAX >= classification_probability_;
		
		diagnostic_msgs::KeyValue kv;
		kv.key = "from";
		kv.value = from;
		knowledge_item.values.push_back(kv);
		
		kv.key = "view";
		kv.value = view;
		knowledge_item.values.push_back(kv);
		
		kv.key = "o";
		kv.value = object;
		knowledge_item.values.push_back(kv);
		
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client_.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (ClassifyObjectPDDLAction) Could not add the classifiable_from predicate to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (ClassifyObjectPDDLAction) Added %s classifiable_from predicate to the knowledge base.", knowledge_item.is_negative ? "NOT" : "");
		knowledge_item.values.clear();
		
		// If the fact is not negative, we need to decide upon the type of this object.
		if (!knowledge_item.is_negative)
		{
			// Check if this object has been classified or not.
			rosplan_knowledge_msgs::GetInstanceService get_instance;
			get_instance.request.type_name = "type";
			
			if (!get_instance_client_.call(get_instance) || get_instance.response.instances.empty())
			{
				ROS_ERROR("KCL: (ClassifyObjectPDDLAction) Could not get the instances of type 'type'.");
				exit(1);
			}
			
			// Select one type at random.
			const std::string type = get_instance.response.instances[rand() % get_instance.response.instances.size()];
			
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			knowledge_item.attribute_name = "is_of_type";
			
			diagnostic_msgs::KeyValue kv;
			kv.key = "o";
			kv.value = object;
			knowledge_item.values.push_back(kv);
			
			kv.key = "t";
			kv.value = type;
			knowledge_item.values.push_back(kv);
			
			knowledge_item.is_negative = false;
			knowledge_update_service.request.knowledge = knowledge_item;
			
			// Check if any of these facts are true.
			if (!update_knowledge_client_.call(knowledge_update_service))
			{
				ROS_ERROR("KCL: (ClassifyObjectPDDLAction) Could not add the (is_of_type %s %s) predicate to the knowledge base.", object.c_str(), type.c_str());
				exit(1);
			}
			ROS_ERROR("KCL: (ClassifyObjectPDDLAction) Added the (is_of_type %s %s) predicate to the knowledge base.", object.c_str(), type.c_str());
		}
	}
	
	fb.action_id = msg->action_id;
	fb.status = "action achieved";
	action_feedback_pub_.publish(fb);
}

};
