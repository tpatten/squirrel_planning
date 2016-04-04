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

#include "ShedKnowledgePDDLAction.h"
#include "squirrel_planning_execution/ViewConeGenerator.h"

namespace KCL_rosplan
{

ShedKnowledgePDDLAction::ShedKnowledgePDDLAction(ros::NodeHandle& node_handle)
{
	action_feedback_pub_ = node_handle.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);

	// Subscribe to the action feedback topic.
	dispatch_sub_ = node_handle.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::ShedKnowledgePDDLAction::dispatchCallback, this);
}

ShedKnowledgePDDLAction::~ShedKnowledgePDDLAction()
{
	
}

void ShedKnowledgePDDLAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
{
	std::string normalised_action_name = msg->name;
	std::transform(normalised_action_name.begin(), normalised_action_name.end(), normalised_action_name.begin(), tolower);
	
	// Check if this action is to be handled by this class.
	if (normalised_action_name != "shed_knowledge")
	{
		return;
	}
	
	// Report this action is enabled and completed successfully.
	rosplan_dispatch_msgs::ActionFeedback fb;
	fb.action_id = msg->action_id;
	fb.status = "action enabled";
	action_feedback_pub_.publish(fb);
	
	fb.action_id = msg->action_id;
	fb.status = "action achieved";
	action_feedback_pub_.publish(fb);
}

};
