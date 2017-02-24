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

#include "FinaliseClassificationPDDLAction.h"

namespace KCL_rosplan
{

FinaliseClassificationPDDLAction::FinaliseClassificationPDDLAction(ros::NodeHandle& node_handle)
{
	// knowledge interface
	action_feedback_pub_ = node_handle.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);

	// Subscribe to the action feedback topic.
	dispatch_sub_ = node_handle.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::FinaliseClassificationPDDLAction::dispatchCallback, this);
}

FinaliseClassificationPDDLAction::~FinaliseClassificationPDDLAction()
{
	
}

void FinaliseClassificationPDDLAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
{
	std::string normalised_action_name = msg->name;
	std::transform(normalised_action_name.begin(), normalised_action_name.end(), normalised_action_name.begin(), tolower);
	
	// Check if this action is to be handled by this class.
	if (normalised_action_name != "finalise_classification" &&
	    normalised_action_name != "finalise_classification_nowhere" &&
	    normalised_action_name != "finalise_classification_success" &&
	    normalised_action_name != "finalise_classification_fail")
	{
		return;
	}
	
	// Report this action is enabled and completed successfully.
	rosplan_dispatch_msgs::ActionFeedback fb;
	fb.action_id = msg->action_id;
	fb.status = "action enabled";
	action_feedback_pub_.publish(fb);
		
	ROS_INFO("KCL: (FinaliseClassificationPDDLAction) Ignore action: (%s)", normalised_action_name.c_str());
	
	fb.action_id = msg->action_id;
	fb.status = "action achieved";
	action_feedback_pub_.publish(fb);
}

};
