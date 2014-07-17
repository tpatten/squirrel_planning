#include "perceptionInterface.h"

/**
 * ROS node stub for SQUIRREL Summer School: PUSH
 * (this code is just a suggestion)
 */
namespace SQUIRREL_summerschool_perception {

	void PerceptionInterface::executePush(const planning_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// acknowledgement feedback
		planning_dispatch_msgs::ActionFeedback feedbackEnabled;
		feedbackEnabled.action_id = msg->action_id;
		feedbackEnabled.status = "action enabled";
		feedbackPub.publish(feedbackEnabled);

		// TODO: push the object!

		if(actionCancelled[msg->action_id]) {
			// TODO: push is cancelled; finish immediately.
		}

		// completion feedback
		planning_dispatch_msgs::ActionFeedback feedbackAchieved;
		feedbackAchieved.action_id = msg->action_id;
		feedbackAchieved.status = "action achieved";
		feedbackPub.publish(feedbackAchieved);
	}

} // close namespace
