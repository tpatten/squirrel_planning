#include "perceptionInterface.h"

/**
 * ROS node stub for SQUIRREL Summer School: EXPLORE
 * (this code is just a suggestion)
 */
namespace SQUIRREL_summerschool_perception {

	void PerceptionInterface::executeExplore(const planning_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// acknowledgement feedback
		planning_dispatch_msgs::ActionFeedback feedbackEnabled;
		feedbackEnabled.action_id = msg->action_id;
		feedbackEnabled.status = "action enabled";
		feedbackPub.publish(feedbackEnabled);

		// TODO: move around intelligently
		// <your code here>

		// in some places:		
		executeObserve(msg);

		if(actionCancelled[msg->action_id]) {
			// TODO: explore is cancelled; finish immediately.
			// -> make the robot stop
			// <your code here>
		}

		// after about "msg->duration" seconds of exploration:
		// TODO: after some reasonable time stop exploring
		// <your code here>

		// completion feedback
		planning_dispatch_msgs::ActionFeedback feedbackAchieved;
		feedbackAchieved.action_id = msg->action_id;
		feedbackAchieved.status = "action achieved";
		feedbackPub.publish(feedbackAchieved);
	}

} // close namespace
