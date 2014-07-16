#include "perceptionInterface.h"

/**
 * ROS node for SQUIRREL Summer School.
 * Contains interface for calling various controllers depending on action dispatch.
 * This file could service all actions implemented by a particular group.
 * It should call the relevant controllers using actionlib.
 */
namespace SQUIRREL_summerschool_perception {

	void PerceptionInterface::dispatchCallback(const planning_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		if(0 == msg->name.compare("cancel_action"))
			actionCancelled[msg->action_id] = true;
		else if(0 == msg->name.compare("observe"))
			executeObserve(msg);
		else if(0 == msg->name.compare("classify"))
			executeClassify(msg);
	}

	void PerceptionInterface::executeObserve(const planning_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

			planning_dispatch_msgs::ActionFeedback feedbackEnabled;
			feedbackEnabled.action_id = msg->action_id;
			feedbackEnabled.status = "action enabled";
			feedbackPub.publish(feedbackEnabled);

//

			planning_dispatch_msgs::ActionFeedback feedbackAchieved;
			feedbackAchieved.action_id = msg->action_id;
			feedbackAchieved.status = "action achieved";
			feedbackPub.publish(feedbackAchieved);
	}

	void PerceptionInterface::executeClassify(const planning_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

			planning_dispatch_msgs::ActionFeedback feedbackEnabled;
			feedbackEnabled.action_id = msg->action_id;
			feedbackEnabled.status = "action enabled";
			feedbackPub.publish(feedbackEnabled);

//

			planning_dispatch_msgs::ActionFeedback feedbackAchieved;
			feedbackAchieved.action_id = msg->action_id;
			feedbackAchieved.status = "action achieved";
			feedbackPub.publish(feedbackAchieved);
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc,argv,"squirrel_perception_interface");
		ros::NodeHandle nh("~");

		SQUIRREL_summerschool_perception::PerceptionInterface pi;
		pi.feedbackPub = nh.advertise<planning_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 1000, true);
		ros::Subscriber dispatchSub = nh.subscribe("/kcl_rosplan/action_dispatch", 1000,
			&SQUIRREL_summerschool_perception::PerceptionInterface::dispatchCallback, &pi);

		ros::spin();
		return 0;
	}
