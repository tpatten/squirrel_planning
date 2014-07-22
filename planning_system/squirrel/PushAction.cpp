#include "perceptionInterface.h"
#include "planning_knowledge_msgs/PositionService.h"

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

		// get object position
		std::string objectID = msg->parameters[0].value;
		ros::ServiceClient positionClient = nh.serviceClient<planning_knowledge_msgs::PositionService>("/kcl_rosplan/get_object_position");
		planning_knowledge_msgs::PositionService positionSrv;
		positionSrv.request.name = objectID;
		ROS_INFO("Push: getting object position.");
		if (positionClient.call(positionSrv))
		{
			geometry_msgs::Point pos = positionSrv.response.position;
			ROS_INFO("pushing object ID '%s' at [%.3f %.3f %.3f]\n",
				objectID.c_str(), (float)pos.x, (float)pos.y, (float)pos.z);

			// TODO: push the object, and then retreat
			// <your code here>
		}

		// completion feedback
		planning_dispatch_msgs::ActionFeedback feedbackAchieved;
		feedbackAchieved.action_id = msg->action_id;
		feedbackAchieved.status = "action achieved";
		feedbackPub.publish(feedbackAchieved);
	}

} // close namespace
