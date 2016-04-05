#include "squirrel_interface_manipulation/RPGraspAction.h"

/* The implementation of RPGraspAction.h */
namespace KCL_rosplan {

	/* constructor */
	RPGraspAction::RPGraspAction(ros::NodeHandle &nh, std::string &blindGraspActionServer)
	 : message_store(nh), blind_grasp_action_client(blindGraspActionServer, true) {

		// create the action clients
		ROS_INFO("KCL: (GraspAction) waiting for action server to start on %s", blindGraspActionServer.c_str());
		blind_grasp_action_client.waitForServer();

		// create the action feedback publisher
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
	}

	/* action dispatch callback */
	void RPGraspAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// ignore non-grasp actions
		if(0==msg->name.compare("pickup_object")) dispatchBlindGraspAction(msg);
	}

	/* blind grasp action dispatch */
	void RPGraspAction::dispatchBlindGraspAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		ROS_INFO("KCL: (GraspAction) blind grasp action recieved");

		// get object ID from action dispatch
		std::string objectID;
		bool foundObject = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			if(0==msg->parameters[i].key.compare("ob")) {
				objectID = msg->parameters[i].value;
				foundObject = true;
			}
		}
		if(!foundObject) {
			ROS_INFO("KCL: (GraspAction) aborting action dispatch; malformed parameters");
			return;
		}
		
		// get details from message store
		std::vector< boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> > results;
		if(message_store.queryNamed<squirrel_object_perception_msgs::SceneObject>(objectID, results)) {
			if(results.size()<1) {
				ROS_INFO("KCL: (GraspAction) aborting action dispatch; no matching objectID %s", objectID.c_str());
				return;
			}
			if(results.size()>1)
				ROS_INFO("KCL: (GraspAction) multiple objects share the same objectID");

			// dispatch Grasp action
			squirrel_manipulation_msgs::BlindGraspGoal goal;
			goal.heap_center_pose = results[0]->pose;
//			goal.heap_bounding_cylinder = results[0].bounding_cylinder;
			goal.heap_point_cloud = results[0]->cloud;
			blind_grasp_action_client.sendGoal(goal);

			// publish feedback (enabled)
			rosplan_dispatch_msgs::ActionFeedback fb;
			fb.action_id = msg->action_id;
			fb.status = "action enabled";
			action_feedback_pub.publish(fb);

			
			bool finished_before_timeout = false;
			finished_before_timeout = blind_grasp_action_client.waitForResult(ros::Duration(msg->duration));

			actionlib::SimpleClientGoalState state = blind_grasp_action_client.getState();
			ROS_INFO("KCL: (GraspAction) action finished: %s", state.toString().c_str());
				
			// publish feedback (achieved)
			fb.action_id = msg->action_id;
			fb.status = "action achieved";
			action_feedback_pub.publish(fb);

		} else ROS_INFO("KCL: (GraspAction) aborting action dispatch; query to sceneDB failed");
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_grasping");
		ros::NodeHandle nh;


		std::string GraspActionserver, blindGraspActionServer;
		nh.param("blind_grasp_action_server", blindGraspActionServer, std::string("/grasp"));

		// create PDDL action subscriber
		KCL_rosplan::RPGraspAction rpga(nh, blindGraspActionServer);
	
		// listen for action dispatch
		ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPGraspAction::dispatchCallback, &rpga);
		ROS_INFO("KCL: (GraspAction) Ready to receive");

		ros::spin();
		return 0;
	}
