#include "squirrel_interface_manipulation/RPGraspAction.h"

/* The implementation of RPGraspAction.h */
namespace KCL_rosplan {

	/* constructor */
	RPGraspAction::RPGraspAction(ros::NodeHandle &nh, std::string &blindGraspActionServer)
	 : message_store(nh), blind_grasp_action_client(blindGraspActionServer, true) { //, drop_action_client(std::string("/hand_controller/actuate_hand"), true) {

		// create the action clients
		drop_client = nh.serviceClient<kclhand_control::graspPreparation>("/hand_controller/openFinger");
		ROS_INFO("KCL: (GraspAction) waiting for action server to start on %s", blindGraspActionServer.c_str());
		//std::string drop_action_server = "/hand_controller/actuate_hand";
		//drop_action_client(drop_action_server, true);
		//ROS_INFO("KCL: (GraspAction) waiting for action server to start on /hand_controller/actuate_hand");
		//blind_grasp_action_client.waitForServer();
		//drop_action_client.waitForServer();
		ROS_INFO("KCL: (GraspAction) action servers found");

		// create the action feedback publisher
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);

		// create knowledge base link
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
	}

	/* action dispatch callback */
	void RPGraspAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		ROS_INFO("Grasp server callback function for action name %s", msg->name.c_str());
		rosplan_dispatch_msgs::ActionFeedback fb;

		// ignore non-grasp actions
		if(0==msg->name.compare("pickup_object")) {

			publishFeedback(msg->action_id, "action enabled");
//			for(int i=0; i<5; i++) {
				if(dispatchBlindGraspAction(msg)) {
					publishFeedback(msg->action_id, "action achieved");
					return;
				}
//			}
			publishFeedback(msg->action_id, "action achieved");
		}

		// ignore non-drop objects
		else if(0==msg->name.compare("putdown_object") || 0==msg->name.compare("put_object_in_box")) {

			publishFeedback(msg->action_id, "action enabled");
			if(dispatchDropActionCorrect(msg)) {
				publishFeedback(msg->action_id, "action achieved");
				return;
			}
			publishFeedback(msg->action_id, "action failed");
		}
	}

	/* blind grasp action dispatch
		:parameters (?v - robot ?wp - waypoint ?o - object)
		:effect (and
				(not (holding ?v ?o))
				(gripper_empty ?v)
				(object_at ?o ?wp)
	*/
	bool RPGraspAction::dispatchDropAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// get object ID from action dispatch
		std::string robotID, objectID, wpID;
		bool foundObject = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			if(0==msg->parameters[i].key.compare("wp"))
				wpID = msg->parameters[i].value;
			if(0==msg->parameters[i].key.compare("r"))
				robotID = msg->parameters[i].value;
			if(0==msg->parameters[i].key.compare("ob")) {
				objectID = msg->parameters[i].value;
				foundObject = true;
			}
		}
		if(!foundObject) {
			ROS_INFO("KCL: (GraspAction) aborting action dispatch; malformed parameters");
			return false;
		}

		// dispatch action as service
		kclhand_control::graspPreparation srv;
		if(drop_client.call(srv)) {

			// gripper_empty fact
			rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
			knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			knowledge_update_service.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			knowledge_update_service.request.knowledge.attribute_name = "gripper_empty";
			diagnostic_msgs::KeyValue kv;
			kv.key = "v";
			kv.value = robotID;
			knowledge_update_service.request.knowledge.values.push_back(kv);
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (PerceptionAction) Could not add gripper_empty predicate to the knowledge base.");
			}

			// holding fact
			knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
			knowledge_update_service.request.knowledge.attribute_name = "holding";
			kv.key = "o";
			kv.value = objectID;
			knowledge_update_service.request.knowledge.values.push_back(kv);
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (PerceptionAction) Could not remove holding predicate from the knowledge base.");
			}

			// object_at fact
			knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			knowledge_update_service.request.knowledge.attribute_name = "object_at";
			knowledge_update_service.request.knowledge.values.clear();
			kv.key = "o";
			kv.value = objectID;
			knowledge_update_service.request.knowledge.values.push_back(kv);
			kv.key = "wp";
			kv.value = wpID;
			knowledge_update_service.request.knowledge.values.push_back(kv);
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (PerceptionAction) Could not remove object_at predicate to the knowledge base.");
			}
			
			return true;
		}
		return false;
	}

	bool RPGraspAction::dispatchDropActionCorrect(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		ROS_INFO("KCL: (DropAction) attempting to drop object");
		// get object ID from action dispatch
		std::string robotID, objectID, wpID;
		bool foundObject = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			std::cout << msg->parameters[i].key << std::endl;
			if(0==msg->parameters[i].key.compare("wp"))
				wpID = msg->parameters[i].value;
			if(0==msg->parameters[i].key.compare("r"))
				robotID = msg->parameters[i].value;
			if(0==msg->parameters[i].key.substr(0,1).compare("o")) {
				objectID = msg->parameters[i].value;
				foundObject = true;
				std::cout << "foundObject is TRUE" << std::endl;
			}
		}
		if(!foundObject) {
			ROS_INFO("KCL: (DropAction) aborting action dispatch; malformed parameters");
			return false;
		}

		// Set up action servers
		actionlib::SimpleActionClient<squirrel_manipulation_msgs::PutDownAction> putDownActionClient("metahand_place_server", true);
        	ROS_INFO("Waiting for action server to start on metahand_place_server");
		putDownActionClient.waitForServer();
		actionlib::SimpleActionClient<kclhand_control::ActuateHandAction> kclhandGraspActionClient("hand_controller/actuate_hand", true);
        	ROS_INFO("Waiting for action server to start on hand_controller/actuate_hand");
		kclhandGraspActionClient.waitForServer();
		ROS_INFO("Found action servers! Dropping the object!");
		bool success = true;

		
		// Call action to move arm to put down location
		squirrel_manipulation_msgs::PutDownActionGoal put_down_goal;
		put_down_goal.goal.destination_id = "box";
        	put_down_goal.goal.destPoseSE2.header.frame_id = "origin";
		put_down_goal.goal.destPoseSE2.pose.position.x = 0.28;
        	put_down_goal.goal.destPoseSE2.pose.position.y = -1.41;
        	put_down_goal.goal.destPoseSE2.pose.position.z = 0.285;
        	//put_down_goal.goal.destPoseSE2.pose.orientation.w = -0.408;
        	//put_down_goal.goal.destPoseSE2.pose.orientation.x = 0.896;
        	//put_down_goal.goal.destPoseSE2.pose.orientation.y = 0.154;
        	//put_down_goal.goal.destPoseSE2.pose.orientation.z = 0.088;
		put_down_goal.goal.destPoseSE2.pose.orientation.w = -0.397;
                put_down_goal.goal.destPoseSE2.pose.orientation.x = 0.906;
                put_down_goal.goal.destPoseSE2.pose.orientation.y = 0.066;
                put_down_goal.goal.destPoseSE2.pose.orientation.z = 0.127;

		// Call action
		putDownActionClient.sendGoal(put_down_goal.goal);
		putDownActionClient.waitForResult();
		sleep(1.0);
		actionlib::SimpleClientGoalState state_put_down = putDownActionClient.getState();
		ROS_INFO("KCL: (DropAction) action finished: %s", state_put_down.toString().c_str());
		if (state_put_down != actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_WARN("KCL (DropAction) failed to put down, but that is not catastrophic so proceeding with opening hand");
			// Call action to open hand
			kclhand_control::ActuateHandActionGoal open_hand_goal;
			open_hand_goal.goal.command = 0;
			open_hand_goal.goal.force_limit = 1.0;
			kclhandGraspActionClient.sendGoal(open_hand_goal.goal);
		
                	// bool finished_before_timeout = false;
                	kclhandGraspActionClient.waitForResult();
                	actionlib::SimpleClientGoalState state = kclhandGraspActionClient.getState();
                	ROS_INFO("KCL: (DropAction) action finished: %s", state.toString().c_str());
                
			if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
				success = false;
		}

		if (success){
			// Return arm to good location
			//put_down_goal.goal.destPoseSE2.pose.position.x = 0.30;
                	//put_down_goal.goal.destPoseSE2.pose.position.y = -0.515;
                	//put_down_goal.goal.destPoseSE2.pose.position.z = 0.585;
                	//put_down_goal.goal.destPoseSE2.pose.orientation.w = -0.408;
                	//put_down_goal.goal.destPoseSE2.pose.orientation.x = 0.896;
                	//put_down_goal.goal.destPoseSE2.pose.orientation.y = 0.154;
                	//put_down_goal.goal.destPoseSE2.pose.orientation.z = 0.088;
			//putDownActionClient.sendGoal(put_down_goal.goal);
                	//putDownActionClient.waitForResult();
                	//state_put_down = putDownActionClient.getState();
                	//ROS_INFO("KCL: (DropAction) action finished: %s", state_put_down.toString().c_str());
                	//if (state_put_down != actionlib::SimpleClientGoalState::SUCCEEDED) {
                        //	ROS_WARN("KCL (DropAction) failed to return, but that is not catastrophic so proceeding");
			//}			

			// gripper_empty fact
			rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
			knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			knowledge_update_service.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			knowledge_update_service.request.knowledge.attribute_name = "gripper_empty";
			diagnostic_msgs::KeyValue kv;
			kv.key = "v";
			kv.value = robotID;
			knowledge_update_service.request.knowledge.values.push_back(kv);
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (PerceptionAction) Could not add gripper_empty predicate to the knowledge base.");
			}

			// holding fact
			knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
			knowledge_update_service.request.knowledge.attribute_name = "holding";
			kv.key = "o";
			kv.value = objectID;
			knowledge_update_service.request.knowledge.values.push_back(kv);
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (PerceptionAction) Could not remove holding predicate from the knowledge base.");
			}

			// object_at fact
			knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			knowledge_update_service.request.knowledge.attribute_name = "object_at";
			knowledge_update_service.request.knowledge.values.clear();
			kv.key = "o";
			kv.value = objectID;
			knowledge_update_service.request.knowledge.values.push_back(kv);
			kv.key = "wp";
			kv.value = wpID;
			knowledge_update_service.request.knowledge.values.push_back(kv);
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (PerceptionAction) Could not remove object_at predicate to the knowledge base.");
			}
			
			return true;
		}
		return false;
	}


	/*
	 * blind grasp action dispatch
	 *	:parameters (?v - robot ?wp - waypoint ?o - object ?t - type)
	 *	:effect (and
	 *		(not (gripper_empty ?v))
	 *		(not (object_at ?o ?wp))
	 *		(holding ?v ?o)
	 */
	bool RPGraspAction::dispatchBlindGraspAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		ROS_INFO("KCL: (GraspAction) blind grasp action recieved");

		// get object ID from action dispatch
		std::string robotID;
		std::string objectID;
		bool foundObject = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			if(0==msg->parameters[i].key.compare("r"))
				robotID = msg->parameters[i].value;
			if(0==msg->parameters[i].key.compare("ob")) {
				objectID = msg->parameters[i].value;
				foundObject = true;
			}
		}
		if(!foundObject) {
			ROS_INFO("KCL: (GraspAction) aborting action dispatch; malformed parameters");
			return false;
		}
		
		// get details from message store
		std::vector< boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> > results;
		if(message_store.queryNamed<squirrel_object_perception_msgs::SceneObject>(objectID, results)) {
			if(results.size()<1) {
				ROS_INFO("KCL: (GraspAction) aborting action dispatch; no matching objectID %s", objectID.c_str());
				return false;
			}
			if(results.size()>1)
				ROS_INFO("KCL: (GraspAction) multiple objects share the same objectID");

			// get pose in odom frame
			geometry_msgs::PoseStamped poseMap;
			poseMap.header = results[0]->header;
			poseMap.pose = results[0]->pose;
/*			tf::TransformListener tfl;
			try {
				tfl.waitForTransform("/odom", "/map", ros::Time::now(), ros::Duration(20.0));
				tfl.transformPose("/odom", poseMap, pose);
			} catch ( tf::TransformException& ex ) {
				ROS_ERROR("%s: error while transforming point: %s", ros::this_node::getName().c_str(), ex.what());
				return false;
			}
*/
			// dispatch Grasp action
			squirrel_manipulation_msgs::BlindGraspGoal goal;
			goal.heap_center_pose = poseMap;
			goal.heap_bounding_cylinder = results[0]->bounding_cylinder;
			goal.heap_point_cloud = results[0]->cloud;
			blind_grasp_action_client.sendGoal(goal);
			
			// bool finished_before_timeout = false;
			blind_grasp_action_client.waitForResult();
			actionlib::SimpleClientGoalState state = blind_grasp_action_client.getState();
			ROS_INFO("KCL: (GraspAction) action finished: %s", state.toString().c_str());
		
/**
 * TODO For the sorting demo we assume that this always fails!
 */	
			if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
/*

				// gripper_empty fact (remove)
				rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
				knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
				knowledge_update_service.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				knowledge_update_service.request.knowledge.attribute_name = "gripper_empty";
				diagnostic_msgs::KeyValue kv;
				kv.key = "v";
				kv.value = robotID;
				knowledge_update_service.request.knowledge.values.push_back(kv);
				if (!update_knowledge_client.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (PerceptionAction) Could not remove gripper_empty predicate from the knowledge base.");
				}

				// holding fact	
				knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
				knowledge_update_service.request.knowledge.attribute_name = "holding";
				kv.key = "o";
				kv.value = objectID;
				knowledge_update_service.request.knowledge.values.push_back(kv);
				if (!update_knowledge_client.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (PerceptionAction) Could not add holding predicate to the knowledge base.");
				}

				// object_at fact (remove)
				knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
				knowledge_update_service.request.knowledge.attribute_name = "object_at";
				knowledge_update_service.request.knowledge.values.clear();
				kv.key = "o";
				kv.value = objectID;
				knowledge_update_service.request.knowledge.values.push_back(kv);
				if (!update_knowledge_client.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (PerceptionAction) Could not remove object_at predicate from the knowledge base.");
				}
*/
				// holding fact	
				rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
				knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
				knowledge_update_service.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				knowledge_update_service.request.knowledge.attribute_name = "holding";
				knowledge_update_service.request.knowledge.is_negative = true;

				diagnostic_msgs::KeyValue kv;
				kv.key = "v";
				kv.value = robotID;
				knowledge_update_service.request.knowledge.values.push_back(kv);
				kv.key = "o";
				kv.value = objectID;
				knowledge_update_service.request.knowledge.values.push_back(kv);
				if (!update_knowledge_client.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (PerceptionAction) Could not add (not (holding %s %s)) predicate to the knowledge base.", robotID.c_str(), objectID.c_str());
				}
			
				return true;

			} else return false;

		} else {
			ROS_INFO("KCL: (GraspAction) aborting action dispatch; query to sceneDB failed");
			return false;
		}
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_grasping");
		ros::NodeHandle nh;


		std::string GraspActionserver, blindGraspActionServer;
		nh.param("blind_grasp_action_server", blindGraspActionServer, std::string("/metahand_grasp_server"));

		// create PDDL action subscriber
		KCL_rosplan::RPGraspAction rpga(nh, blindGraspActionServer);
	
		// listen for action dispatch
		ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPGraspAction::dispatchCallback, &rpga);
		ROS_INFO("KCL: (GraspAction) Ready to receive");

		ros::spin();
		return 0;
	}
