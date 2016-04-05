#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>
#include <actionlib/client/simple_action_client.h>
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "squirrel_object_perception_msgs/LookForObjectsAction.h"
#include "squirrel_object_perception_msgs/FindDynamicObjects.h"
#include "squirrel_interface_perception/RPPerceptionAction.h"
#include "squirrel_planning_knowledge_msgs/AddObjectService.h"
#include "mongodb_store/message_store.h"
#include "geometry_msgs/PoseStamped.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"

/* The implementation of RPMoveBase.h */
namespace KCL_rosplan {

	/* constructor */
	RPPerceptionAction::RPPerceptionAction(ros::NodeHandle &nh, std::string &actionserver)
	 : message_store(nh), examine_action_client(actionserver, true) {

		// create the action clients
		ROS_INFO("KCL: (PerceptionAction) waiting for action server to start on %s", actionserver.c_str());
		examine_action_client.waitForServer();
		
		// create the action feedback publisher
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		find_dynamic_objects_client = nh.serviceClient<squirrel_object_perception_msgs::FindDynamicObjects>("/squirrel_find_dynamic_objects");
	}

	/* action dispatch callback; parameters (?v - robot ?wp - waypoint) */
	void RPPerceptionAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// ignore non-perception actions
		if(0==msg->name.compare("explore_waypoint")) exploreAction(msg);
		if(0==msg->name.compare("observe-classifiable_from")) examineAction(msg);

	}

	
	/* action dispatch callback; explore action */
	void RPPerceptionAction::exploreAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		ROS_INFO("KCL: (PerceptionAction) explore action recieved");

		// get waypoint ID from action dispatch
		std::string explored_waypoint;
		bool foundWP = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			if(0==msg->parameters[i].key.compare("view")) {
				explored_waypoint = msg->parameters[i].value;
				foundWP = true;
			}
		}
		if(!foundWP) {
			ROS_INFO("KCL: (PerceptionAction) aborting action dispatch; malformed parameters");
			return;
		}

		// report this action is enabled
		publishFeedback(msg->action_id,"action enabled");

		// find dynamic objects
		squirrel_object_perception_msgs::FindDynamicObjects fdSrv;
		if (!find_dynamic_objects_client.call(fdSrv)) {
			ROS_ERROR("KCL: (PerceptionAction) Could not call the find_dyamic_objects service.");
		}

		// add all new objects
		std::vector<squirrel_object_perception_msgs::SceneObject>::const_iterator ci = fdSrv.response.dynamic_objects_added.begin();
		for (; ci != fdSrv.response.dynamic_objects_added.end(); ++ci) {

			squirrel_object_perception_msgs::SceneObject so = (*ci);
			std::stringstream wpid;
			wpid << "waypoint_" << so.id << std::endl;
			std::string wpName(wpid.str());

			// add the new object
			rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
			knowledge_update_service.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			knowledge_update_service.request.knowledge.instance_type = "object";
			knowledge_update_service.request.knowledge.instance_name = so.id;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (PerceptionAction) Could not add the object %s to the knowledge base.", so.id.c_str());
			}

			// add the new object's waypoint
			knowledge_update_service.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			knowledge_update_service.request.knowledge.instance_type = "waypoint";
			knowledge_update_service.request.knowledge.instance_name = wpName;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (PerceptionAction) Could not add the object %s to the knowledge base.", wpName.c_str());
			}

			// object_at fact	
			knowledge_update_service.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			knowledge_update_service.request.knowledge.attribute_name = "object_at";
			diagnostic_msgs::KeyValue kv;
			kv.key = "o";
			kv.value = so.id;
			knowledge_update_service.request.knowledge.values.push_back(kv);
			kv.key = "wp";
			kv.value = wpName;
			knowledge_update_service.request.knowledge.values.push_back(kv);
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (PerceptionAction) Could not add object_at predicate to the knowledge base.");
			}

			// is_of_type fact	
			knowledge_update_service.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			knowledge_update_service.request.knowledge.attribute_name = "is_of_type";
			knowledge_update_service.request.knowledge.values.pop_back();
			kv.key = "t";
			kv.value = "unknown";
			knowledge_update_service.request.knowledge.values.push_back(kv);
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (PerceptionAction) Could not add is_of_type predicate to the knowledge base.");
			}

			//data
			db_name_map[wpName] = message_store.insertNamed(wpName, ci->pose);
			db_name_map[so.id] = message_store.insertNamed(so.id, so);
		}

		// update all new objects
		ci = fdSrv.response.dynamic_objects_updated.begin();
		for (; ci != fdSrv.response.dynamic_objects_updated.end(); ++ci) {

			squirrel_object_perception_msgs::SceneObject so = (*ci);
			std::stringstream wpid;
			wpid << "waypoint_" << so.id << std::endl;
			std::string wpName(wpid.str());

			//data
			db_name_map[wpName] = message_store.updateNamed(wpName, ci->pose);
			db_name_map[so.id] = message_store.updateNamed(so.id, so);
		}

		// remove ghost objects
		ci = fdSrv.response.dynamic_objects_removed.begin();
		for (; ci != fdSrv.response.dynamic_objects_removed.end(); ++ci) {

			squirrel_object_perception_msgs::SceneObject so = (*ci);
			std::stringstream wpid;
			wpid << "waypoint_" << so.id << std::endl;
			std::string wpName(wpid.str());

			rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
			updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
			updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			updateSrv.request.knowledge.instance_type = "waypoint";
			updateSrv.request.knowledge.instance_name = wpName;
			update_knowledge_client.call(updateSrv);

			updateSrv.request.knowledge.instance_type = "object";
			updateSrv.request.knowledge.instance_name = so.id;
			update_knowledge_client.call(updateSrv);

			//data
			message_store.deleteID(db_name_map[wpName]);
			message_store.deleteID(db_name_map[so.id]);
		}

		// add the new knowledge
		rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		rosplan_knowledge_msgs::KnowledgeItem knowledge;
		knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge.attribute_name = "explored";
		diagnostic_msgs::KeyValue kv;
		kv.key = "wp";
		kv.value = explored_waypoint;
		knowledge.values.push_back(kv);

		knowledge_update_service.request.knowledge = knowledge;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (PerceptionAction) Could not add the explored predicate to the knowledge base.");
		}

		// report this action is achieved
		publishFeedback(msg->action_id,"action achieved");
	}

	/* action dispatch callback; examine action (?from ?view - waypoint ?o - object) */
	void RPPerceptionAction::examineAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		ROS_INFO("KCL: (PerceptionAction) explore action recieved");

		// get waypoint ID from action dispatch
		std::string explored_waypoint;
		bool foundWP = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			if(0==msg->parameters[i].key.compare("view")) {
				explored_waypoint = msg->parameters[i].value;
				foundWP = true;
			}
		}
		if(!foundWP) {
			ROS_INFO("KCL: (PerceptionAction) aborting action dispatch; malformed parameters");
			return;
		}

		// publish feedback (enabled)
		publishFeedback(msg->action_id,"action enabled");

		// dispatch Perception action
		squirrel_object_perception_msgs::LookForObjectsGoal perceptionGoal;
		perceptionGoal.look_for_object = squirrel_object_perception_msgs::LookForObjectsGoal::EXPLORE;
		examine_action_client.sendGoal(perceptionGoal);

		bool finished_before_timeout = examine_action_client.waitForResult(ros::Duration(msg->duration));
		actionlib::SimpleClientGoalState state = examine_action_client.getState();
		bool success = (finished_before_timeout);
		ROS_INFO("KCL: (PerceptionAction) observe finished: %s", state.toString().c_str());

		if (success) {
			ROS_INFO("KCL: (PerceptionAction) action complete");
			publishFeedback(msg->action_id, "action achieved");
		} else {
			ROS_INFO("KCL: (PerceptionAction) action timed out");
			publishFeedback(msg->action_id, "action failed");
		}
	}

	void RPPerceptionAction::publishFeedback(int action_id, std::string feedback) {
		// publish feedback
		rosplan_dispatch_msgs::ActionFeedback fb;
		fb.action_id = action_id;
		fb.status = feedback;
		action_feedback_pub.publish(fb);
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_perception");
		ros::NodeHandle nh;

		std::string actionserver;
		nh.param("action_server", actionserver, std::string("/look_for_objects"));

		// create PDDL action subscriber
		KCL_rosplan::RPPerceptionAction rppa(nh, actionserver);
	
		// listen for action dispatch
		ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPPerceptionAction::dispatchCallback, &rppa);
		ROS_INFO("KCL: (PerceptionAction) Ready to receive");

		ros::spin();
		return 0;
	}
