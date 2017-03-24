#include <sstream>
#include <complex>
#include <limits>
#include <set>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_dispatch_msgs/ActionFeedback.h>
#include <rosplan_knowledge_msgs/KnowledgeQueryService.h>

#include "SimulatedObservePDDLAction.h"

namespace KCL_rosplan
{

SimulatedObservePDDLAction::SimulatedObservePDDLAction(ros::NodeHandle& node_handle)
	: message_store_(node_handle)
{
	// knowledge interface
	update_knowledge_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
	get_instance_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
	get_attribute_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
	action_feedback_pub_ = node_handle.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
	query_knowledge_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>("/kcl_rosplan/query_knowledge_base");

	// Subscribe to the action feedback topic.
	dispatch_sub_ = node_handle.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::SimulatedObservePDDLAction::dispatchCallback, this);
}

SimulatedObservePDDLAction::~SimulatedObservePDDLAction()
{
	
}

void SimulatedObservePDDLAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
{
	std::string normalised_action_name = msg->name;
	std::transform(normalised_action_name.begin(), normalised_action_name.end(), normalised_action_name.begin(), tolower);
	
	// Check if this action is to be handled by this class.
	if (normalised_action_name != "observe-has_commanded" &&
	    normalised_action_name != "observe-is_of_type" &&
	    normalised_action_name != "observe-holding" &&
	    normalised_action_name != "observe-sorting_done" &&
	    normalised_action_name != "observe-is_examined" &&
	    normalised_action_name != "observe-belongs_in" &&
	    normalised_action_name != "jump")
	{
		return;
	}
	
	ROS_INFO("KCL: (SimulatedObservePDDLAction) Process the action: %s", normalised_action_name.c_str());
	
	// Report this action is enabled and completed successfully.
	rosplan_dispatch_msgs::ActionFeedback fb;
	fb.action_id = msg->action_id;
	fb.status = "action enabled";
	action_feedback_pub_.publish(fb);
	
	if (normalised_action_name == "observe-sorting_done")
	{
		static int call_counter = 0;
		++call_counter;
		// Add the new knowledge.
		rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_item.attribute_name = "sorting_done";
		
//		float p = (float)rand() / (float)RAND_MAX;
//		ROS_INFO("KCL: (SimulatedObservePDDLAction) Done sorting? %f >= %f.", p, 0.5f);
//		knowledge_item.is_negative = p >= 0.5f;
		knowledge_item.is_negative = call_counter < 1;
		
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client_.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SimulatedObservePDDLAction) Could not add the sorting_done predicate to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SimulatedObservePDDLAction) Added %s (sorting_done) to the knowledge base.", knowledge_item.is_negative ? "NOT" : "");
		
		// Remove the opposite option from the knowledge base.
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
		knowledge_item.is_negative = !knowledge_item.is_negative;
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client_.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SimulatedObservePDDLAction) Could not remove the sorting_done predicate to the knowledge base.");
	// 		exit(-1);
		}
		ROS_INFO("KCL: (SimulatedObservePDDLAction) Removed %s (sorting_done) to the knowledge base.", knowledge_item.is_negative ? "NOT" : "");
		
		knowledge_item.values.clear();
	}
	else if (normalised_action_name == "observe-toy_at_right_box")
	{
		/**
		 * Get the location of the robot and find out which box is closest to it. 
		 * Then find all toys and check if all toys near the box are of the correct 
		 * type or already tidied.
		 */
		
		// Locate the location of the robot.
		tf::StampedTransform transform;
		tf::TransformListener tfl;
		try {
			tfl.waitForTransform("/map","/base_link", ros::Time::now(), ros::Duration(1.0));
			tfl.lookupTransform("/map", "/base_link", ros::Time(0), transform);
		} catch ( tf::TransformException& ex ) {
			ROS_ERROR("KCL: (SimulatedObservePDDLAction) Error find the transform between /map and /base_link.");
			fb.action_id = msg->action_id;
			fb.status = "action failed";
			action_feedback_pub_.publish(fb);
			return;
		}
		
		std::string closest_box;
		geometry_msgs::PoseStamped closest_box_pose;
		float min_distance_from_robot = std::numeric_limits<float>::max();
		
		// Get all boxes and their poses, pick the one that is closest.
		rosplan_knowledge_msgs::GetInstanceService getInstances;
		getInstances.request.type_name = "box";
		if (!get_instance_client_.call(getInstances)) {
			ROS_ERROR("KCL: (SimulatedObservePDDLAction) Failed to get all the box instances.");
			fb.action_id = msg->action_id;
			fb.status = "action failed";
			action_feedback_pub_.publish(fb);
			return;
		}
		ROS_INFO("TEST");
		ROS_INFO("KCL: (SimulatedObservePDDLAction) Received all the box instances testesttetst %zd.", getInstances.response.instances.size());
		for (std::vector<std::string>::const_iterator ci = getInstances.response.instances.begin(); ci != getInstances.response.instances.end(); ++ci)
		{
			// fetch position of the box from message store
			std::stringstream ss;
			ss << *ci << "_location";
			std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
			ROS_ERROR("KCL: (SimulatedObservePDDLAction) query the message store for %s.", ss.str().c_str());

			if(message_store_.queryNamed<geometry_msgs::PoseStamped>(*ci, results)) {
				if(results.empty()) {
					ROS_ERROR("KCL: (SimulatedObservePDDLAction) aborting waypoint request; no matching boxID %s", ss.str().c_str());
					fb.action_id = msg->action_id;
					fb.status = "action failed";
					action_feedback_pub_.publish(fb);
					return;
				}
				else
				{
					ROS_ERROR("KCL: (SimulatedObservePDDLAction) Found the box pose!");
				}
			} else {
				std::cout << ss.str() << std::endl;
				ROS_ERROR("KCL: (SimulatedObservePDDLAction) could not query message store to fetch box pose, why!? %s", ss.str().c_str());
				fb.action_id = msg->action_id;
				fb.status = "action failed";
				action_feedback_pub_.publish(fb);
				return;
			}

			// request manipulation waypoints for object
			geometry_msgs::PoseStamped &box_pose = *results[0];
			float distance = (box_pose.pose.position.x - transform.getOrigin().getX()) * (box_pose.pose.position.x - transform.getOrigin().getX()) +
			                 (box_pose.pose.position.x - transform.getOrigin().getX()) * (box_pose.pose.position.x - transform.getOrigin().getX());
			
			if (distance < min_distance_from_robot)
			{
				min_distance_from_robot = distance;
				closest_box = *ci;
				closest_box_pose = box_pose;
			}
		}
		
		// Get all toys and check if any of these toys are near the box and not tidied.
		getInstances.request.type_name = "object";
		if (!get_instance_client_.call(getInstances)) {
			ROS_ERROR("KCL: (SimulatedObservePDDLAction) Failed to get all the object instances.");
			fb.action_id = msg->action_id;
			fb.status = "action failed";
			action_feedback_pub_.publish(fb);
			return;
		}
		ROS_INFO("KCL: (SimulatedObservePDDLAction) Received all the object instances.");
		
		// Fetch all the objects.
		rosplan_knowledge_msgs::GetAttributeService get_attribute;
		get_attribute.request.predicate_name = "tidy";
		if (!get_attribute_client_.call(get_attribute)) {
			ROS_ERROR("KCL: (ExamineAreaPDDLAction) Failed to recieve the attributes of the predicate 'tidy'");
			fb.action_id = msg->action_id;
			fb.status = "action failed";
			action_feedback_pub_.publish(fb);
			return;
		}
		
		std::set<std::string> tidied_objects;
		for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = get_attribute.response.attributes.begin(); ci != get_attribute.response.attributes.end(); ++ci)
		{
			const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
			for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci) {
				const diagnostic_msgs::KeyValue& key_value = *ci;
				if ("o" == key_value.key) {
					tidied_objects.insert(key_value.value);
				}
			}
		}
		
		for (std::vector<std::string>::const_iterator ci = getInstances.response.instances.begin(); ci != getInstances.response.instances.end(); ++ci)
		{
			const std::string& object_name = *ci;
			
			if (tidied_objects.find(object_name) != tidied_objects.end())
			{
				ROS_INFO("KCL: (SimulatedObservePDDLAction) Object %s has already been tidied, ignore.", object_name.c_str());
				continue;
			}
			
			// fetch position of the box from message store
			std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
			if(message_store_.queryNamed<geometry_msgs::PoseStamped>(*ci, results)) {
				if(results.size()<1) {
					ROS_ERROR("KCL: (SimulatedObservePDDLAction) aborting waypoint request; no matching object %s", (*ci).c_str());
					fb.action_id = msg->action_id;
					fb.status = "action failed";
					action_feedback_pub_.publish(fb);
					return;
				}
			} else {
				ROS_ERROR("KCL: (SimulatedObservePDDLAction) could not query message store to fetch object pose");
				fb.action_id = msg->action_id;
				fb.status = "action failed";
				action_feedback_pub_.publish(fb);
				return;
			}

			// check whether it is close enough to the box we are interested in to be relevant.
			geometry_msgs::PoseStamped &object_pose = *results[0];
			if ((object_pose.pose.position.x - closest_box_pose.pose.position.x) * (object_pose.pose.position.x - closest_box_pose.pose.position.x) -
			    (object_pose.pose.position.y - closest_box_pose.pose.position.y) * (object_pose.pose.position.y - closest_box_pose.pose.position.y) < 1.5f)
			{
				// Check if the untidied toy belong in this box.
				rosplan_knowledge_msgs::KnowledgeQueryService knowledge_query;
	
				rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
				knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				knowledge_item.attribute_name = "belongs_in";
				
				diagnostic_msgs::KeyValue kv;
				kv.key = "o";
				kv.value = object_name;
				knowledge_item.values.push_back(kv);
				
				kv.key = "b";
				kv.value = closest_box;
				knowledge_item.values.push_back(kv);
				
				knowledge_query.request.knowledge.push_back(knowledge_item);
				
				// Check if any of these facts are true.
				if (!query_knowledge_client_.call(knowledge_query))
				{
					ROS_ERROR("KCL: (SimulatedObservePDDLAction) Could not call the query knowledge server.");
					exit(1);
				}
				knowledge_item.values.clear();
				
				// Add the new knowledge.
				rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
				knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
				
				knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				knowledge_item.attribute_name = "toy_at_right_box";
				
				knowledge_item.is_negative = knowledge_query.response.results[0] == 0;
				
				knowledge_update_service.request.knowledge = knowledge_item;
				if (!update_knowledge_client_.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (SimulatedObservePDDLAction) Could not add the toy_at_right_box predicate to the knowledge base.");
					exit(-1);
				}
				ROS_INFO("KCL: (SimulatedObservePDDLAction) Added %s (toy_at_right_box) to the knowledge base.", knowledge_item.is_negative ? "NOT" : "");
				
				// Remove the opposite option from the knowledge base.
				knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
				knowledge_item.is_negative = !knowledge_item.is_negative;
				knowledge_update_service.request.knowledge = knowledge_item;
				if (!update_knowledge_client_.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (SimulatedObservePDDLAction) Could not remove the toy_at_right_box predicate to the knowledge base.");
			// 		exit(-1);
				}
				ROS_INFO("KCL: (SimulatedObservePDDLAction) Removed %s (toy_at_right_box) to the knowledge base.", knowledge_item.is_negative ? "NOT" : "");
				
				knowledge_item.values.clear();
			}
		}
	}
	
	fb.action_id = msg->action_id;
	fb.status = "action achieved";
	action_feedback_pub_.publish(fb);
}

};
