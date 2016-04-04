#include <std_msgs/Int8.h>

#include <map>
#include <algorithm>
#include <string>
#include <sstream>

#include "squirrel_planning_execution/RPSquirrelRecursion.h"
#include "squirrel_planning_execution/ContingentStrategicClassifyPDDLGenerator.h"
#include "squirrel_planning_execution/ContingentTacticalClassifyPDDLGenerator.h"
#include "squirrel_planning_execution/ContingentTidyPDDLGenerator.h"
#include "squirrel_planning_execution/ViewConeGenerator.h"
#include <squirrel_planning_execution/ClassicalTidyPDDLGenerator.h>
#include "pddl_actions/PlannerInstance.h"
#include "pddl_actions/GotoPDDLAction.h"
#include "pddl_actions/ExploreWaypointPDDLAction.h"
#include "pddl_actions/ClearObjectPDDLAction.h"
#include "pddl_actions/ClassifyObjectPDDLAction.h"
#include "pddl_actions/PutObjectInBoxPDDLAction.h"
#include "pddl_actions/TidyObjectPDDLAction.h"
#include "pddl_actions/PickupPDDLAction.h"
#include "pddl_actions/ShedKnowledgePDDLAction.h"

#define SQUIRREL_SIMULATION


/* The implementation of RPSquirrelRecursion.h */
namespace KCL_rosplan {

	/*-------------*/
	/* constructor */
	/*-------------*/

	RPSquirrelRecursion::RPSquirrelRecursion(ros::NodeHandle &nh)
		: node_handle(&nh), message_store(nh), initial_problem_generated(false)
	{
		// knowledge interface
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		query_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>("/kcl_rosplan/query_knowledge_base");
		
		// create the action feedback publisher
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
		
		get_instance_client = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
		get_attribute_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
		
		std::string classifyTopic("/squirrel_perception_examine_waypoint");
		nh.param("squirrel_perception_classify_waypoint_service_topic", classifyTopic, classifyTopic);
		classify_object_waypoint_client = nh.serviceClient<squirrel_waypoint_msgs::ExamineWaypoint>(classifyTopic);
		
		pddl_generation_service = nh.advertiseService("/kcl_rosplan/generate_planning_problem", &KCL_rosplan::RPSquirrelRecursion::generatePDDLProblemFile, this);

#ifndef SQUIRREL_SIMULATION
		std::string occupancyTopic("/squirrel_nav/occupancy_map");
		nh.param("occupancy_topic", occupancyTopic, occupancyTopic);
		view_cone_generator = new ViewConeGenerator(nh, occupancyTopic);
#else
		setupSimulation();
#endif
		/*
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id = "map";
		pose.pose.position.x = 1;
		pose.pose.position.y = 2;
		pose.pose.position.z = 0.0;
		pose.pose.orientation.x = 0.0;
		pose.pose.orientation.y = 0.0;
		pose.pose.orientation.z = 0.0;
		pose.pose.orientation.w = 1.0;
		std::string id(message_store.insertNamed("teddybeer", pose));
		//ros::spinOnce();
		
		generateInitialState();
		*/
		// { "_id" : ObjectId("56a64dab82b5af8506124f35"), "header" : { "stamp" : { "secs" : 0, "nsecs" : 0 }, "frame_id" : "map", "seq" : 0 }, "pose" : { "position" : { "y" : 2, "x" : 1, "z" : 0 }, "orientation" : { "y" : 0, "x" : 0, "z" : 0, "w" : 1 } }, "_meta" : { "stored_type" : "geometry_msgs/PoseStamped", "inserted_by" : "/rosplan_interface_mapping", "stored_class" : "geometry_msgs.msg._PoseStamped.PoseStamped", "name" : "teddybeer", "inserted_at" : ISODate("1970-01-01T00:00:30.476Z") } }
		// { "_id" : ObjectId("56a64f691d41c83466e349f1"), "header" : { "stamp" : { "secs" : 0, "nsecs" : 0 }, "frame_id" : "map", "seq" : 0 }, "pose" : { "position" : { "y" : 2, "x" : 1, "z" : 0 }, "orientation" : { "y" : 0, "x" : 0, "z" : 0, "w" : 1 } }, "_meta" : { "stored_type" : "geometry_msgs/PoseStamped", "inserted_by" : "/squirrel_interface_recursion", "stored_class" : "geometry_msgs.msg._PoseStamped.PoseStamped", "name" : "teddybeer", "inserted_at" : ISODate("2016-01-25T16:38:01.392Z") } }
	}
	
	void RPSquirrelRecursion::setupSimulation()
	{
		// We will make some fictional objects and associated waypoints.
		rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		
		// Create some types for the toys.
		std::vector<std::string> toy_types;
		toy_types.push_back("horse");
		toy_types.push_back("car");
		toy_types.push_back("unknown");
		
		for (std::vector<std::string>::const_iterator ci = toy_types.begin(); ci != toy_types.end(); ++ci)
		{
			const std::string& type_predicate = *ci;
			rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			knowledge_item.instance_type = "type";
			knowledge_item.instance_name = type_predicate;
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the type %s to the knowledge base.", type_predicate.c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (RPSquirrelRecursion) Added %s to the knowledge base.", type_predicate.c_str());
			
			// Create a box for each type.
			knowledge_item.instance_type = "box";
			std::stringstream ss;
			ss << *ci << "_box";
			std::string box_name = ss.str();
			knowledge_item.instance_name = box_name;
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the box %s to the knowledge base.", box_name.c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (RPSquirrelRecursion) Added %s to the knowledge base.", box_name.c_str());
			
			// Add the box constraints.
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			knowledge_item.attribute_name = "can_fit_inside";
			knowledge_item.is_negative = false;
			
			diagnostic_msgs::KeyValue kv;
			kv.key = "t";
			kv.value = type_predicate;
			knowledge_item.values.push_back(kv);
			
			kv.key = "b";
			kv.value = box_name;
			knowledge_item.values.push_back(kv);
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the fact (cat_fit_inside %s %s) to the knowledge base.", type_predicate.c_str(), box_name.c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (RPSquirrelRecursion) Added the fact (cat_fit_inside %s %s) to the knowledge base.", type_predicate.c_str(), box_name.c_str());
			
			// Place the boxes at their own waypoints.
			knowledge_item.values.clear();
			
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			knowledge_item.instance_type = "waypoint";
			ss << "_waypoint";
			knowledge_item.instance_name = ss.str();
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the waypoint %s to the knowledge base.", ss.str().c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (RPSquirrelRecursion) Added %s to the knowledge base.", ss.str().c_str());
			
			// Link the boxes to these waypoints.
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			knowledge_item.attribute_name = "box_at";
			knowledge_item.is_negative = false;
			
			kv.key = "b";
			kv.value = box_name;
			knowledge_item.values.push_back(kv);
			
			kv.key = "wp";
			kv.value = ss.str();
			knowledge_item.values.push_back(kv);
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the fact (box_at %s %s) to the knowledge base.", box_name.c_str(), ss.str().c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (RPSquirrelRecursion) Added the fact (box_at %s %s) to the knowledge base.", box_name.c_str(), ss.str().c_str());
			knowledge_item.values.clear();
			
			// Make sure the robots can pickup all types.
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			knowledge_item.attribute_name = "can_pickup";
			knowledge_item.is_negative = false;
			
			kv.key = "r";
			kv.value = "kenny";
			knowledge_item.values.push_back(kv);
			
			kv.key = "t";
			kv.value = type_predicate;
			knowledge_item.values.push_back(kv);
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the fact (can_pickup %s %s) to the knowledge base.", "kenny", type_predicate.c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (RPSquirrelRecursion) Added the fact (can_pickup %s %s) to the knowledge base.", "kenny", type_predicate.c_str());
			knowledge_item.values.clear();
		}
		/*
		for (unsigned int i = 0; i < 2; ++i)
		{
			std::stringstream ss;
			ss << "object" << i;
			std::string object_name = ss.str();
			
			ss.str(std::string());
			ss << "waypoint_object" << i;
			std::string waypoint_name = ss.str();
			
			rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			knowledge_item.instance_type = "object";
			knowledge_item.instance_name = object_name;
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the object %s to the knowledge base.", object_name.c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (RPSquirrelRecursion) Added %s to the knowledge base.", object_name.c_str());
			
			knowledge_item.instance_type = "waypoint";
			knowledge_item.instance_name = waypoint_name;
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the waypoint %s to the knowledge base.", waypoint_name.c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (RPSquirrelRecursion) Added %s to the knowledge base.", waypoint_name.c_str());
			
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			knowledge_item.attribute_name = "object_at";
			knowledge_item.is_negative = false;
			
			diagnostic_msgs::KeyValue kv;
			kv.key = "o";
			kv.value = object_name;
			knowledge_item.values.push_back(kv);
			
			kv.key = "wp";
			kv.value = waypoint_name;
			knowledge_item.values.push_back(kv);
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the new object_at predicate to the knowledge base.");
				exit(-1);
			}
			ROS_INFO("KCL: (RPSquirrelRecursion) Added the new object_at predicate to the knowledge base.");
		}
		*/
	}

	/*---------------------------*/
	/* strategic action callback */
	/*---------------------------*/

	void RPSquirrelRecursion::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		rosplan_dispatch_msgs::ActionDispatch normalised_action_dispatch = *msg;
		std::string action_name = msg->name;
		std::transform(action_name.begin(), action_name.end(), action_name.begin(), tolower);
		normalised_action_dispatch.name = action_name;
		
		ROS_INFO("KCL: (RPSquirrelRecursion) Action received %s", msg->name.c_str());
		
		// ignore actions
		if("observe-classifiable_on_attempt" != action_name &&
		   "examine_area" != action_name &&
		   "explore_area" != action_name &&
		   "tidy_area" != action_name)
		{
			return;
		}

		bool actionAchieved = false;
		last_received_msg.push_back(normalised_action_dispatch);
		
		ROS_INFO("KCL: (RPSquirrelRecursion) action recieved %s", action_name.c_str());
		
		PlannerInstance& planner_instance = PlannerInstance::createInstance(*node_handle);
		
		// Lets start the planning process.
		std::string data_path;
		node_handle->getParam("/data_path", data_path);
		
		std::string planner_path;
		node_handle->getParam("/planner_path", planner_path);
		
		std::stringstream ss;
		ss << data_path << action_name << "_domain-nt.pddl";
		std::string domain_name = ss.str();
		
		ss.str(std::string());
		ss << data_path << action_name << "_problem.pddl";
		std::string problem_name = ss.str();
		
		ss.str(std::string());
		ss << "timeout 60 " << planner_path << "ff -o DOMAIN -f PROBLEM";
		std::string planner_command = ss.str();
		
		// Before calling the planner we create the domain so it can be parsed.
		if (!createDomain(action_name))
		{
			ROS_ERROR("KCL: (RPSquirrelRecursion) failed to produce a domain at %s for action name %s.", domain_name.c_str(), action_name.c_str());
			return;
		}
		
		planner_instance.startPlanner(domain_name, problem_name, data_path, planner_command);
		
		// publish feedback (enabled)
		rosplan_dispatch_msgs::ActionFeedback fb;
		fb.action_id = msg->action_id;
		fb.status = "action enabled";
		action_feedback_pub.publish(fb);

		// wait for action to finish
		ros::Rate loop_rate(1);
		while (ros::ok() && (planner_instance.getState() == actionlib::SimpleClientGoalState::ACTIVE || planner_instance.getState() == actionlib::SimpleClientGoalState::PENDING)) {
			ros::spinOnce();
			loop_rate.sleep();
		}

		actionlib::SimpleClientGoalState state = planner_instance.getState();
		ROS_INFO("KCL: (RPSquirrelRecursion) action finished: %s, %s", action_name.c_str(), state.toString().c_str());

		if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
			
			// Update the knowledge base with what has been achieved.
			if ("explore_area" == action_name)
			{
				// Update the domain.
				const std::string& robot = msg->parameters[0].value;
				const std::string& area = msg->parameters[1].value;
				
				ROS_INFO("KCL (RPSquirrelRecursion) Process the action: %s, Explore %s by %s", action_name.c_str(), area.c_str(), robot.c_str());
				
				// Remove the old knowledge.
				rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
				knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
				rosplan_knowledge_msgs::KnowledgeItem kenny_knowledge;
				kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				kenny_knowledge.attribute_name = "explored";
				kenny_knowledge.is_negative = false;
				
				diagnostic_msgs::KeyValue kv;
				kv.key = "a";
				kv.value = area;
				kenny_knowledge.values.push_back(kv);
				
				knowledge_update_service.request.knowledge = kenny_knowledge;
				if (!update_knowledge_client.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the (explored %s) predicate to the knowledge base.", area.c_str());
					exit(-1);
				}
				ROS_INFO("KCL: (RPSquirrelRecursion) Added the (explored %s) predicate to the knowledge base.", area.c_str());
				kenny_knowledge.values.clear();
			} else if ("examine_area" == action_name) {
				// Update the domain.
				const std::string& robot = msg->parameters[0].value;
				const std::string& area = msg->parameters[1].value;
				
				ROS_INFO("KCL (RPSquirrelRecursion) Process the action: %s, Examine %s by %s", action_name.c_str(), area.c_str(), robot.c_str());
				
				// Remove the old knowledge.
				rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
				knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
				rosplan_knowledge_msgs::KnowledgeItem kenny_knowledge;
				kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				kenny_knowledge.attribute_name = "examined";
				kenny_knowledge.is_negative = false;
				
				diagnostic_msgs::KeyValue kv;
				kv.key = "a";
				kv.value = area;
				kenny_knowledge.values.push_back(kv);
				
				knowledge_update_service.request.knowledge = kenny_knowledge;
				if (!update_knowledge_client.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the (examined %s) predicate to the knowledge base.", area.c_str());
					exit(-1);
				}
				ROS_INFO("KCL: (RPSquirrelRecursion) Added the action (examined %s) predicate to the knowledge base.", area.c_str());
				kenny_knowledge.values.clear();
			} else if ("observe-classifiable_on_attempt" == action_name) {
				// Update the domain.
				const std::string& object = msg->parameters[0].value;
				const std::string& counter = msg->parameters[1].value;
				
				// Add the new knowledge.
				rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
				knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
				rosplan_knowledge_msgs::KnowledgeItem kenny_knowledge;
				kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				kenny_knowledge.attribute_name = "classifiable_on_attempt";
				
				// Check if this object has been classified or not.
				rosplan_knowledge_msgs::GetInstanceService get_instance;
				get_instance.request.type_name = "waypoint";
				
				if (!get_instance_client.call(get_instance))
				{
					ROS_ERROR("KCL: (RPSquirrelRecursion) Could not get the instances of type 'waypoint'.");
					exit(1);
				}
				
				rosplan_knowledge_msgs::KnowledgeQueryService knowledge_query;
				
				// Find if this object has been classified at any location.
				for (std::vector<std::string>::const_iterator ci = get_instance.response.instances.begin(); ci != get_instance.response.instances.end(); ++ci)
				{
					const std::string& wp1 = *ci;
					for (std::vector<std::string>::const_iterator ci = get_instance.response.instances.begin(); ci != get_instance.response.instances.end(); ++ci)
					{
						const std::string& wp2 = *ci;
						
						rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
						knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
						knowledge_item.attribute_name = "classifiable_from";
						
						diagnostic_msgs::KeyValue kv;
						kv.key = "from";
						kv.value = wp1;
						knowledge_item.values.push_back(kv);
						
						kv.key = "view";
						kv.value = wp2;
						knowledge_item.values.push_back(kv);
						
						kv.key = "o";
						kv.value = object;
						knowledge_item.values.push_back(kv);
						knowledge_item.is_negative = false;
						
						knowledge_query.request.knowledge.push_back(knowledge_item);
					}
				}
				
				// Check if any of these facts are true.
				if (!query_knowledge_client.call(knowledge_query))
				{
					ROS_ERROR("KCL: (RPSquirrelRecursion) Could not call the query knowledge server.");
					exit(1);
				}
				
				bool object_is_classified = false;
				for (std::vector<unsigned char>::const_iterator ci = knowledge_query.response.results.begin(); ci != knowledge_query.response.results.end(); ci++)
				{
					if (*ci == 1)
					{
						object_is_classified = true;
						break;
					}
				}
				
				kenny_knowledge.is_negative = !object_is_classified;
				if (kenny_knowledge.is_negative)
				{
					ROS_INFO("KCL: (RPSquirrelRecursion) %s was not classified on the %sth attempt.", object.c_str(), counter.c_str());
				}
				else
				{
					ROS_INFO("KCL: (RPSquirrelRecursion) %s was classified on the %sth attempt!", object.c_str(), counter.c_str());
				}
				
				diagnostic_msgs::KeyValue kv;
				kv.key = "o";
				kv.value = object;
				kenny_knowledge.values.push_back(kv);
				
				kv.key = "c";
				kv.value = counter;
				kenny_knowledge.values.push_back(kv);
				
				knowledge_update_service.request.knowledge = kenny_knowledge;
				if (!update_knowledge_client.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (ClassifyObjectPDDLAction) Could not add the classifiable_on_attempt predicate to the knowledge base.");
					exit(-1);
				}
				ROS_INFO("KCL: (ClassifyObjectPDDLAction) Added classifiable_on_attempt predicate to the knowledge base.");
				kenny_knowledge.values.clear();
			}
			
			// publish feedback (achieved)
			rosplan_dispatch_msgs::ActionFeedback fb;
			fb.action_id = msg->action_id;
			fb.status = "action achieved";
			action_feedback_pub.publish(fb);
		} else {
			// publish feedback (failed)
			rosplan_dispatch_msgs::ActionFeedback fb;
			fb.action_id = msg->action_id;
			fb.status = "action failed";
			action_feedback_pub.publish(fb);
		}

		last_received_msg.pop_back();
	}
	
	/*--------------------*/
	/* problem generation */
	/*--------------------*/

	/**
	 * Generate a contingent problem
	 */
	bool RPSquirrelRecursion::generatePDDLProblemFile(rosplan_knowledge_msgs::GenerateProblemService::Request &req, rosplan_knowledge_msgs::GenerateProblemService::Response &res) {
		
		ROS_INFO("KCL: (RPSquirrelRecursion) generatePDDLProblemFile: %s", req.problem_path.c_str());
		
		// Lets start the planning process.
		std::string data_path;
		node_handle->getParam("/data_path", data_path);
		
		/**
		 * If no message has been received yet we setup the initial condition.
		 */
		if (last_received_msg.empty() && !initial_problem_generated) {
			ROS_INFO("KCL: (RPSquirrelRecursion) Create the initial problem.");
			
			std::stringstream domain_ss;
			domain_ss << data_path << "tidy_room_domain-nt.pddl";
			std::string domain_name = domain_ss.str();
			
			generateInitialState();
			PlanningEnvironment planning_environment;
			planning_environment.parseDomain(domain_name);
			planning_environment.update(*node_handle);
			PDDLProblemGenerator pddl_problem_generator;
			
			pddl_problem_generator.generatePDDLProblemFile(planning_environment, req.problem_path);
			initial_problem_generated = true;
			return true;
		}
		
		else if (last_received_msg.empty())
		{
			ROS_INFO("KCL: (RPSquirrelRecursion) No messages received...");
			return false;
		}
		
		// All domains / problem files should all ready been generated.
		return true;
	}
	
	void RPSquirrelRecursion::generateInitialState()
	{
		// Add kenny
		rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
		knowledge_item.instance_type = "robot";
		knowledge_item.instance_name = "kenny";
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add kenny to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (RPSquirrelRecursion) Added kenny to the knowledge base.");
		
		// Add the single room.
		knowledge_item.instance_type = "area";
		knowledge_item.instance_name = "room";
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add area to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (RPSquirrelRecursion) Added area to the knowledge base.");
		
		// Set the location of the robot.
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_item.attribute_name = "robot_in";
		knowledge_item.is_negative = false;
		diagnostic_msgs::KeyValue kv;
		kv.key = "r";
		kv.value = "kenny";
		knowledge_item.values.push_back(kv);
		kv.key = "a";
		kv.value = "room";
		knowledge_item.values.push_back(kv);
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the fact (robot_in kenny room) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (RPSquirrelRecursion) Added (robot_in kenny room) to the knowledge base.");
		knowledge_item.values.clear();
		
		// Setup the goal.
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_item.attribute_name = "tidy";
		knowledge_item.is_negative = false;
		kv.key = "a";
		kv.value = "room";
		knowledge_item.values.push_back(kv);
		
		
		
		// Add the goal.
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the goal (tidy room) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (RPSquirrelRecursion) Added the goal (tidy room) to the knowledge base.");
	}
	
	bool RPSquirrelRecursion::createDomain(const std::string& action_name)
	{
		// Lets start the planning process.
		std::string data_path;
		node_handle->getParam("/data_path", data_path);

		std::stringstream ss;

		ss << last_received_msg.back().name << "_domain-nt.pddl";
		std::string domain_name = ss.str();
		ss.str(std::string());

		ss << data_path << domain_name;
		std::string domain_path = ss.str();		
		ss.str(std::string());

		ss << last_received_msg.back().name << "_problem.pddl";
		std::string problem_name = ss.str();
		ss.str(std::string());

		ss << data_path << problem_name;
		std::string problem_path = ss.str();
		ss.str(std::string());
		
 		if (action_name == "explore_area") {
			
			std::vector<geometry_msgs::Pose> view_poses;
#ifndef SQUIRREL_SIMULATION
			view_cone_generator->createViewCones(view_poses, 10, 5, 30.0f, 2.0f, 100, 0.5f);
#else
			view_poses.push_back(geometry_msgs::Pose());
			view_poses.push_back(geometry_msgs::Pose());
			view_poses.push_back(geometry_msgs::Pose());
			view_poses.push_back(geometry_msgs::Pose());
#endif 
			
			// Add these poses to the knowledge base.
			rosplan_knowledge_msgs::KnowledgeUpdateService add_waypoints_service;
			add_waypoints_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			
			unsigned int waypoint_number = 0;
			std::stringstream ss;
			for (std::vector<geometry_msgs::Pose>::const_iterator ci = view_poses.begin(); ci != view_poses.end(); ++ci) {
				
				ss.str(std::string());
				ss << "explore_wp" << waypoint_number;
				rosplan_knowledge_msgs::KnowledgeItem waypoint_knowledge;
				add_waypoints_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
				waypoint_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
				waypoint_knowledge.instance_type = "waypoint";
				waypoint_knowledge.instance_name = ss.str();
				add_waypoints_service.request.knowledge = waypoint_knowledge;
				if (!update_knowledge_client.call(add_waypoints_service)) {
					ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add an explore wayoint to the knowledge base.");
					exit(-1);
				}
				
				// Setup the goal.
				waypoint_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				waypoint_knowledge.attribute_name = "explored";
				waypoint_knowledge.is_negative = false;
				diagnostic_msgs::KeyValue kv;
				kv.key = "wp";
				kv.value = ss.str();
				waypoint_knowledge.values.push_back(kv);
				
				// Add the goal.
				add_waypoints_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
				add_waypoints_service.request.knowledge = waypoint_knowledge;
				if (!update_knowledge_client.call(add_waypoints_service)) {
					ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the goal (explored %s) to the knowledge base.", ss.str().c_str());
					exit(-1);
				}
				ROS_INFO("KCL: (RPSquirrelRecursion) Added the goal (explored %s) to the knowledge base.", ss.str().c_str());
				++waypoint_number;
			}
			
			rosplan_knowledge_msgs::KnowledgeItem waypoint_knowledge;
			add_waypoints_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			waypoint_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			waypoint_knowledge.instance_type = "waypoint";
			waypoint_knowledge.instance_name = "kenny_waypoint";
			add_waypoints_service.request.knowledge = waypoint_knowledge;
			if (!update_knowledge_client.call(add_waypoints_service)) {
				ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add an explore wayoint to the knowledge base.");
				exit(-1);
			}
			
			// Set the location of the robot.
			waypoint_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			waypoint_knowledge.attribute_name = "robot_at";
			waypoint_knowledge.is_negative = false;
			diagnostic_msgs::KeyValue kv;
			kv.key = "r";
			kv.value = "kenny";
			waypoint_knowledge.values.push_back(kv);
			kv.key = "wp";
			kv.value = "kenny_waypoint";
			waypoint_knowledge.values.push_back(kv);
			add_waypoints_service.request.knowledge = waypoint_knowledge;
			if (!update_knowledge_client.call(add_waypoints_service)) {
				ROS_ERROR("KCL: (TidyRooms) Could not add the fact (robot_at kenny room) to the knowledge base.");
				exit(-1);
			}
			ROS_INFO("KCL: (TidyRooms) Added (robot_at kenny room) to the knowledge base.");
			waypoint_knowledge.values.clear();
			
			
			std_msgs::Int8 nr_waypoint_number_int8;
			nr_waypoint_number_int8.data = waypoint_number;
			ROS_INFO("KCL: (RPSquirrelRecursion) Added %d waypoints to the knowledge base.", nr_waypoint_number_int8.data);
			
			PlanningEnvironment planning_environment;
			planning_environment.parseDomain(domain_path);
			planning_environment.update(*node_handle);
			PDDLProblemGenerator pddl_problem_generator;
			
			pddl_problem_generator.generatePDDLProblemFile(planning_environment, problem_path);

		} else if (action_name == "examine_area") {
			
			// Fetch all the objects.
			rosplan_knowledge_msgs::GetAttributeService get_attribute;
			get_attribute.request.predicate_name = "object_at";
			if (!get_attribute_client.call(get_attribute)) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the attributes of the predicate 'object_at'");
				return false;
			}
			
			std::map<std::string, std::string> object_to_location_mappings;
			for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = get_attribute.response.attributes.begin(); ci != get_attribute.response.attributes.end(); ++ci) {
				const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
				std::string object_predicate;
				std::string location_predicate;
				for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci) {
					const diagnostic_msgs::KeyValue& key_value = *ci;
					if ("o" == key_value.key) {
						object_predicate = key_value.value;
					}
					
					if ("wp" == key_value.key) {
						location_predicate = key_value.value;
					}
				}
				
				object_to_location_mappings[object_predicate] = location_predicate;
			}
			std_msgs::Int8 nr_objects;
			nr_objects.data = object_to_location_mappings.size();
			ROS_INFO("KCL: (RPSquirrelRecursion) Found %d objects to eximine.", nr_objects.data);
			
			std::string robot_location;
			for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = get_attribute.response.attributes[0].values.begin(); ci != get_attribute.response.attributes[0].values.end(); ++ci) {
				const diagnostic_msgs::KeyValue& knowledge_item = *ci;
				
				ROS_INFO("KCL: (RPSquirrelRecursion) Process robot_at attribute: %s %s", knowledge_item.key.c_str(), knowledge_item.value.c_str());
				
				if ("wp" == knowledge_item.key) {
					robot_location = knowledge_item.value;
				}
			}
			
			if ("" == robot_location) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the location of Kenny");
				return false;
			}
			
			ROS_INFO("KCL: (RPSquirrelRecursion) Kenny is at waypoint: %s", robot_location.c_str());
			
			ContingentStrategicClassifyPDDLGenerator::createPDDL(data_path, domain_name, problem_name, robot_location, object_to_location_mappings, 3);
			
		// Create the classify_object contingent domain and problem files.
		} else if (action_name == "observe-classifiable_on_attempt") {
		
			// Find the object that needs to be classified.
			std::string object_name;
			
			ROS_INFO("KCL: (RPSquirrelRecursion) %s.", action_name.c_str());
			
			object_name = last_received_msg.back().parameters[0].value;
			std::transform(object_name.begin(), object_name.end(), object_name.begin(), tolower);
			
			ROS_INFO("KCL: (RPSquirrelRecursion) Object name is: %s", object_name.c_str());
			
			// Get the location of the object.
			// (object_at ?o - object ?wp - location)
			rosplan_knowledge_msgs::GetAttributeService get_attribute;
			get_attribute.request.predicate_name = "object_at";
			if (!get_attribute_client.call(get_attribute)) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the attributes of the predicate 'object_at'");
				return false;
			}
			
			std::string object_location;
			bool found_object_location = false;
			for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = get_attribute.response.attributes.begin(); ci != get_attribute.response.attributes.end(); ++ci) {
				const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
				ROS_INFO("KCL: (RPSquirrelRoadmap) %s", knowledge_item.attribute_name.c_str());
				for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci) {
					const diagnostic_msgs::KeyValue& key_value = *ci;
					if ("o" == key_value.key && object_name == key_value.value) {
						found_object_location = true;
					}
					
					if ("wp" == key_value.key) {
						object_location = key_value.value;
					}
					
					ROS_INFO("KCL: (RPSquirrelRoadmap) %s -> %s.", key_value.key.c_str(), key_value.value.c_str());
				}
				
				if (found_object_location) {
					break;
				}
			}
			
			if (!found_object_location) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the location of the object %s", object_name.c_str());
				return false;
			}
			
			ROS_INFO("KCL: (RPSquirrelRecursion) Object location is: %s", object_location.c_str());
			
#ifndef SQUIRREL_SIMULATION
			// fetch position of object from message store
			std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
			if(message_store.queryNamed<geometry_msgs::PoseStamped>(object_name, results)) {
				if(results.size()<1) {
					ROS_ERROR("KCL: (RPSquirrelRoadmap) aborting waypoint request; no matching obID %s", object_name.c_str());
					return false;
				}
			} else {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) could not query message store to fetch object pose");
				return false;
			}

			// request classification waypoints for object
			geometry_msgs::PoseStamped &objPose = *results[0];
			
			squirrel_waypoint_msgs::ExamineWaypoint getTaskPose;
			getTaskPose.request.object_pose.header = objPose.header;
			getTaskPose.request.object_pose.pose = objPose.pose;
			if (!classify_object_waypoint_client.call(getTaskPose)) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve classification waypoints for %s.", object_name.c_str());
				return false;
			}

			std_msgs::Int8 debug_pose_number;
			debug_pose_number.data = getTaskPose.response.poses.size();
			ROS_INFO("KCL: (RPSquirrelRecursion) Found %d observation poses", debug_pose_number.data);
#else
			squirrel_waypoint_msgs::ExamineWaypoint getTaskPose;
			
			for (unsigned int i = 0; i < 4; ++i)
			{
				geometry_msgs::PoseWithCovarianceStamped pwcs;
				getTaskPose.response.poses.push_back(pwcs);
			}
#endif

			// Add all the waypoints to the knowledge base.
			std::stringstream ss;
			std::vector<std::string> observation_location_predicates;
			for(int i=0;i<getTaskPose.response.poses.size(); i++) {
				
				ss.str(std::string());
				ss << object_name << "_observation_wp" << i;
				
				ROS_INFO("KCL: (RPSquirrelRecursion) Process observation pose: %s", ss.str().c_str());
				
				rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
				updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
				updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
				updateSrv.request.knowledge.instance_type = "waypoint";
				updateSrv.request.knowledge.instance_name = ss.str();
				update_knowledge_client.call(updateSrv);
				
				observation_location_predicates.push_back(ss.str());
			}
			
			// Add a special observation waypoint.
			observation_location_predicates.push_back("nowhere");
			
			// Get the location of kenny.
			get_attribute.request.predicate_name = "robot_at";
			if (!get_attribute_client.call(get_attribute)) {// || get_attribute.response.attributes.size() != 3) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the attributes of the predicate 'robot_at'");
				return false;
			}
			
			for (unsigned int i = 0; i < get_attribute.response.attributes.size(); ++i)
			{
				ROS_INFO("robot_at attributes: %s", get_attribute.response.attributes[i].attribute_name.c_str());
				for (unsigned int j = 0; j < get_attribute.response.attributes[i].values.size(); ++j)
				{
					ROS_INFO("\t %s -> %s", get_attribute.response.attributes[i].values[j].key.c_str(), get_attribute.response.attributes[i].values[j].value.c_str());
				}
			}
			
			std::string robot_location;
			for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = get_attribute.response.attributes[0].values.begin(); ci != get_attribute.response.attributes[0].values.end(); ++ci) {
				const diagnostic_msgs::KeyValue& knowledge_item = *ci;
				
				ROS_INFO("KCL: (RPSquirrelRecursion) Process robot_at attribute: %s %s", knowledge_item.key.c_str(), knowledge_item.value.c_str());
				
				if ("wp" == knowledge_item.key) {
					robot_location = knowledge_item.value;
				}
			}
			
			if ("" == robot_location) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the location of Kenny");
				return false;
			}
			
			ROS_INFO("KCL: (RPSquirrelRecursion) Kenny is at waypoint: %s", robot_location.c_str());
			
			ContingentTacticalClassifyPDDLGenerator::createPDDL(data_path, domain_name, problem_name, robot_location, observation_location_predicates, object_name, object_location);
		} else if (action_name == "tidy_area") {
			// Get all the objects in the knowledge base that are in this area. 
			// TODO For now we assume there is only one area, so all objects in the knowledge base are relevant (unless already tidied).
			// Get the location of the objects.
			// (object_at ?o - object ?wp - location)
			rosplan_knowledge_msgs::GetAttributeService get_attribute;
			get_attribute.request.predicate_name = "object_at";
			if (!get_attribute_client.call(get_attribute)) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the attributes of the predicate 'object_at'");
				return false;
			}
			
			// Create a mapping of each object to its location.
			std::map<std::string, std::string> object_to_location_mapping;
			for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = get_attribute.response.attributes.begin(); ci != get_attribute.response.attributes.end(); ++ci) {
				const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
				std::string object_predicate;
				std::string object_location_predicate;
				for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci) {
					const diagnostic_msgs::KeyValue& key_value = *ci;
					if ("o" == key_value.key) {
						object_predicate = key_value.value;
					} else if ("wp" == key_value.key) {
						object_location_predicate = key_value.value;
					}
				}
				
				if (object_predicate != "" && object_location_predicate != "")
				{
					object_to_location_mapping[object_predicate] = object_location_predicate;
				}
			}
			
			// Filter those objects that are already tidied.
			// (tidy ?o - object)
			get_attribute.request.predicate_name = "tidy";
			if (!get_attribute_client.call(get_attribute)) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the attributes of the predicate 'tidy'");
				return false;
			}
			
			for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = get_attribute.response.attributes.begin(); ci != get_attribute.response.attributes.end(); ++ci) {
				const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
				for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci) {
					const diagnostic_msgs::KeyValue& key_value = *ci;
					if ("o" == key_value.key) {
						object_to_location_mapping.erase(key_value.value);
						break;
					}
				}
			}
			
			std_msgs::Int8 nr_untidied_objects;
			nr_untidied_objects.data = object_to_location_mapping.size();
			ROS_INFO("KCL: (RPSquirrelRecursion) Found %d untidied objects.", nr_untidied_objects.data);
			
			// Fetch the types of the untidied objects.
			// (is_of_type ?o - object ?t -type)
			get_attribute.request.predicate_name = "is_of_type";
			if (!get_attribute_client.call(get_attribute)) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the attributes of the predicate 'is_of_type'");
				return false;
			}
			
			std::map<std::string, std::string> object_to_type_mapping;
			for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = get_attribute.response.attributes.begin(); ci != get_attribute.response.attributes.end(); ++ci) {
				const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
				std::string type_predicate;
				std::string object_predicate;
				for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci) {
					const diagnostic_msgs::KeyValue& key_value = *ci;
					if ("o" == key_value.key && object_to_location_mapping.count(key_value.value) == 1) {
						object_predicate = key_value.value;
					} else if ("t" == key_value.key) {
						type_predicate = key_value.value;
					}
				}
				
				if ("" != object_predicate) {
					object_to_type_mapping[object_predicate] = type_predicate;
				}
			}
			
			// For all object that do not have a type (i.e. were not classified) we add the 'unknown' type.
			for (std::map<std::string, std::string>::const_iterator ci = object_to_location_mapping.begin(); ci != object_to_location_mapping.end(); ++ci)
			{
				const std::string& object_name = (*ci).first;
				if (object_to_type_mapping.find(object_name) == object_to_location_mapping.end())
				{
					object_to_type_mapping[object_name] = "unknown";
				}
			}
			
			// Get the location of the boxes.
			// (box_at ?b - box ?wp - waypoint)
			get_attribute.request.predicate_name = "box_at";
			if (!get_attribute_client.call(get_attribute)) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the attributes of the predicate 'box_at'");
				return false;
			}
			
			std::map<std::string, std::string> box_to_location_mapping;
			for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = get_attribute.response.attributes.begin(); ci != get_attribute.response.attributes.end(); ++ci) {
				const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
				std::string box_predicate;
				std::string box_location_predicate;
				for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci) {
					const diagnostic_msgs::KeyValue& key_value = *ci;
					if ("b" == key_value.key) {
						box_predicate = key_value.value;
					} else if ("wp" == key_value.key) {
						box_location_predicate = key_value.value;
					}
				}
				
				box_to_location_mapping[box_predicate] = box_location_predicate;
			}
			
			// Figure out which types of objects fit in each box.
			// (can_fit_inside ?t - type ?b - box)
			get_attribute.request.predicate_name = "can_fit_inside";
			if (!get_attribute_client.call(get_attribute)) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the attributes of the predicate 'box_at'");
				return false;
			}
			
			std::map<std::string, std::string> box_to_type_mapping;
			for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = get_attribute.response.attributes.begin(); ci != get_attribute.response.attributes.end(); ++ci) {
				const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
				std::string box_predicate;
				std::string type_predicate;
				for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci) {
					const diagnostic_msgs::KeyValue& key_value = *ci;
					if ("b" == key_value.key) {
						box_predicate = key_value.value;
					} else if ("t" == key_value.key) {
						type_predicate = key_value.value;
					}
				}
				
				box_to_type_mapping[box_predicate] = type_predicate;
			}
			
			// Get the location of kenny.
			// (robot_at ?r - robot ?wp - waypoint)
			get_attribute.request.predicate_name = "robot_at";
			if (!get_attribute_client.call(get_attribute)) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the attributes of the predicate 'robot_at'");
				return false;
			}
			
			std::string robot_location;
			for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = get_attribute.response.attributes[0].values.begin(); ci != get_attribute.response.attributes[0].values.end(); ++ci) {
				const diagnostic_msgs::KeyValue& knowledge_item = *ci;
				
				ROS_INFO("KCL: (RPSquirrelRecursion) Process robot_at attribute: %s %s", knowledge_item.key.c_str(), knowledge_item.value.c_str());
				
				if ("wp" == knowledge_item.key) {
					robot_location = knowledge_item.value;
				}
			}
			
			if ("" == robot_location) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the location of Kenny");
				return false;
			}
			
			ROS_INFO("KCL: (RPSquirrelRecursion) Kenny is at waypoint: %s", robot_location.c_str());
			
			
			ClassicalTidyPDDLGenerator::createPDDL(data_path, domain_name, problem_name, robot_location, object_to_location_mapping, object_to_type_mapping, box_to_location_mapping, box_to_type_mapping);
		} else {
			ROS_INFO("KCL: (RPSquirrelRecursion) Unable to create a domain for unknown action %s.", action_name.c_str());
			return false;
		}
		return true;
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_RPSquirrelRecursion");
		ros::NodeHandle nh;

		// create PDDL action subscriber
		KCL_rosplan::RPSquirrelRecursion rpsr(nh);
		
		// Setup all the simulated actions.
		KCL_rosplan::GotoPDDLAction goto_action(nh);
		KCL_rosplan::ExploreWaypointPDDLAction explore_waypoint_action(nh);
		KCL_rosplan::ClearObjectPDDLAction clear_object_action(nh);
		KCL_rosplan::ClassifyObjectPDDLAction classify_object_action(nh, 0.5f);
		KCL_rosplan::PutObjectInBoxPDDLAction pub_object_in_box_action(nh);
		KCL_rosplan::TidyObjectPDDLAction tidy_object_action(nh);
		KCL_rosplan::PickupPDDLAction pickup_action(nh);
		KCL_rosplan::ShedKnowledgePDDLAction shed_knowledge_action(nh);
		
		// listen for action dispatch
		ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPSquirrelRecursion::dispatchCallback, &rpsr);
		ROS_INFO("KCL: (RPSquirrelRecursion) Ready to receive");
		
		// Lets start the planning process.
		std::string data_path;
		nh.getParam("/data_path", data_path);
		
		std::string planner_path;
		nh.getParam("/planner_path", planner_path);
		
		std::stringstream ss;
		ss << data_path << "tidy_room_domain-nt.pddl";
		std::string domain_path = ss.str();
		
		ss.str(std::string());
		ss << data_path << "tidy_room_problem.pddl";
		std::string problem_path = ss.str();
		
		ss.str(std::string());
		ss << "timeout 10 " << planner_path << "ff -o DOMAIN -f PROBLEM";
		std::string planner_command = ss.str();
		
		rosplan_dispatch_msgs::PlanGoal psrv;
		psrv.domain_path = domain_path;
		psrv.problem_path = problem_path;
		psrv.data_path = data_path;
		psrv.planner_command = planner_command;
		psrv.start_action_id = 0;

		ROS_INFO("KCL: (RPSquirrelRecursion) Start plan action");
		actionlib::SimpleActionClient<rosplan_dispatch_msgs::PlanAction> plan_action_client("/kcl_rosplan/start_planning", true);

		plan_action_client.waitForServer();
		ROS_INFO("KCL: (RPSquirrelRecursion) Start planning server found");
		
		// send goal
		plan_action_client.sendGoal(psrv);
		ROS_INFO("KCL: (RPSquirrelRecursion) Goal sent");

		
		/*
		ros::ServiceClient run_planner_client = nh.serviceClient<rosplan_dispatch_msgs::PlanGoal>("/kcl_rosplan/planning_server");
		if (!run_planner_client.call(psrv))
		{
			ROS_ERROR("KCL: (TidyRoom) Failed to run the planning system.");
			exit(-1);
		}
		ROS_INFO("KCL: (TidyRoom) Planning system returned.");
		// Start the service ROSPlan will call when a domain and problem file needs to be generated.
		//ros::ServiceServer pddl_generation_service = nh.advertiseService("/kcl_rosplan/generate_planning_problem", &KCL_rosplan::RPSquirrelRecursion::generatePDDLProblemFile, &rpsr);
		*/

		ros::spin();
		return 0;
	}
