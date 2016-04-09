#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>
#include "mongodb_store/message_store.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_srvs/Empty.h"
#include "diagnostic_msgs/KeyValue.h"
#include <actionlib/client/simple_action_client.h>

#include "rosplan_dispatch_msgs/PlanAction.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"

#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "rosplan_knowledge_msgs/GenerateProblemService.h"
#include "rosplan_knowledge_msgs/KnowledgeQueryService.h"
#include "rosplan_dispatch_msgs/PlanningService.h"

#include "rosplan_planning_system/PlanningEnvironment.h"
#include "rosplan_planning_system/PDDLProblemGenerator.h"

#include "squirrel_waypoint_msgs/ExamineWaypoint.h"

#ifndef SQUIRREL_PLANNING_EXECUTION_SORTINGGAME_H
#define SQUIRREL_PLANNING_EXECUTION_SORTINGGAME_H

namespace KCL_rosplan {
	
	class SortingGame
	{

	private:
		ros::NodeHandle* node_handle;
		mongodb_store::MessageStoreProxy message_store;
		ros::Publisher action_feedback_pub;
		
		/* PDDL problem generation */
		

		/* knowledge service clients */
		ros::ServiceClient update_knowledge_client;
		ros::ServiceClient get_instance_client;
		ros::ServiceClient get_attribute_client;
		ros::ServiceClient query_knowledge_client;
		
		// waypoint request services
		ros::ServiceClient classify_object_waypoint_client;
		
		// server that generates the PDDL domain and problem files.
		ros::ServiceServer pddl_generation_service;
		
		// Cache the last message sent.
		std::vector<rosplan_dispatch_msgs::ActionDispatch> last_received_msg;
		
		// Generate the initial state.
		void generateInitialState();
		
		bool initial_problem_generated;
		
	public:

		/* constructor */
		SortingGame(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif
