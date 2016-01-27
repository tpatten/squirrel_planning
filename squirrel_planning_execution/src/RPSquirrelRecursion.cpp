#include <std_msgs/Int8.h>

#include "squirrel_planning_execution/RPSquirrelRecursion.h"
#include "squirrel_planning_execution/ContingentStrategicClassifyPDDLGenerator.h"
#include "squirrel_planning_execution/ContingentTacticalClassifyPDDLGenerator.h"

/* The implementation of RPSquirrelRecursion.h */
namespace KCL_rosplan {

	/* constructor */
	RPSquirrelRecursion::RPSquirrelRecursion(ros::NodeHandle &nh) : node_handle(&nh), message_store(nh) {
		
		// knowledge interface
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		
		// create the action feedback publisher
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
		
		get_instance_client = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
		get_attribute_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
		
		std::string classifyTopic("/squirrel_perception_examine_waypoint");
		nh.param("squirrel_perception_classify_waypoint_service_topic", classifyTopic, classifyTopic);
		classify_object_waypoint_client = nh.serviceClient<squirrel_planning_knowledge_msgs::TaskPoseService>(classifyTopic);
		
		pddl_generation_service = nh.advertiseService("/kcl_rosplan/generate_planning_problem", &KCL_rosplan::RPSquirrelRecursion::generatePDDLProblemFile, this);
		
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
		
		// { "_id" : ObjectId("56a64dab82b5af8506124f35"), "header" : { "stamp" : { "secs" : 0, "nsecs" : 0 }, "frame_id" : "map", "seq" : 0 }, "pose" : { "position" : { "y" : 2, "x" : 1, "z" : 0 }, "orientation" : { "y" : 0, "x" : 0, "z" : 0, "w" : 1 } }, "_meta" : { "stored_type" : "geometry_msgs/PoseStamped", "inserted_by" : "/rosplan_interface_mapping", "stored_class" : "geometry_msgs.msg._PoseStamped.PoseStamped", "name" : "teddybeer", "inserted_at" : ISODate("1970-01-01T00:00:30.476Z") } }
		// { "_id" : ObjectId("56a64f691d41c83466e349f1"), "header" : { "stamp" : { "secs" : 0, "nsecs" : 0 }, "frame_id" : "map", "seq" : 0 }, "pose" : { "position" : { "y" : 2, "x" : 1, "z" : 0 }, "orientation" : { "y" : 0, "x" : 0, "z" : 0, "w" : 1 } }, "_meta" : { "stored_type" : "geometry_msgs/PoseStamped", "inserted_by" : "/squirrel_interface_recursion", "stored_class" : "geometry_msgs.msg._PoseStamped.PoseStamped", "name" : "teddybeer", "inserted_at" : ISODate("2016-01-25T16:38:01.392Z") } }
	}

	/* action dispatch callback */
	void RPSquirrelRecursion::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		bool actionAchieved = false;
		last_received_msg = *msg;
		
		ROS_INFO("KCL: (RPSquirrelRecursion) action recieved %s", msg->name.c_str());

		// ignore actions
		if(0!=msg->name.compare("classify_object")
				&& 0!=msg->name.compare("examine_area")
				&& 0!=msg->name.compare("explore_area")
				&& 0!=msg->name.compare("tidy_area"))
			return;
		
		rosplan_dispatch_msgs::ActionFeedback fb;
		fb.action_id = msg->action_id;
		fb.status = "action enabled";
		action_feedback_pub.publish(fb);
		
		// create new planning system
		PlanningSystem planningSystem(*node_handle);
		SimplePlanDispatcher spd;
		planningSystem.plan_dispatcher = &spd;
		
		// publishers and such
		planningSystem.state_publisher = node_handle->advertise<std_msgs::String>("/kcl_rosplan/system_state", 5, true);
		planningSystem.plan_publisher = node_handle->advertise<rosplan_dispatch_msgs::CompletePlan>("/kcl_rosplan/plan", 5, true);
		planningSystem.plan_dispatcher->action_publisher = node_handle->advertise<rosplan_dispatch_msgs::ActionDispatch>("/kcl_rosplan/action_dispatch", 1000, true);
		planningSystem.plan_dispatcher->action_feedback_pub = node_handle->advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 5, true);
		ros::Subscriber feedback_sub = node_handle->subscribe("/kcl_rosplan/action_feedback", 10, &KCL_rosplan::PlanDispatcher::feedbackCallback, planningSystem.plan_dispatcher);
		ros::Subscriber command_sub = node_handle->subscribe("/kcl_rosplan/planning_commands", 10, &KCL_rosplan::PlanningSystem::commandCallback, &planningSystem);
		planningSystem.filter_publisher = node_handle->advertise<rosplan_knowledge_msgs::Filter>("/kcl_rosplan/planning_filter", 10, true);
		ros::Subscriber notification_sub = node_handle->subscribe("/kcl_rosplan/notification", 10, &KCL_rosplan::PlanningSystem::notificationCallBack, &planningSystem);
		
		// HERE GOES PROBLEM CONSTRUCTION AND CALLING RUNPLAN
		if (msg->name == "classify_object") {
			domain_name = "classify_domain.pddl";
			problem_name = "classify_problem.pddl";
			path = "";
			
			ROS_INFO("KCL: (RPSquirrelRecursion) process the action: %s", msg->name.c_str());
			
			// Run planner
			planningSystem.system_status = READY;
			actionAchieved = planningSystem.runPlanningServer(domain_name,problem_name,path,"ff -o DOMAIN -f PROBLEM");
		}

		ROS_INFO("KCL: (RPSquirrelRecursion) Ended: %s", msg->name.c_str());
		if(actionAchieved)
			fb.status = "action achieved";
		else
			fb.status = "action failed";
		action_feedback_pub.publish(fb);
	}
	
	/* Callback function that gets called by ROSPlan, we construct a domain and problem file */
	bool RPSquirrelRecursion::generatePDDLProblemFile(rosplan_knowledge_msgs::GenerateProblemService::Request &req, rosplan_knowledge_msgs::GenerateProblemService::Response &res)
	{
		ROS_INFO("RPSquirrelRecursion::generatePDDLProblemFile %s, with last msg: %s.", req.problem_path.c_str(), last_received_msg.name.c_str());
		// Create the classify_object contingent domain and problem files.
		if (last_received_msg.name == "classify_object") {
		
			// Find the object that needs to be classified.
			std::string object_name;
			
			// publish feedback (enabled)
			ROS_INFO("KCL: (RPSquirrelRecursion) Started: %s", last_received_msg.name.c_str());
			
			for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = last_received_msg.parameters.begin(); ci != last_received_msg.parameters.end(); ++ci) {
				if ("o" == (*ci).key) {
					object_name = (*ci).value;
				}
			}
			
			ROS_INFO("KCL: (RPSquirrelRecursion) Object name is: %s", object_name.c_str());
			
			// Get the location of the object.
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
				for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci) {
					const diagnostic_msgs::KeyValue& key_value = *ci;
					if ("o" == key_value.key && object_name == key_value.value) {
						found_object_location = true;
					}
					
					if ("wp" == key_value.key) {
						object_location = key_value.value;
					}
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

			// TODO Change to actual service type.
			squirrel_planning_knowledge_msgs::TaskPoseService getTaskPose;
			getTaskPose.request.target.header = objPose.header;
			getTaskPose.request.target.point = objPose.pose.position;
			if (!classify_object_waypoint_client.call(getTaskPose)) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve classification waypoints for %s.", object_name.c_str());
				return false;
			}
			// Dummy test code.
			/*
			for (unsigned int i = 0; i < 4; ++i)
			{
				geometry_msgs::PoseWithCovariance p;
				getTaskPose.response.poses.push_back(p);
			}
			*/
			std_msgs::Int8 debug_pose_number;
			debug_pose_number.data = getTaskPose.response.poses.size();
			ROS_INFO("KCL: (RPSquirrelRecursion) Found %d observation poses", debug_pose_number.data);
			
			// Add all the waypoints to the knowledge base.
			std::stringstream ss;
			std::vector<std::string> observation_location_predicates;
			for(int i=0;i<getTaskPose.response.poses.size(); i++) {
				geometry_msgs::Point p = getTaskPose.response.poses[i].pose.position;
				
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
			
			// Get the location of kenny.
			get_attribute.request.predicate_name = "robot_at";
			if (!get_attribute_client.call(get_attribute) || get_attribute.response.attributes.size() != 1) {
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
			
			// TODO Change it to the strategic contingency plan generator.
			ContingentTacticalClassifyPDDLGenerator::createPDDL(path, domain_name, problem_name, robot_location, observation_location_predicates, object_name, object_location);
		} /*else if (last_received_msg.name == "tidy_area") {
			// Get all the objects in the knowledge base that are in this area. 
			// TODO For now we assume there is only one area, so all objects in the knowledge base are relevant (unless already tidied).
			// Get the location of the objects.
			rosplan_knowledge_msgs::GetAttributeService get_attribute;
			get_attribute.request.predicate_name = "object_at";
			if (!get_attribute_client.call(get_attribute)) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the attributes of the predicate 'object_at'");
				return false;
			}
			
			std::vector<rosplan_knowledge_msgs::KnowledgeItem> object_locations = get_attribute.response.attributes;
			
			// Filter those objects that are already tidied.
			get_attribute.request.predicate_name = "tidy";
			if (!get_attribute_client.call(get_attribute)) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the attributes of the predicate 'tidy'");
				return false;
			}
			
			(tidy ?o - object
			for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = get_attribute.response.attributes.begin(); ci != get_attribute.response.attributes.end(); ++ci) {
				const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
				for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci) {
					const diagnostic_msgs::KeyValue& key_value = *ci;
					if ("o" == key_value.key) {
						found_object_location = true;
					}
					
					if ("wp" == key_value.key) {
						object_location = key_value.value;
					}
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
		}*/
		
		// Domain and problem files are generated!
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
		
		// Start the service ROSPlan will call when a domain and problem file needs to be generated.
		//ros::ServiceServer pddl_generation_service = nh.advertiseService("/kcl_rosplan/generate_planning_problem", &KCL_rosplan::RPSquirrelRecursion::generatePDDLProblemFile, &rpsr);

		// listen for action dispatch
		ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPSquirrelRecursion::dispatchCallback, &rpsr);
		ROS_INFO("KCL: (RPSquirrelRecursion) Ready to receive");

		ros::spin();
		return 0;
	}
