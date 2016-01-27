#include <std_msgs/Int8.h>

#include "squirrel_planning_execution/RPSquirrelRecursion.h"
#include "squirrel_planning_execution/ContingentStrategicClassifyPDDLGenerator.h"
#include "squirrel_planning_execution/ContingentTacticalClassifyPDDLGenerator.h"
#include "squirrel_planning_execution/ContingentTidyPDDLGenerator.h"
#include <map>

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
		classify_object_waypoint_client = nh.serviceClient<squirrel_waypoint_msgs::ExamineWaypoint>(classifyTopic);
		
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
		last_received_msg.push_back(*msg);
		
		ROS_INFO("KCL: (RPSquirrelRecursion) action recieved %s", msg->name.c_str());

		// ignore actions
		if(0!=msg->name.compare("classify_object")
				&& 0!=msg->name.compare("examine_area")
				&& 0!=msg->name.compare("explore_area")
				&& 0!=msg->name.compare("tidy_area"))
			return;

		// publish feedback (enabled)
		rosplan_dispatch_msgs::ActionFeedback fb;
		fb.action_id = msg->action_id;
		fb.status = "action enabled";
		action_feedback_pub.publish(fb);
		
		// create new planning system
		std::stringstream nspace;
		nspace << msg->name << "_" << msg->action_id;
		std::stringstream commandLine;
		commandLine << "rosrun rosplan_planning_system planner ";
		commandLine << "/rosplan_planning_system:=/" << nspace.str() << "/rosplan_planning_system ";
		commandLine << "/kcl_rosplan/plan:=/kcl_rosplan" << nspace.str() << "/plan ";
		commandLine << "/kcl_rosplan/system_state:=/kcl_rosplan" << nspace.str() << "/system_state ";
		commandLine << "/kcl_rosplan/planning_commands:=/kcl_rosplan" << nspace.str() << "/planning_commands ";
		commandLine << "/kcl_rosplan/planning_server:=/kcl_rosplan" << nspace.str() << "/planning_server ";
		commandLine << "/kcl_rosplan/planning_server_params:=/kcl_rosplan" << nspace.str() << "/planning_server_params ";
		system(commandLine.str().c_str());

		// Problem Construction and planning
		if (msg->name == "classify_object") {
			
			ROS_INFO("KCL: (RPSquirrelRecursion) process the action: %s", msg->name.c_str());
			
			// Run planner
			ros::NodeHandle nh;
			std::stringstream commandPub;
			commandPub << "/kcl_rosplan" << nspace.str() << "/planning_server_params";
			ros::ServiceClient planningClient = nh.serviceClient<rosplan_dispatch_msgs::PlanningService>(commandPub.str());
			
			domain_name = "classify_domain.pddl";
			problem_name = "classify_problem.pddl";
			path = "";

			rosplan_dispatch_msgs::PlanningService psrv;
			psrv.request.domain_path = domain_name;
			psrv.request.problem_path = problem_name;
			psrv.request.data_path = path;
			psrv.request.planner_command = "ff -o DOMAIN -p PROBLEM";

			if(planningClient.call(psrv)) {
				actionAchieved = true;
			}
			ros::spinOnce();
		}

		ROS_INFO("KCL: (RPSquirrelRecursion) Ended: %s", msg->name.c_str());
		if(actionAchieved)
			fb.status = "action achieved";
		else
			fb.status = "action failed";
		action_feedback_pub.publish(fb);

		last_received_msg.pop_back();
	}
	
	/* Callback function that gets called by ROSPlan, we construct a domain and problem file */
	bool RPSquirrelRecursion::generatePDDLProblemFile(rosplan_knowledge_msgs::GenerateProblemService::Request &req, rosplan_knowledge_msgs::GenerateProblemService::Response &res)
	{
		ROS_INFO("RPSquirrelRecursion::generatePDDLProblemFile %s, with last msg: %s.", req.problem_path.c_str(), last_received_msg.back().name.c_str());
		// Create the classify_object contingent domain and problem files.
		if (last_received_msg.back().name == "classify_object") {
		
			// Find the object that needs to be classified.
			ROS_INFO("KCL: (RPSquirrelRecursion) Started: %s", last_received_msg.back().name.c_str());
			std::string object_name;
			
			for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = last_received_msg.back().parameters.begin(); ci != last_received_msg.back().parameters.end(); ++ci) {
				if ("o" == (*ci).key) {
					object_name = (*ci).value;
				}
			}
			
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
			
			// Get the location of kenny.
			// (robot_at ?r - robot ?wp - waypoint)
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
		} else if (last_received_msg.name == "tidy_area") {
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
			
			
			ContingentTidyPDDLGenerator::createPDDL(path, domain_name, problem_name, robot_location, object_to_location_mapping, object_to_type_mapping, box_to_location_mapping, box_to_type_mapping);
		}
		
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
