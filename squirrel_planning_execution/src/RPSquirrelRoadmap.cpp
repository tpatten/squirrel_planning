#include "squirrel_planning_execution/RPSquirrelRoadmap.h"

/* implementation of squirrel_planning_execution::RPSquirrelRoadmap.h */
namespace KCL_rosplan {

	/* constructor */
	RPSquirrelRoadmap::RPSquirrelRoadmap(ros::NodeHandle &nh, std::string frame)
	 : message_store(nh), fixed_frame(frame) {

		// config
		std::string dataPath("common/");
		nh.param("data_path", data_path, dataPath);

		// knowledge interface
		get_instance_client = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_instances");
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");

		// visualisation
		waypoints_pub = nh.advertise<visualization_msgs::MarkerArray>("/kcl_rosplan/viz/waypoints", 10, true);
	}

	/*-----------*/
	/* build PRM */
	/*-----------*/

	/**
	 * Generates waypoints and stores them in the knowledge base and scene database
	 */
	bool RPSquirrelRoadmap::generateRoadmap(rosplan_knowledge_msgs::CreatePRM::Request &req, rosplan_knowledge_msgs::CreatePRM::Response &res) {

		ros::NodeHandle nh("~");

		// clear previous roadmap from knowledge base
		ROS_INFO("KCL: (RPSquirrelRoadmap) Cleaning old roadmap");
		rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
		updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
		updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
		updateSrv.request.knowledge.instance_type = "waypoint";
		update_knowledge_client.call(updateSrv);

		// clear previous roadmap from scene database
		for (std::map<std::string,Waypoint*>::iterator wit=waypoints.begin(); wit!=waypoints.end(); ++wit) {
			message_store.deleteID(db_name_map[wit->first]);
		}
		db_name_map.clear();

		// clear from visualization
		clearMarkerArrays(nh);

		// generate waypoints
		ROS_INFO("KCL: (RPSquirrelRoadmap) Requesting waypoints");

		for (std::map<std::string, Waypoint*>::const_iterator ci = waypoints.begin(); ci != waypoints.end(); ++ci)
			delete (*ci).second;
		waypoints.clear();

		// fetch waypoints for observations

		// fetch waypoints for manipulations
		rosplan_knowledge_msgs::GetInstanceService getInstances;
		getInstances.request.type_name = "object";		
		if (!get_instance_client.call(getInstances)) {
			ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to get all the object instances.");
			return false;
		}
		ROS_INFO("KCL: (RPSquirrelRoadmap) Received all the object instances.");
		for (std::vector<std::string>::const_iterator ci = getInstances.response.instances.begin(); ci != getInstances.response.instances.end(); ++ci) {

			// fetch position of object from message store
			std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
			if(message_store.queryNamed<geometry_msgs::PoseStamped>(*ci, results)) {
				if(results.size()<1) {
					ROS_ERROR("KCL: (RPSquirrelRoadmap) aborting waypoint request; no matching obID %s", (*ci).c_str());
					return false;
				}
			} else {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) could not query message store to fetch object pose");
				return false;
			}

			// request manipulation waypoints for object
			geometry_msgs::PoseStamped &objPose = *results[0];

			squirrel_planning_knowledge_msgs::TaskPoseService getTaskPose;
			getTaskPose.request.target.header = objPose.header;
			getTaskPose.request.target.point = objPose.pose.position;
			/* TODO get service name for this
			if (!get_instance_client.call(getInstances)) {

				// save here for viz
				Waypoint* wp = new Waypoint(name, pose.pose.position.x, pose.pose.position.y);
				waypoints[wp->wpID] = wp;

				// publish visualization
				publishWaypointMarkerArray(nh);

				// add roadmap to knowledge base and scene database
				ROS_INFO("KCL: (RPSquirrelRoadmap) Adding knowledge");
				for (std::map<std::string,Waypoint*>::iterator wit=waypoints.begin(); wit!=waypoints.end(); ++wit) {

					// instance
					rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
					updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
					updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
					updateSrv.request.knowledge.instance_type = "waypoint";
					updateSrv.request.knowledge.instance_name = wit->first;
					update_knowledge_client.call(updateSrv);

					res.waypoints.push_back(wit->first);

					//data
					geometry_msgs::PoseStamped pose;
					pose.header.frame_id = fixed_frame;
					pose.pose.position.x = wit->second->real_x;
					pose.pose.position.y = wit->second->real_y;
					pose.pose.position.z = 0.0;
					pose.pose.orientation.x = 0.0;;
					pose.pose.orientation.y = 0.0;;
					pose.pose.orientation.z = 1.0;
					pose.pose.orientation.w = 1.0;
					std::string id(message_store.insertNamed(wit->first, pose));
					db_name_map[wit->first] = id;
				}
			}
			*/
		}

		ROS_INFO("KCL: (RPSquirrelRoadmap) Done");
		return true;
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		// setup ros
		ros::init(argc, argv, "rosplan_squirrel_map_server");
		ros::NodeHandle nh("~");

		// params
		std::string fixed_frame("world");
		nh.param("fixed_frame", fixed_frame, fixed_frame);

		// init
		KCL_rosplan::RPSquirrelRoadmap sms(nh, fixed_frame);
		ros::ServiceServer createPRMService = nh.advertiseService("/kcl_rosplan/roadmap_server/create_waypoints", &KCL_rosplan::RPSquirrelRoadmap::generateRoadmap, &sms);

		ROS_INFO("KCL: (RPSquirrelRoadmap) Ready to receive.");
		ros::spin();
		return 0;
	}
