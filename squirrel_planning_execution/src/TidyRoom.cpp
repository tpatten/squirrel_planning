#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/KnowledgeItem.h>


int main(int argc, char **argv) {

	ros::init(argc, argv, "tidy_room_execution");
	ros::NodeHandle nh;

	ros::ServiceClient roadmap_service = nh.serviceClient<std_srvs::Empty>("/kcl_rosplan/roadmap_server");
	
	// Get access to the knowledge base.
	ros::ServiceClient get_instance_client = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_instances");
	ros::ServiceClient get_attribute_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_instances_attributes");
	ros::ServiceClient knowledge_update_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
	
	// Planner control.
	ros::ServiceClient run_planner_client = nh.serviceClient<std_srvs::Empty>("/kcl_rosplan/planning_server");
	
	// Keep running forever.
	bool room_is_tidy = false;
	while (!room_is_tidy) {
		ros::spinOnce();
		
		// Start by generating some waypoints.
		std_srvs::Empty dummy;
		if (!roadmap_service.call(dummy)) {
			ROS_ERROR("KCL: (TidyRoom) Failed to call the road map service.");
			return -1;
		}
		
		// Retreive the waypoints and add an observation goal to them.
		rosplan_knowledge_msgs::GetInstanceService get_instances;
		get_instances.request.type_name = "waypoint";
		
		if (!get_instance_client.call(get_instances)) {
			ROS_ERROR("KCL: (TidyRoom) Failed to get all the waypoint instances.");
			return -1;
		}
		
		// Generate a new goal for every waypoint we have received.
		for (std::vector<std::string>::const_iterator ci = get_instances.response.instances.begin(); ci != get_instances.response.instances.end(); ++ci)
		{
			const std::string& waypoint = *ci;
			
			// Setup the goal.
			rosplan_knowledge_msgs::KnowledgeItem waypoint_goal;
			waypoint_goal.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE_ATTRIBUTE;
			waypoint_goal.attribute_name = "explored";
			diagnostic_msgs::KeyValue kv;
			kv.key = "wp";
			kv.value = *ci;
			waypoint_goal.values.push_back(kv);
			
			// Add the goal.
			rosplan_knowledge_msgs::KnowledgeUpdateService add_goal;
			add_goal.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
			add_goal.request.knowledge = waypoint_goal;
			
			if (!knowledge_update_client.call(add_goal)) {
				ROS_ERROR("KCL: (TidyRoom) Could not add the goal for waypoint %s to the knowledge base.", (*ci).c_str());
				exit(-1);
			}
		}
		
		// Run the planner.
		if (!run_planner_client.call(dummy)) {
			ROS_ERROR("KCL: (TidyRoom) Failed to run the planning system.");
			exit(-1);
		}
	}
	return 0;
}
