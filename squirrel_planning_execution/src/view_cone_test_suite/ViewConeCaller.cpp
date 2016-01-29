#include <vector>

#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <squirrel_planning_execution/ViewConeGenerator.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char **argv) {

	if (argc < 7) {
		std::cout << "Usage: ./view_cone_test {occupancy_threshold} {nr_view_cones} {view_distance} {fov} {sample_size} {safe_distance}." << std::endl;
		exit(0);
	}
	
	int occupancy_threshold = ::atoi(argv[1]); // 2
	int nr_view_cones = ::atoi(argv[2]);       // 20
	float view_distance = ::atof(argv[3]);     //2.0f;
	float fov = ::atof(argv[4]);               //70.0f * M_PI / 180.0f;
	unsigned int sample_size = ::atoi(argv[5]); //1000;
	float safe_distance = ::atof(argv[6]);      //0.5f;
	
	ros::init(argc, argv, "rosplan_interface_view_cone_test_suite_ViewConeCaller");
	ros::NodeHandle nh;

	// create PDDL action subscriber
	KCL_rosplan::ViewConeGenerator vg(nh, "/test_occupancy_grid");
	ROS_INFO("Waiting for the occupancy grid to be published...");
	while (!vg.hasReceivedOccupancyGrid() && ros::ok()) {
		ros::spinOnce();
	}
	
	ROS_INFO("Occupancy grid published!");
	std::vector<geometry_msgs::Pose> poses;
	/*
	int occupancy_threshold = 2;
	int nr_view_cones = 20;
	float view_distance = 2.0f;
	float fov = 70.0f * M_PI / 180.0f;
	unsigned int sample_size = 1000;
	float safe_distance = 0.5f;
	*/
	vg.createViewCones(poses, nr_view_cones, occupancy_threshold, fov, view_distance, sample_size, safe_distance);

	ROS_INFO("Got the view cones, there are %d!", poses.size());

	std::vector<geometry_msgs::Point> waypoints;
	std::vector<std_msgs::ColorRGBA> waypoint_colours;
	std::vector<geometry_msgs::Point> triangle_points;
	std::vector<std_msgs::ColorRGBA> triangle_colours;
	
	for (std::vector<geometry_msgs::Pose>::const_iterator ci = poses.begin(); ci != poses.end(); ++ci) {
		const geometry_msgs::Pose& pose = *ci;
		
		// Create a triangle out of this pose.
		tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
		float yaw = tf::getYaw(q);
		
		ROS_INFO("(ViewConeGenerator) Yaw: %f.", yaw);
		
		// Calculate the triangle points encompasses the area that is viewed.
		tf::Vector3 view_point(pose.position.x, pose.position.y, pose.position.z);
		
		tf::Vector3 v0(view_distance, 0.0f, 0.0f);
		v0 = v0.rotate(tf::Vector3(0.0f, 0.0f, 1.0f), yaw);
		
		ROS_INFO("(ViewConeGenerator) Viewing direction: (%f, %f, %f).", v0.x(), v0.y(), v0.z());
		
		tf::Vector3 v0_normalised = v0.normalized();
		
		ROS_INFO("(ViewConeGenerator) Viewing direction (normalised): (%f, %f, %f).", v0_normalised.x(), v0_normalised.y(), v0_normalised.z());
		
		tf::Vector3 v1 = v0;
		v1 = v1.rotate(tf::Vector3(0.0f, 0.0f, 1.0f), fov / 2.0f);
		v1 = v1.normalize();
		
		ROS_INFO("(ViewConeGenerator) V1 (normalised): (%f, %f, %f).", v1.x(), v1.y(), v1.z());
		
		float length = v0.length() / v0_normalised.dot(v1);
		v1 *= length;
		v1 += view_point;
		
		ROS_INFO("(ViewConeGenerator) V1 (actual): (%f, %f, %f); length = %f.", v1.x(), v1.y(), v1.z(), length);
		
		tf::Vector3 v2 = v0;
		v2 = v2.rotate(tf::Vector3(0.0f, 0.0f, 1.0f), -fov / 2.0f);
		v2 = v2.normalize();
		ROS_INFO("(ViewConeGenerator) V2 (normalised): (%f, %f, %f).", v2.x(), v2.y(), v2.z());
		
		length = v0.length() / v0_normalised.dot(v2);
		v2 *= length;
		v2 += view_point;
		ROS_INFO("(ViewConeGenerator) V2 (actual): (%f, %f, %f); length = %f.", v2.x(), v2.y(), v2.z(), length);
		
		ROS_INFO("(ViewConeGenerator) Triangle: (%f, %f, %f), (%f, %f, %f), (%f, %f, %f).", view_point.x(), view_point.y(), view_point.z(), v1.x(), v1.y(), v1.z(), v2.x(), v2.y(), v2.z());
		

		
		geometry_msgs::Point pose_view_point;
		pose_view_point.x = view_point.x();
		pose_view_point.y = view_point.y();
		pose_view_point.z = view_point.z();
		triangle_points.push_back(pose_view_point);
		waypoints.push_back(pose_view_point);
		
		geometry_msgs::Point pose_v1;
		pose_v1.x = v1.x();
		pose_v1.y = v1.y();
		pose_v1.z = v1.z();
		triangle_points.push_back(pose_v1);
		
		geometry_msgs::Point pose_v2;
		pose_v2.x = v2.x();
		pose_v2.y = v2.y();
		pose_v2.z = v2.z();
		triangle_points.push_back(pose_v2);
		
		std_msgs::ColorRGBA colour;
		colour.r = (float)rand() / (float)RAND_MAX;
		colour.b = (float)rand() / (float)RAND_MAX;
		colour.g = (float)rand() / (float)RAND_MAX;
		colour.a = 1.0f;
		triangle_colours.push_back(colour);
		triangle_colours.push_back(colour);
		triangle_colours.push_back(colour);
		
		colour.r = 1;
		colour.b = 1;
		colour.g = 1;
		colour.a = 1.0f;
		waypoint_colours.push_back(colour);
	}
	
	ros::Publisher rivz_pub = nh.advertise<visualization_msgs::MarkerArray>("/vis/view_cones", 1000);
	
	visualization_msgs::MarkerArray marker_array;
	
	
	// Create a marker to be displayed.
	visualization_msgs::Marker marker;
		
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();
	marker.id = 0;
	marker.ns = "Waypoints";
	marker.type = visualization_msgs::Marker::SPHERE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.points = waypoints;
	marker.colors = waypoint_colours;
	marker.pose.position.x = 0.0;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;
	marker.color.g = 1.0;
	marker.color.a = 1.0;

	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker_array.markers.push_back(marker);
	
	// Reuse the marker for the actual view cones.2
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();
	marker.id = 1;
	marker.ns = "ViewCone";
	marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.points = triangle_points;
	marker.colors = triangle_colours;
	marker.pose.position.x = 0.0;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;
	marker.color.g = 1.0;
	marker.color.a = 1.0;

	marker.scale.x = 1.;
	marker.scale.y = 1.;
	marker.scale.z = 1.;
	marker_array.markers.push_back(marker);
	
	ros::Rate loop_rate(100);
	while (ros::ok()) {
		rivz_pub.publish(marker_array);
		loop_rate.sleep();
	}
	
	ros::spin();
	return 0;
}