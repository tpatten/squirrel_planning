#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char **argv) {

	ros::init(argc, argv, "rosplan_interface_view_cone_test_suite_OccupancyGridPublisher");
	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("/test_occupancy_grid", 1);
	
	// Test occupancy grid.
	nav_msgs::OccupancyGrid grid;
	grid.header.seq = 0;
	grid.header.stamp = ros::Time::now();
	grid.header.frame_id = "map";
	
	// Meta data
	grid.info.map_load_time = ros::Time::now();
	grid.info.resolution = 0.25f;
	grid.info.width = 40;
	grid.info.height = 40;
	grid.info.origin.position.x = 0;
	grid.info.origin.position.y = 0;
	grid.info.origin.position.z = 0;
	grid.info.origin.orientation.x = 0;
	grid.info.origin.orientation.y = 0;
	grid.info.origin.orientation.z = 0;
	grid.info.origin.orientation.w = 1;
	
	// Actual cell data.
	for (int y = 0; y < grid.info.height; ++y) {
		for (int x = 0; x < grid.info.width; ++x) {
			if (x < 2 || x > grid.info.width - 3 || y < 2 || y > grid.info.height - 3) {
				grid.data.push_back(100);
			} else {
				grid.data.push_back(0);
			}
		}
	}
	
	// Create a broadcast of TF.
	tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(0, 0, 0));
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	transform.setRotation(q);
	
	while (ros::ok()) {
		pub.publish(grid);
		ros::spinOnce();
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "map"));
		
		grid.header.seq = grid.header.seq + 1;
		grid.header.stamp = ros::Time::now();
		ros::Duration(1.0).sleep();
	}
	return 0;
}
