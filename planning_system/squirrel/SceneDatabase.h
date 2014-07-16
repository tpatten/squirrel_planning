#include "ros/ros.h"

#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "perception_msgs/SegmentedObject.h"
#include "planning_knowledge_msgs/PointCloudService.h"

#include <vector>
#include <iostream>
#include <fstream>

#ifndef KCL_sceneDatabase
#define KCL_sceneDatabase

namespace KCL_rosplan {

	class SceneDatabase
	{
	private:

		std::map<std::string, sensor_msgs::PointCloud2> clouds;

	public:

		// fetching, adding, and removing items to and from the scene database
		bool getPointCloud(planning_knowledge_msgs::PointCloudService::Request  &req, planning_knowledge_msgs::PointCloudService::Response &res);
		void addPointCloud(const perception_msgs::SegmentedObject::ConstPtr& msg);
		void removePointCloud(const std_msgs::String::ConstPtr& msg);
	};
}
#endif
