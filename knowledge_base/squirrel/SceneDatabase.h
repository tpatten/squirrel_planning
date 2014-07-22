#include "ros/ros.h"

#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "perception_msgs/SegmentedObject.h"
#include "perception_msgs/ObjectPosition.h"
#include "geometry_msgs/Point.h"
#include "planning_knowledge_msgs/PointCloudService.h"
#include "planning_knowledge_msgs/PositionService.h"

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
		std::map<std::string, geometry_msgs::Point> positions;
		
	public:

		// fetching, adding, and removing items to and from the scene database
		bool getPointCloud(planning_knowledge_msgs::PointCloudService::Request  &req, planning_knowledge_msgs::PointCloudService::Response &res);
		void addPointCloud(const perception_msgs::SegmentedObject::ConstPtr& msg);
		void removePointCloud(const std_msgs::String::ConstPtr& msg);

		bool getPosition(planning_knowledge_msgs::PointCloudService::Request  &req, planning_knowledge_msgs::PositionService::Response &res);
		void addPosition(const perception_msgs::ObjectPosition::ConstPtr& msg);
		void removePosition(const std_msgs::String::ConstPtr& msg);
	};
}
#endif
