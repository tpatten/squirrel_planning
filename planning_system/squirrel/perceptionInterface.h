#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "perception_msgs/SegmentedObject.h"

#include "planning_dispatch_msgs/ActionDispatch.h"
#include "planning_dispatch_msgs/ActionFeedback.h"

#include <iostream>
#include <string>
#include <stdio.h>
#include <string.h>

#ifndef SQUIRREL_perceptionInterface
#define SQUIRREL_perceptionInterface

/**
 * ROS node for SQUIRREL Summer School.
 * Contains interface for calling various controllers depending on action dispatch.
 * A class like this could service all actions implemented by a particular group.
 * It should call the relevant controllers using actionlib.
 */
namespace SQUIRREL_summerschool_perception {

	class PerceptionInterface
	{
		private:

		ros::NodeHandle nh;
		// action_id -> cancelled
		std::map<int,bool> actionCancelled;
		sensor_msgs::PointCloud2::Ptr currentPointCloud;
		bool havePointCloud;
		int objectCnt;

		// action execution methods
		void executeExplore(const planning_dispatch_msgs::ActionDispatch::ConstPtr& msg);
		void executeObserve(const planning_dispatch_msgs::ActionDispatch::ConstPtr& msg);
		void executeClassify(const planning_dispatch_msgs::ActionDispatch::ConstPtr& msg);
		std::vector<perception_msgs::SegmentedObject::Ptr> segmentObjects (const sensor_msgs::PointCloud2::ConstPtr& pointCloud);
		void classifyObject(const std::string &objectID, float &confidence);

		public:

		PerceptionInterface() : objectCnt(0), havePointCloud(false)
		{
			currentPointCloud.reset(new sensor_msgs::PointCloud2());
		}
		// ROS publisher and subscriber
		void dispatchCallback(const planning_dispatch_msgs::ActionDispatch::ConstPtr& msg);
		void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
		ros::Publisher feedbackPub;
		ros::Publisher objectPointCloudPub;
		ros::Publisher objectKnowledgePub;
	};
} // close namespace

#endif
