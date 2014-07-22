#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include "perception_msgs/SegmentedObject.h"
#include "perception_msgs/classification.h"
#include "perception_srv_definitions/segment.h"
#include "perception_srv_definitions/classify.h"
#include "planning_knowledge_msgs/KnowledgeItem.h"
#include "planning_knowledge_msgs/PointCloudService.h"
#include "perceptionInterface.h"

/**
 * ROS node for SQUIRREL Summer School.
 * Contains interface for calling various controllers depending on action dispatch.
 * This file could service all actions implemented by a particular group.
 * It should call the relevant controllers using actionlib.
 */
namespace SQUIRREL_summerschool_perception {

	void PerceptionInterface::dispatchCallback(const planning_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		if(0 == msg->name.compare("cancel_action"))
			actionCancelled[msg->action_id] = true;
		else if(0 == msg->name.compare("explore"))
			executeExplore(msg);
		else if(0 == msg->name.compare("push"))
			executePush(msg);
		else if(0 == msg->name.compare("classify"))
			executeClassify(msg);
	}

	void PerceptionInterface::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{
		ROS_INFO("Have new point cloud: w/h: %d/%d\n", msg->width, msg->height);
		*currentPointCloud = *msg;
		havePointCloud = true;
	}

	/**
	 * Get a point cloud of the scene and extraxct all point clusters that stick out from the plane as objects.
	 * Objects are given unique ID names.
	 * @param pointCloudMsg input point cloud message
	 * @return list of segmented object messages
	 */
	std::vector<perception_msgs::SegmentedObject::Ptr> PerceptionInterface::segmentObjects(const sensor_msgs::PointCloud2::ConstPtr& pointCloudMsg)
	{
		std::vector<perception_msgs::SegmentedObject::Ptr> objectMsgs;

		ROS_INFO("going to call segmentation service...\n");
		ros::ServiceClient client = nh.serviceClient<perception_srv_definitions::segment>("/object_segmenter_service/object_segmenter");
		client.waitForExistence();

		perception_srv_definitions::segment srv;
		srv.request.cloud = *pointCloudMsg;

		std::vector<sensor_msgs::PointCloud2::Ptr> objects;
		if (client.call(srv))
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::fromROSMsg (*pointCloudMsg, *scene);
			ROS_INFO("Number of segmented objects: %d\n", (int)srv.response.clusters_indices.size());
			for(size_t i = 0; i < srv.response.clusters_indices.size(); i++)
			{
				ROS_INFO("object %d size: %d\n", (int)i, (int)srv.response.clusters_indices[i].data.size());
				pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);
				for(size_t k = 0; k < srv.response.clusters_indices[i].data.size(); k++)
				{
					object->push_back(scene->at(srv.response.clusters_indices[i].data[k]));
				}
				perception_msgs::SegmentedObject::Ptr objectMsg(new perception_msgs::SegmentedObject());
				sensor_msgs::PointCloud2::Ptr segmentMsg(new sensor_msgs::PointCloud2());
				pcl::toROSMsg (*object, *segmentMsg);
				
				// add a unique object ID
				std::stringstream ss;
				ss << "object" << objectCnt;
				objectCnt++;
				objectMsg->name = ss.str();
				objectMsg->segment = *segmentMsg;
				objectMsgs.push_back(objectMsg);
			}
		}
		else
		{
			ROS_ERROR("Call did not succeed\n");
		}

        	return objectMsgs;
	}

	void PerceptionInterface::classifyObject(const std::string &objectID, std::string &objectClass, float &classConfidence, geometry_msgs::Point32 &position)
	{
		objectClass = "";
		classConfidence = 0.;
		position.x = position.y = position.z = 0.;

		ros::ServiceClient pointCloudClient = nh.serviceClient<planning_knowledge_msgs::PointCloudService>("/kcl_rosplan/get_point_cloud");
		planning_knowledge_msgs::PointCloudService pointCloudSrv;
		pointCloudSrv.request.name = objectID;
		ROS_INFO("Classify: getting object point cloud.");
		if (pointCloudClient.call(pointCloudSrv))
		{
			ros::ServiceClient classifierClient = nh.serviceClient<perception_srv_definitions::classify>("/classifier_service/classify");
			perception_srv_definitions::classify classifySrv;

			// the classify service needs an index vector, so we just create one, which selects all the
			// points
			std::vector<std_msgs::Int32MultiArray> clusterIndices;
			clusterIndices.push_back(std_msgs::Int32MultiArray());
			clusterIndices[0].data.resize(pointCloudSrv.response.cloud.height*pointCloudSrv.response.cloud.width);
			for(size_t i = 0; i < clusterIndices[0].data.size(); i++)
				clusterIndices[0].data[i] = i;

			classifySrv.request.cloud = pointCloudSrv.response.cloud;
			classifySrv.request.clusters_indices = clusterIndices;
			ROS_INFO("Calling classsifier service.");
			if (classifierClient.call(classifySrv))
			{
				// se sent one object for classification, so we expect one answer
				if(classifySrv.response.class_results.size() == 1)
				{
					// the classifier returns all classes with confidences, take the one with the highest
					float maxConf = 0;
					for(size_t i = 0; i < classifySrv.response.class_results[0].class_type.size(); i++)
					{
						if(classifySrv.response.class_results[0].confidence[i] > maxConf)
						{
							maxConf = classifySrv.response.class_results[0].confidence[i];
							objectClass = classifySrv.response.class_results[0].class_type[i].data;
							position = classifySrv.response.centroid[0];
						}
					}
				}
			}
			else
			{
				ROS_ERROR("Failed to call classifier service.");
			}
		}
		else
		{
			ROS_ERROR("Failed to call point cloud service.");
		}
	}

	void PerceptionInterface::executeObserve(const planning_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		ROS_INFO("Execute observe\n");

		if(havePointCloud)
		{
			std::vector<perception_msgs::SegmentedObject::Ptr> objects = segmentObjects(currentPointCloud);
			for(size_t i = 0; i < objects.size(); i++)
			{
				// now we have to publish two things:
				// 1. the knowledge that the (as yet unknown) objects exist, for the planner
				// 2. tha actual point cloud that makes up this object, for the scene database
				planning_knowledge_msgs::KnowledgeItem::Ptr objectKnowledge(new planning_knowledge_msgs::KnowledgeItem());
				objectKnowledge->knowledge_type = planning_knowledge_msgs::KnowledgeItem::INSTANCE;
				objectKnowledge->instance_type = "object";
				objectKnowledge->instance_name = objects[i]->name;
				objectKnowledgePub.publish(objectKnowledge);
				objectPointCloudPub.publish(objects[i]);
			}
		}
	}

	void PerceptionInterface::executeClassify(const planning_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		ROS_INFO("Execute classify\n");

		planning_dispatch_msgs::ActionFeedback feedbackEnabled;
		feedbackEnabled.action_id = msg->action_id;
		feedbackEnabled.status = "action enabled";
		feedbackPub.publish(feedbackEnabled);

		if(msg->parameters.size() == 1 && msg->parameters[0].key == "o")
		{
			std::string objectID = msg->parameters[0].value;
			std::string objectClass;
			float classConfidence;
			geometry_msgs::Point32 position;
			classifyObject(objectID, objectClass, classConfidence, position);

			ROS_INFO("found object of class '%s' with confidence %.3f at [%.3f %.3f %.3f]\n",
				objectClass.c_str(), classConfidence, position.x, position.y, position.z);

			//objectPositionPub.publish(position ..);
			
			// tell planning that we have classified the object
			planning_knowledge_msgs::KnowledgeItem::Ptr know1(new planning_knowledge_msgs::KnowledgeItem());
			know1->knowledge_type = planning_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE;
			know1->attribute_name = "classified";
			diagnostic_msgs::KeyValue classpair;
			classpair.key = "o";
			classpair.value = msg->parameters[0].value;
			know1->values.push_back(classpair);
			objectKnowledgePub.publish(know1);

			// Now depemnding on which class we found we decide what to do: clean away toys (i.e. set them untidy), ignore other stuff
			if(objectClass == "car") // or any other toy
			{
				planning_knowledge_msgs::KnowledgeItem::Ptr know2(new planning_knowledge_msgs::KnowledgeItem());
				know2->knowledge_type = planning_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE;
				know2->attribute_name = "untidy";
				diagnostic_msgs::KeyValue tidypair;
				tidypair.key = "o";
				tidypair.value = msg->parameters[0].value;
				know2->values.push_back(tidypair);
				objectKnowledgePub.publish(know2);
			}
		}

		planning_dispatch_msgs::ActionFeedback feedbackAchieved;
		feedbackAchieved.action_id = msg->action_id;
		feedbackAchieved.status = "action achieved";
		feedbackPub.publish(feedbackAchieved);
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc,argv,"squirrel_perception_interface");
		ros::NodeHandle nh("~");

		SQUIRREL_summerschool_perception::PerceptionInterface pi;
		pi.feedbackPub = nh.advertise<planning_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 1000, true);
		pi.objectPointCloudPub = nh.advertise<perception_msgs::SegmentedObject>("/kcl_rosplan/add_point_cloud", 100, true);
		//pi.objectPositionPub = nh.advertise<perception_msgs::SegmentedObject>("/kcl_rosplan/add_position", 100, true);
		pi.objectKnowledgePub = nh.advertise<planning_knowledge_msgs::KnowledgeItem>("/kcl_rosplan/add_knowledge", 100, true);
		ros::Subscriber dispatchSub = nh.subscribe("/kcl_rosplan/action_dispatch", 1000,
			&SQUIRREL_summerschool_perception::PerceptionInterface::dispatchCallback, &pi);
		ros::Subscriber pointCloudSub = nh.subscribe("/cloud_pcd", 1, //("/camera/depth_registered/points", 1,
			&SQUIRREL_summerschool_perception::PerceptionInterface::pointCloudCallback, &pi);

		ros::spin();
		return 0;
	}
