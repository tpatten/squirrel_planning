#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "diagnostic_msgs/KeyValue.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "rosplan_knowledge_msgs/GetDomainAttributeService.h"
#include "rosplan_knowledge_msgs/GetDomainTypeService.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"
#include "rosplan_knowledge_msgs/DomainFormula.h"
#include "squirrel_prediction_msgs/RecommendRelations.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>

#ifndef KCL_recommender
#define KCL_recommender

/**
 * This file defines the RPRecommender class.
 */
namespace KCL_rosplan {

	class RPRecommender
	{

	private:
		
		std::string data_path;

		ros::ServiceClient recommender_client;

		// Knowledge base
		ros::ServiceClient update_knowledge_client;
		ros::ServiceClient domain_types_client;
		ros::ServiceClient domain_attributes_client;
		ros::ServiceClient current_instances_client;
		ros::ServiceClient current_goals_client;
		ros::ServiceClient current_attributes_client;

		// knowledge
		std::vector<std::string> domain_types;
		std::vector<rosplan_knowledge_msgs::DomainFormula> domain_attributes;
		std::map<std::string, std::string> current_instances;
		std::vector<std::string> all_instances;
		std::map<std::string,std::vector<rosplan_knowledge_msgs::KnowledgeItem> > current_attributes;

		bool attributeTrue(std::string instanceA, std::string instanceB, std::string attributeName);
		bool attributeFalse(std::string typeA, std::string typeB, rosplan_knowledge_msgs::DomainFormula attribute);

	public:

		/* constructor */
		RPRecommender(ros::NodeHandle &nh);

		/* database access */
		bool initialiseDatabase(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool readDatabase(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
	};
}
#endif
