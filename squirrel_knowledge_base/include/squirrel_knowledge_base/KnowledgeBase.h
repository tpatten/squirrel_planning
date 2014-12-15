#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>

#include "squirrel_planning_knowledge_msgs/InstanceService.h"
#include "squirrel_planning_knowledge_msgs/AttributeService.h"
#include "squirrel_planning_knowledge_msgs/KnowledgeItem.h"
#include "squirrel_planning_knowledge_msgs/Notification.h"
#include "squirrel_planning_knowledge_msgs/Filter.h"

#ifndef KCL_knowledgebase
#define KCL_knowledgebase

namespace KCL_rosplan {

	class KnowledgeBase
	{
	private:

		// checking if filters are violated
		bool containsInstance(const squirrel_planning_knowledge_msgs::KnowledgeItem &a, std::string &name);
		bool sameKnowledge(const squirrel_planning_knowledge_msgs::KnowledgeItem &a, const squirrel_planning_knowledge_msgs::KnowledgeItem &b);
		bool isInFilter(const squirrel_planning_knowledge_msgs::KnowledgeItem &a, const squirrel_planning_knowledge_msgs::KnowledgeItem &b);
		void checkFilters(const squirrel_planning_knowledge_msgs::KnowledgeItem &a, bool added);

	public:

		// symbolic model
		std::map<std::string, std::vector<std::string> > domainInstances;
		std::vector<squirrel_planning_knowledge_msgs::KnowledgeItem> domainAttributes;
		std::vector<squirrel_planning_knowledge_msgs::KnowledgeItem> domainFunctions;
		std::vector<squirrel_planning_knowledge_msgs::KnowledgeItem> domainGoals;
		std::map<std::string, std::vector<squirrel_planning_knowledge_msgs::KnowledgeItem> > instanceAttributes;

		// planning and mission filter
		std::vector<squirrel_planning_knowledge_msgs::KnowledgeItem> planningFilter;
		std::vector<squirrel_planning_knowledge_msgs::KnowledgeItem> missionFilter;

		// planning_system notification
		void planningFilterCallback(const squirrel_planning_knowledge_msgs::Filter::ConstPtr& msg);
		ros::Publisher notificationPublisher;

		// fetching the symbolic model
		bool getInstances(squirrel_planning_knowledge_msgs::InstanceService::Request  &req, squirrel_planning_knowledge_msgs::InstanceService::Response &res);
		bool getInstanceAttr(squirrel_planning_knowledge_msgs::AttributeService::Request  &req, squirrel_planning_knowledge_msgs::AttributeService::Response &res);
		bool getDomainAttr(squirrel_planning_knowledge_msgs::AttributeService::Request  &req, squirrel_planning_knowledge_msgs::AttributeService::Response &res);
		bool getCurrentGoals(squirrel_planning_knowledge_msgs::AttributeService::Request  &req, squirrel_planning_knowledge_msgs::AttributeService::Response &res);

		// adding and removing items to and from the knowledge base
		void addKnowledge(const squirrel_planning_knowledge_msgs::KnowledgeItem::ConstPtr& msg);
		void addMissionGoal(const squirrel_planning_knowledge_msgs::KnowledgeItem::ConstPtr& msg);
		void removeKnowledge(const squirrel_planning_knowledge_msgs::KnowledgeItem::ConstPtr& msg);
	};
}
#endif
