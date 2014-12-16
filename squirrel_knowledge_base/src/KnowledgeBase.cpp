#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "squirrel_knowledge_base/KnowledgeBase.h"

namespace KCL_rosplan {

	/*----------------*/
	/* removing items */
	/*----------------*/

	void KnowledgeBase::removeKnowledge(const squirrel_planning_knowledge_msgs::KnowledgeItem::ConstPtr& msg) {

		if(msg->knowledge_type == squirrel_planning_knowledge_msgs::KnowledgeItem::INSTANCE) {		

			// search for instance
			std::vector<std::string>::iterator iit;
			iit = domainInstances[msg->instance_type].begin();
			while(iit!=domainInstances[msg->instance_type].end()) {
				std::string name = *iit;
				if(name.compare(msg->instance_name)==0 || msg->instance_name.compare("")==0) {

					// remove instance from knowledge base
					ROS_INFO("KCL: (KB) Removing instance (%s, %s)", msg->instance_type.c_str(), msg->instance_name.c_str());
					checkFilters(*msg, false);
					domainInstances[msg->instance_type].erase(iit);

					// remove affected domain attributes
					std::vector<squirrel_planning_knowledge_msgs::KnowledgeItem>::iterator pit;
					for(pit=domainAttributes.begin(); pit!=domainAttributes.end(); pit++) {
						if(containsInstance(*pit, name)) {
							ROS_INFO("KCL: (KB) Removing domain attribute (%s)", pit->attribute_name.c_str());
							checkFilters(*pit, false);
							pit = domainAttributes.erase(pit);
							if(pit==domainAttributes.end()) break;
						}
					}

					// remove affected instance attributes
					for(pit=instanceAttributes[name].begin(); pit!=instanceAttributes[name].end(); pit++) {
						if(containsInstance(*pit, name)) {
							ROS_INFO("KCL: (KB) Removing instance attribute (%s, %s)", name.c_str(), pit->attribute_name.c_str());
							checkFilters(*pit, false);
							pit = instanceAttributes[name].erase(pit);
							if(pit==instanceAttributes[name].end()) break;
						}
					}

					// stop looping through instances
					break;
				} 
			}
		} else if(msg->knowledge_type == squirrel_planning_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE) {

			// remove domain attribute (predicate) from knowledge base
			std::vector<squirrel_planning_knowledge_msgs::KnowledgeItem>::iterator pit;
			for(pit=domainAttributes.begin(); pit!=domainAttributes.end(); pit++) {
				if(sameKnowledge(*msg, *pit)) {
					ROS_INFO("KCL: (KB) Removing domain attribute (%s)", msg->attribute_name.c_str());
					checkFilters(*msg, false);
					pit = domainAttributes.erase(pit);
					if(pit==domainAttributes.end()) break;
				}
			}

		} else if(msg->knowledge_type == squirrel_planning_knowledge_msgs::KnowledgeItem::INSTANCE_ATTRIBUTE) {

			// remove instance attribute (non-symbolic) from knowledge base
			std::vector<squirrel_planning_knowledge_msgs::KnowledgeItem>::iterator pit;
			for(pit=instanceAttributes[msg->instance_name].begin(); pit!=instanceAttributes[msg->instance_name].end(); pit++) {
				if(sameKnowledge(*msg, *pit)) {
					ROS_INFO("KCL: (KB) Removing instance attribute (%s, %s)", msg->instance_name.c_str(), msg->attribute_name.c_str());
					checkFilters(*msg, false);
					pit = instanceAttributes[msg->instance_name].erase(pit);
					if(pit==instanceAttributes[msg->instance_name].end()) break;
				}
			}
		}
	}

	/*--------------*/
	/* adding items */
	/*--------------*/

	void KnowledgeBase::addKnowledge(const squirrel_planning_knowledge_msgs::KnowledgeItem::ConstPtr& msg) {
		
		if(msg->knowledge_type == squirrel_planning_knowledge_msgs::KnowledgeItem::INSTANCE) {

			// check if instance is already in knowledge base
			std::vector<std::string>::iterator iit;
			iit = find(domainInstances[msg->instance_type].begin(), domainInstances[msg->instance_type].end(), msg->instance_name);

			if(iit==domainInstances[msg->instance_type].end()) {
				// add instance to knowledge base
				ROS_INFO("KCL: (KB) Adding instance (%s, %s)", msg->instance_type.c_str(), msg->instance_name.c_str());
				domainInstances[msg->instance_type].push_back(msg->instance_name);
				checkFilters(*msg, true);
			}

		} else if(msg->knowledge_type == squirrel_planning_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE) {

			// add domain attribute to knowledge base		
			std::vector<squirrel_planning_knowledge_msgs::KnowledgeItem>::iterator pit;
			for(pit=domainAttributes.begin(); pit!=domainAttributes.end(); pit++) {
				if(sameKnowledge(*msg, *pit)) {
					// already added
					return;
				}
			}
			ROS_INFO("KCL: (KB) Adding domain attribute (%s)", msg->attribute_name.c_str());
			domainAttributes.push_back(*msg);
			checkFilters(*msg, true);

		} else if(msg->knowledge_type == squirrel_planning_knowledge_msgs::KnowledgeItem::INSTANCE_ATTRIBUTE) {

			// add instance attribute to knowledge base
			std::vector<squirrel_planning_knowledge_msgs::KnowledgeItem>::iterator pit;
			for(pit=instanceAttributes[msg->instance_name].begin(); pit!=instanceAttributes[msg->instance_name].end(); pit++) {
				if(sameKnowledge(*msg, *pit)) {
					// already added
					return;
				}
			}
			ROS_INFO("KCL: (KB) Adding instance attribute (%s, %s)", msg->instance_name.c_str(), msg->attribute_name.c_str());
			instanceAttributes[msg->instance_name].push_back(*msg);
			checkFilters(*msg, true);
		}
	}

	void KnowledgeBase::addMissionGoal(const squirrel_planning_knowledge_msgs::KnowledgeItem::ConstPtr& msg) {
		// add mission goal to knowledge base
		ROS_INFO("KCL: (KB) Adding mission goal (%s)", msg->attribute_name.c_str());
		domainGoals.push_back(*msg);
	}

	/*----------------*/
	/* fetching items */
	/*----------------*/

	bool KnowledgeBase::getInstances(squirrel_planning_knowledge_msgs::InstanceService::Request  &req, squirrel_planning_knowledge_msgs::InstanceService::Response &res)
	{
		ROS_INFO("KCL: (KB) Sending %s getInstances", req.type_name.c_str());
	
		// fetch the instances of the correct type
		std::map<std::string,std::vector<std::string> >::iterator iit;
		iit = domainInstances.find(req.type_name);
		if(iit != domainInstances.end()) {
			for(size_t j=0; j<iit->second.size(); j++)
				res.instances.push_back(iit->second[j]);
		}

		return true;
	}

	bool KnowledgeBase::getInstanceAttr(squirrel_planning_knowledge_msgs::AttributeService::Request  &req, squirrel_planning_knowledge_msgs::AttributeService::Response &res)
	{
		ROS_INFO("KCL: (KB) Sending getInstanceAttr response for %s %s", req.type_name.c_str(), req.instance_name.c_str());

		// fetch the instances of the correct type
		std::map<std::string,std::vector<squirrel_planning_knowledge_msgs::KnowledgeItem> >::iterator iit;
		iit = instanceAttributes.find(req.instance_name);
		if(iit != instanceAttributes.end()) {
			// check to make sure each knowledge item is of the correct instance type
			for(size_t j=0; j<iit->second.size(); j++) {
				if(iit->second[j].instance_type.compare(req.type_name)==0) {
					res.attributes.push_back(iit->second[j]);
				}
			}
		}

		return true;
	}

	bool KnowledgeBase::getDomainAttr(squirrel_planning_knowledge_msgs::AttributeService::Request  &req, squirrel_planning_knowledge_msgs::AttributeService::Response &res)
	{
		ROS_INFO("KCL: (KB) Sending getDomainAttr response for %s", req.predicate_name.c_str());

		// fetch the knowledgeItems of the correct attribute
		for(size_t i=0; i<domainAttributes.size(); i++) {
			if(req.predicate_name.compare(domainAttributes[i].attribute_name)==0)
				res.attributes.push_back(domainAttributes[i]);
		}

		// ...or fetch the knowledgeItems of the correct function
		for(size_t i=0; i<domainFunctions.size(); i++) {
			if(req.predicate_name.compare(domainFunctions[i].attribute_name)==0)
				res.attributes.push_back(domainFunctions[i]);
		}

		return true;
	}

	bool KnowledgeBase::getCurrentGoals(squirrel_planning_knowledge_msgs::AttributeService::Request  &req, squirrel_planning_knowledge_msgs::AttributeService::Response &res)
	{
		ROS_INFO("KCL: (KB) Sending getCurrentGoals response");

		// fetch the knowledgeItems of the correct attribute
		for(size_t i=0; i<domainGoals.size(); i++) {
			res.attributes.push_back(domainGoals[i]);
		}

		// TESTING for SQUIRREL integration meeting 1/2
		std::vector<std::string>::iterator iit;
		for(iit = domainInstances["object"].begin(); iit!=domainInstances["object"].end(); iit++) {

			bool classified = false;
			std::vector<squirrel_planning_knowledge_msgs::KnowledgeItem>::iterator pit;
			for(pit=domainAttributes.begin(); pit!=domainAttributes.end(); pit++) {
				if(pit->attribute_name.compare("classified")==0 && containsInstance(*pit, *iit))
					classified = true;
			}

			// not classified: add classify
			if(!classified) {
				squirrel_planning_knowledge_msgs::KnowledgeItem goal;
				goal.knowledge_type = squirrel_planning_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE;
				goal.attribute_name = "classified";
				diagnostic_msgs::KeyValue pair;
				pair.key = "o";
				pair.value = *iit;
				goal.values.push_back(pair);
				res.attributes.push_back(goal);
			}

			// not classified: add tidy
			if(!classified) {
				squirrel_planning_knowledge_msgs::KnowledgeItem goal;
				goal.knowledge_type = squirrel_planning_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE;
				goal.attribute_name = "tidy";
				diagnostic_msgs::KeyValue pair;
				pair.key = "o";
				pair.value = *iit;
				goal.values.push_back(pair);
				res.attributes.push_back(goal);
			}

			// classified: move to location
			if(classified) {
				squirrel_planning_knowledge_msgs::KnowledgeItem goal;
				goal.knowledge_type = squirrel_planning_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE;
				goal.attribute_name = "object_at";
				diagnostic_msgs::KeyValue pair;
				pair.key = "o"; pair.value = *iit;
				goal.values.push_back(pair);
				diagnostic_msgs::KeyValue pair1;
				pair1.key = "wp"; pair1.value = "wp3";
				goal.values.push_back(pair1);
				res.attributes.push_back(goal);
			}
		}

		// explore room
		for(iit = domainInstances["waypoint"].begin(); iit!=domainInstances["waypoint"].end(); iit++) {
			
			bool explored = false;
			std::vector<squirrel_planning_knowledge_msgs::KnowledgeItem>::iterator pit;
			for(pit=domainAttributes.begin(); pit!=domainAttributes.end(); pit++) {
				if(pit->attribute_name.compare("explored")==0 && containsInstance(*pit, *iit))
					explored = true;
			}

			// not explored: add classify
			if(!explored) {
				squirrel_planning_knowledge_msgs::KnowledgeItem goal;
				goal.knowledge_type = squirrel_planning_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE;
				goal.attribute_name = "explored";
				diagnostic_msgs::KeyValue pair;
				pair.key = "wp";
				pair.value = *iit;
				goal.values.push_back(pair);
				res.attributes.push_back(goal);
			}
		}
		// END TESTING */

		return true;
	}

} // close namespace

/*-------------*/
/* main method */
/*-------------*/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "KCL_knowledge_base");
	ros::NodeHandle n;

	KCL_rosplan::KnowledgeBase kb;

	// TESTING for SQUIRREL integration meeting 2/2
	{
		// objects
		kb.domainInstances["waypoint"].push_back("wp1");
		kb.domainInstances["waypoint"].push_back("wp2");
		kb.domainInstances["waypoint"].push_back("wp3");
		kb.domainInstances["robot"].push_back("tommy");
		kb.domainInstances["object"].push_back("object0");
		kb.domainInstances["object"].push_back("object1");
		kb.domainInstances["object"].push_back("object2");

		// attributes (1/7) connected
		squirrel_planning_knowledge_msgs::KnowledgeItem can_push_1;
		can_push_1.knowledge_type = squirrel_planning_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE;
		can_push_1.attribute_name = "connected";
		const std::string can_push_keys[] = {"from","to"};
		const std::string values[] = {"wp1","wp2"};
		for(int i=0;i<2;i++) {
			diagnostic_msgs::KeyValue pair;
			pair.key = can_push_keys[i];
			pair.value = values[i];
			can_push_1.values.push_back(pair);
		}
		kb.domainAttributes.push_back(can_push_1);

		squirrel_planning_knowledge_msgs::KnowledgeItem can_push_2;
		can_push_2.knowledge_type = squirrel_planning_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE;
		can_push_2.attribute_name = "connected";
		const std::string values1[] = {"wp2","wp3"};
		for(int i=0;i<2;i++) {
			diagnostic_msgs::KeyValue pair;
			pair.key = can_push_keys[i];
			pair.value = values1[i];
			can_push_2.values.push_back(pair);
		}
		kb.domainAttributes.push_back(can_push_2);

		// attributes (2/7) (classified ?o - object)
		squirrel_planning_knowledge_msgs::KnowledgeItem classified;
		classified.knowledge_type = squirrel_planning_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE;
		classified.attribute_name = "classified";
		diagnostic_msgs::KeyValue classified_pair;
		classified_pair.key = "o";
		classified_pair.value = "object0";
		classified.values.push_back(classified_pair);
		kb.domainAttributes.push_back(classified);

		// attributes (3/6) (explored ?wp - waypoint)
		squirrel_planning_knowledge_msgs::KnowledgeItem explored;
		explored.knowledge_type = squirrel_planning_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE;
		explored.attribute_name = "explored";
		diagnostic_msgs::KeyValue explored_pair;
		explored_pair.key = "wp";
		explored_pair.value = "wp1";
		explored.values.push_back(explored_pair);
		kb.domainAttributes.push_back(explored);

		// attributes (4/6) (object_at ?o - object ?wp - waypoint)
		const std::string object_at_o[] = {"object0","object1","object2"};
		const std::string object_at_wp[] = {"wp1","wp2","wp2"}; 
		for(int i=0;i<3;i++) {		
			squirrel_planning_knowledge_msgs::KnowledgeItem object_at;
			object_at.knowledge_type = squirrel_planning_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE;
			object_at.attribute_name = "object_at";
			diagnostic_msgs::KeyValue pair;
			pair.key = "o"; pair.value = object_at_o[i];
			object_at.values.push_back(pair);
			diagnostic_msgs::KeyValue pair1;
			pair1.key = "wp"; pair1.value = object_at_wp[i];
			object_at.values.push_back(pair1);
			kb.domainAttributes.push_back(object_at);
		}

		// attributes (5/6) (robot_at ?v - robot ?wp - waypoint)
		squirrel_planning_knowledge_msgs::KnowledgeItem robot_at;
		robot_at.knowledge_type = squirrel_planning_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE;
		robot_at.attribute_name = "robot_at";
		diagnostic_msgs::KeyValue robot_at_pair;
		robot_at_pair.key = "v";
		robot_at_pair.value = "tommy";
		robot_at.values.push_back(robot_at_pair);
		diagnostic_msgs::KeyValue robot_at_pair1;
		robot_at_pair1.key = "wp";
		robot_at_pair1.value = "wp1";
		robot_at.values.push_back(robot_at_pair1);
		kb.domainAttributes.push_back(robot_at);

		// attributes (6/6) (tidy ?o - object)
		// none

		// functions (1/1)
		squirrel_planning_knowledge_msgs::KnowledgeItem distance_1;
		distance_1.knowledge_type = squirrel_planning_knowledge_msgs::KnowledgeItem::DOMAIN_FUNCTION;
		distance_1.attribute_name = "distance";
		const std::string distance_keys[] = {"wp1","wp2"};
		for(int i=0;i<2;i++) {
			diagnostic_msgs::KeyValue pair;
			pair.key = distance_keys[i];
			pair.value = values[i];
			distance_1.values.push_back(pair);
		}
		distance_1.function_value = 10.0;
		kb.domainAttributes.push_back(distance_1);

		squirrel_planning_knowledge_msgs::KnowledgeItem distance_2;
		distance_2.knowledge_type = squirrel_planning_knowledge_msgs::KnowledgeItem::DOMAIN_FUNCTION;
		distance_2.attribute_name = "distance";
		for(int i=0;i<2;i++) {
			diagnostic_msgs::KeyValue pair;
			pair.key = distance_keys[i];
			pair.value = values1[i];
			distance_2.values.push_back(pair);
		}
		distance_2.function_value = 10.0;
		kb.domainAttributes.push_back(distance_2);

		// mission filter
		squirrel_planning_knowledge_msgs::KnowledgeItem objectFilter;
		objectFilter.knowledge_type = squirrel_planning_knowledge_msgs::KnowledgeItem::INSTANCE;
		objectFilter.instance_type = "object";
		objectFilter.instance_name = "";
		kb.missionFilter.push_back(objectFilter);
	}
	// END TESTING */

	// environment services
	ros::ServiceServer instanceServer = n.advertiseService("/kcl_rosplan/get_instances", &KCL_rosplan::KnowledgeBase::getInstances, &kb);
	ros::ServiceServer attributeServer = n.advertiseService("/kcl_rosplan/get_instance_attributes", &KCL_rosplan::KnowledgeBase::getInstanceAttr, &kb);
	ros::ServiceServer domainServer = n.advertiseService("/kcl_rosplan/get_domain_attributes", &KCL_rosplan::KnowledgeBase::getDomainAttr, &kb);
	ros::ServiceServer goalServer = n.advertiseService("/kcl_rosplan/get_current_goals", &KCL_rosplan::KnowledgeBase::getCurrentGoals, &kb);

	// filter services
	kb.notificationPublisher = n.advertise<squirrel_planning_knowledge_msgs::Notification>("/kcl_rosplan/notification", 10, true);
	ros::Subscriber filterSub = n.subscribe("/kcl_rosplan/filter", 100, &KCL_rosplan::KnowledgeBase::planningFilterCallback, &kb);

	// knowledge subscribers
	ros::Subscriber addKnowledgeSub = n.subscribe("/kcl_rosplan/add_knowledge", 100, &KCL_rosplan::KnowledgeBase::addKnowledge, &kb);
	ros::Subscriber removeKnowledgeSub = n.subscribe("/kcl_rosplan/remove_knowledge", 100, &KCL_rosplan::KnowledgeBase::removeKnowledge, &kb);
	ros::Subscriber addMissionGoalSub = n.subscribe("/kcl_rosplan/add_mission_goal", 100, &KCL_rosplan::KnowledgeBase::addMissionGoal, &kb);

	ROS_INFO("KCL: (KB) Ready to receive");
	ros::spin();

	return 0;
}
