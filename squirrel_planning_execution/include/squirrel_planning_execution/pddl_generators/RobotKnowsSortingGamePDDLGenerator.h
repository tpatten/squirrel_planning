#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <stdlib.h>
#include <map>
#include <set>
#include <string>
#include <ros/ros.h>

namespace KCL_rosplan {

class RobotKnowsSortingGamePDDLGenerator
{
	private:
		
		/**
		 * Data structure for a type.
		 */
		struct Type
		{
			Type(const std::string& name)
				: name_(name)
			{
				
			}
			
			std::string name_;
		};
		
		/**
		 * Data structure of a location, for each location we store the name, whether it is clear and all the 
		 * locations it is connected to.
		 */
		struct Location
		{
			Location(const std::string& name, bool is_blocked)
				: name_(name), is_blocked_(is_blocked)
			{
				
			}
			
			Location(const std::string& name, const std::vector<const Location*>& connected_locations, bool is_blocked)
				: name_(name), connected_locations_(connected_locations), is_blocked_(is_blocked)
			{
				
			}
			
			std::string name_;
			std::vector<const Location*> connected_locations_;
			std::vector<const Location*> near_locations_;
			bool is_blocked_;
		};
		
		/**
		 * Data structure for a child.
		 */
		struct Child
		{
			Child(const std::string& name, const Location& location)
			: name_(name), location_(&location)
			{
				
			}
			
			std::string name_;
			const Location* location_;
		};

		/**
		 * Data structure of an object, for each object we store the name, the location, and its type.
		 */
		struct Object
		{
			Object(const std::string& name, const Location& location)
				: name_(name), location_(&location), child_(NULL)
			{
				
			}
			
			Object(const std::string& name, const Child& child)
				: name_(name), location_(NULL), child_(&child)
			{
				
			}
			
			std::string name_;
			const Location* location_; // The object is either at a location or held by a child.
			const Child* child_;       // The child that holds the object (if any).
		};
		
		struct Box
		{
			Box(const std::string& name, const Location& location, const std::vector<const Type*>& types_that_fit, const std::vector<const Object*>& objects_inside)
				: name_(name), location_(&location), types_that_fit_(types_that_fit), objects_inside_(objects_inside)
			{
				
			}
			
			std::string name_;
			const Location* location_;
			std::vector<const Type*> types_that_fit_;
			std::vector<const Object*> objects_inside_;
			const Type* type_;
		};
		
		/**
		 * Data structure of a box, for each box we store what type of objects belong in it and where it is.
		 */

		/**
		 * Data structure of a believe state. Each state has a name and it contains for a subset of objects the classification 
		 * attempt at which it will be classified. The subset of objects depends on the factorisation that is performed in the 
		 * knowledge base. In this specific instance each object is part of a seperate Knowledge base which means that each state 
		 * contains only ONE mapping from one object to one location.
		 */
		struct State
		{
			State(const std::string& state_name)
				: state_name_(state_name)
			{
				
			}
			
			State(const std::string& state_name, const std::map<const Object*, const Type*>& type_mapping)
				: state_name_(state_name), type_mapping_(type_mapping)
			{
				
			}
			
			std::string state_name_;
			std::map<const Object*, const Type*> type_mapping_;
		};

		/**
		 * Data structure for a knowledge base. The knowledge bases are created based on the factorisation of the domain. In this 
		 * specific instance each object is part of a seperate Knowledge base. So the number of knowledge bases is #objects + 1, the +1 
		 * is the global knowledge base. Each knowledge base contains a set of states which are the believe states, each state encodes 
		 * from which observation point an object is classifiable.
		 */
		struct KnowledgeBase
		{
			KnowledgeBase(const std::string& name)
				: name_(name)
			{
				
			}
			
			void addChild(const KnowledgeBase& knowledge_base)
			{
				children_.push_back(&knowledge_base);
			}
			
			void addState(State& state)
			{
				states_.push_back(&state);
			}
			
			std::string name_;
			std::vector<const State*> states_;
			std::vector<const KnowledgeBase*> children_;
		};
		
		void generateProblemFile(const std::string& file_name, const KnowledgeBase& current_knowledge_base, const std::vector<const KnowledgeBase*>& knowledge_base, const Location& robot_location, const std::vector<const Location*>& locations, const std::vector<const Object*>& objects, const std::vector<const Box*>& boxes, const std::vector<const Type*>& types, const std::vector<const Child*>& children);

		void generateDomainFile(const std::string& file_name, const KnowledgeBase& current_knowledge_base, const std::vector<const KnowledgeBase*>& knowledge_bases, const Location& robot_location, const std::vector<const Location*>& locations, const std::vector<const Object*>& objects, const std::vector<const Box*>& boxes, const std::vector<const Type*>& types, const std::vector<const Child*>& children);

	public:
		void createPDDL(const std::string& path, const std::string& domain_file, const std::string& problem_file, const std::string& robot_location_predicate, const std::map<std::string, std::string>& object_to_location_mapping, std::map<std::string, std::vector<std::string> >& near_waypoint_mappings, const std::map<std::string, std::string>& object_to_type_mapping, const std::map<std::string, std::string>& box_to_location_mapping, const std::map<std::string, std::string>& box_to_type_mapping, const std::vector<std::string>& children, const std::map<std::string, std::string>& object_to_child_mapping, const std::map<std::string, std::string>& child_location_mapping);
};

};

