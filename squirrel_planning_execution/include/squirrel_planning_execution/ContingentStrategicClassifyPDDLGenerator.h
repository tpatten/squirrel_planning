#ifndef KCL_ROSPLAN_CONTINGENTSTRATEGICCLASSIFYPDDLGENERATOR_H
#define KCL_ROSPLAN_CONTINGENTSTRATEGICCLASSIFYPDDLGENERATOR_H

/**
 * This file defines the RPSquirrelRecursion class.
 * RPSquirrelRecursion is used to execute strategic PDDL actions
 * that correspond to a tactical problem. This is done through
 * instantiating a new planning system node.
 */
namespace KCL_rosplan {

	
	class ContingentStrategicClassifyPDDLGenerator
	{
	private:
		
		/**
		 * Data structure of a location, for each location we store the name, whether it is clear and all the 
		 * locations it is connected to.
		 */
		struct Location
		{
			Location(const std::string& name, bool is_clear)
				: name_(name), is_clear_(is_clear)
			{
				
			}
			
			Location(const std::string& name, const std::vector<const Location*>& connected_locations, bool is_clear)
				: name_(name), connected_locations_(connected_locations), is_clear_(is_clear)
			{
				
			}
			
			std::string name_;
			std::vector<const Location*> connected_locations_;
			std::vector<const Location*> near_locations_;
			bool is_clear_;
		};

		/**
		 * Data structure of an object, for each object we store the name and the location at which it is stored.
		 */
		struct Object
		{
			Object(const std::string& name, const Location& location)
				: name_(name), location_(&location)
			{
				
			}
			
			std::string name_;
			const Location* location_;
		};

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
			
			State(const std::string& state_name, const std::map<const Object*, unsigned int>& classifiable_at_attempt)
				: state_name_(state_name), classifiable_at_attempt_(classifiable_at_attempt)
			{
				
			}
			
			std::string state_name_;
			std::map<const Object*, unsigned int> classifiable_at_attempt_;
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
		
		static void generateProblemFile(const std::string& file_name, const KnowledgeBase& current_knowledge_base, const std::vector<const KnowledgeBase*>& knowledge_base, const Location& robot_location, const std::vector<Location*>& locations, const std::vector<Object*>& objects, unsigned int max_classification_attemps);
		static void generateDomainFile(const std::string& file_name, const KnowledgeBase& current_knowledge_base, const std::vector<const KnowledgeBase*>& knowledge_bases, const Location& robot_location, const std::vector<Location*>& locations, const std::vector<Object*>& objects, unsigned int max_classification_attemps);

	public:

		/**
		 * Create the PDDL domain and problem file, the name of the domain file is @ref{path}/@ref{domain_file} and the 
		 * name of the problem file is @ref{path}/@ref{problem_path}.
		 * @param path The path where the domain and problem files are stored.
		 * @param domain_file The name of the PDDL file where the domain is stored.
		 * @param problem_file The name of the PDDL file where the problem is stored.
		 * @param robot_location_predicate The predicate name of the waypoint where the robot is.
		 * @param object_location_predicates A mapping from the predicates of each object to the predicate of the waypoint where it is located.
		 * @param near_waypoint_mapping Mapping of waypoints to waypoint that are near each other, allowing grasping, dropping, and pushing operations.
		 * @param max_classification_attemps The maximum number of classification attemps we allow per object before giving up.
		 */
		static void createPDDL(const std::string& path, const std::string& domain_file, const std::string& problem_file, const std::string& robot_location_predicate, const std::map<std::string, std::string>& object_location_predicates, const std::map<std::string, std::vector<std::string> >& near_waypoint_mapping, unsigned int max_classification_attemps);
	};
}
#endif
