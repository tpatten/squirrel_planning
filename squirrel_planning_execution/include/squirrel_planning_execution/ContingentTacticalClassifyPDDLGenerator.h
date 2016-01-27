#ifndef KCL_ROSPLAN_CONTINGENTTACTICALCLASSIFYPDDLGENERATOR_H
#define KCL_ROSPLAN_CONTINGENTTACTICALCLASSIFYPDDLGENERATOR_H

/**
 * Creates a PDDL domain and problem file that models the classification task on the tactical level. This 
 * means that a single object has been selected for classification, that the area around that object is 
 * clear and we have a set of observation locations from which we want to observe the object from.
 * 
 * The generated problem instances is solvable by FF (or it dies trying!) and a solution to this problem 
 * is for the robot to visit each observation waypoint, perform the classification action, and solve the 
 * two branches that emerge after the observation:
 * - If the classification succeeded we are done!
 * - If the classification failed, we go to the next waypoint and try again.
 * If all possible classification actions have been performed there is a special action that terminates the 
 * failed branch and finishes the plan.
 */
namespace KCL_rosplan {

	
	class ContingentTacticalClassifyPDDLGenerator
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
			bool is_clear_;
		};

		/**
		 * Data structure of an object, for each object we store the name, the location at which it is stored, and all 
		 * locations from where it is classifiable.
		 */
		struct Object
		{
			Object(const std::string& name, const Location& location)
				: name_(name), location_(&location)
			{
				
			}
			
			Object(const std::string& name, const Location& location, const std::vector<const Location*>& observable_locations)
				: name_(name), location_(&location), observable_locations_(observable_locations)
			{
				
			}
			
			std::string name_;
			const Location* location_;
			std::vector<const Location*> observable_locations_;
		};

		/**
		 * Data structure of a believe state. Each state has a name and it contains for a subset of objects the location from 
		 * where it will be classified. The subset of objects depends on the factorisation that is performed in the knowledge base.
		 * In this specific instance each object is part of a seperate Knowledge base which means that each state contains only ONE 
		 * mapping from one object to one location.
		 */
		struct State
		{
			State(const std::string& state_name)
				: state_name_(state_name)
			{
				
			}
			
			State(const std::string& state_name, const std::map<const Object*, const Location*>& classifiable_from)
				: state_name_(state_name), classifiable_from_(classifiable_from)
			{
				
			}
			
			std::string state_name_;
			std::map<const Object*, const Location*> classifiable_from_;
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
		
		/**
		 * Generate the problem file given the set of objects, locations, etc.
		 * @param file_name The path and file name of where the problem file should be written to.
		 * @param current_knowledge_base The global knowledge base that is active in the initial state.
		 * @param knowledge_bases All the knowledge bases the planning problem is factorised in.
		 * @param robot_location The initial location of the robot.
		 * @param locations All the locations that exist in the domain.
		 * @param objects All the objects that exist in the domain that must be classified.
		 */
		static void generateProblemFile(const std::string& file_name, const KnowledgeBase& current_knowledge_base, const std::vector<const KnowledgeBase*>& knowledge_bases, const Location& robot_location, const std::vector<Location*>& locations, const std::vector<Object*>& objects);
		
		/**
		 * Generate the domain file given the set of objects, locations, etc.
		 * @param file_name The path and file name of where the domain file should be written to.
		 * @param current_knowledge_base The global knowledge base that is active in the initial state.
		 * @param knowledge_base All the knowledge bases the planning problem is factorised in.
		 * @param robot_location The initial location of the robot.
		 * @param locations All the locations that exist in the domain.
		 * @param objects All the objects that exist in the domain that must be classified.
		 */
		static void generateDomainFile(const std::string& file_name, const KnowledgeBase& current_knowledge_base, const std::vector<const KnowledgeBase*>& knowledge_bases, const Location& robot_location, const std::vector<Location*>& locations, const std::vector<Object*>& objects);

	public:
		
		/**
		 * Create the PDDL domain and problem file, the name of the domain file is @ref{path}/@ref{domain_file} and the 
		 * name of the problem file is @ref{path}/@ref{problem_path}.
		 * @param path The path where the domain and problem files are stored.
		 * @param domain_file The name of the PDDL file where the domain is stored.
		 * @param problem_file The name of the PDDL file where the problem is stored.
		 * @param robot_location_predicate The predicate name of the waypoint where the robot is.
		 * @param location_predicates The predicates of all waypoints in the domain (excluding the location of the robot and object!).
		 * @param object_predicate The predicate name of the object.
		 * @param object_location_predicate The predicate of the waypoint where the object is.
		 */
		static void createPDDL(const std::string& path, const std::string& domain_file, const std::string& problem_file, const std::string& robot_location_predicate, const std::vector<std::string>& location_predicates, const std::string& object_predicate, const std::string& object_location_predicate);
	};
}
#endif
