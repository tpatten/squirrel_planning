#ifndef KCL_ROSPLAN_CONTINGENTTIDIPDDLGENERATOR_H
#define KCL_ROSPLAN_CONTINGENTTIDIPDDLGENERATOR_H

/**
 * Creates a PDDL domain and problem file that models the plan to sense domain. It gets its information 
 * from the recommender system that suggests a sequence of observation actions to perform. Before such 
 * action can be performed, we need to find these objects.
 */
namespace KCL_rosplan {

	
	class PlanToSensePDDLGenerator
	{
	private:
		
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
			
			Location(const std::string& name, bool is_blocked, const std::vector<const Location*>& connected_locations)
				: name_(name), is_blocked_(is_blocked), connected_locations_(connected_locations)
			{
				
			}
			
			std::string name_;
			bool is_blocked_;
			std::vector<const Location*> connected_locations_;
			std::vector<const Location*> near_locations_;
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
			
			std::string name_;
			const Location* location_;
		};
		
		struct Box
		{
			Box(const std::string& name, const Location& location, const std::vector<const Object*>& objects_inside)
				: name_(name), location_(&location)
			{
				
			}
			
			std::string name_;
			const Location* location_;
			std::vector<const Object*> objects_inside_;
		};
		
		struct TreeNode
		{
			TreeNode(const Box& box, const Object& object, TreeNode* parent)
				: box_(&box), object_(&object), parent_(parent), false_branch_(NULL), true_branch_(NULL)
			{
				
			}
			
			const Box* box_;       // The box to put the object in.
			const Object* object_; // The object to put in a box.
			
			TreeNode* parent_;       // Parent node.
			TreeNode* false_branch_; // The false branch.
			TreeNode* true_branch_;  // The true branch.
		};
		
		/**
		 * Data structure of a believe state. Each state has a name and it contains for a subset of objects the location from 
		 * where it will be classified. The subset of objects depends on the factorisation that is performed in the knowledge base.
		 * In this specific instance each object is part of a seperate Knowledge base which means that each state contains only ONE 
		 * mapping from one object to one location.
		 */
		struct State
		{
			State(const std::string& state_name, const std::vector<const TreeNode*>& sense_sequence, const std::map<const Object*, const Box*>& believe_state)
				: state_name_(state_name), sense_sequence_(sense_sequence), believe_state_(believe_state)
			{
				
			}
			
			std::string state_name_;                             // The unique name of this state.
			std::vector<const TreeNode*> sense_sequence_;        // The sequence of observations that need to be made.
			std::map<const Object*, const Box*> believe_state_; // The believe state.
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
		 * @param boxes All the boxes that exist in the domain.
		 * @param types All the types that exist in the domain.
		 */
		static void generateProblemFile(const std::string& file_name, const KnowledgeBase& current_knowledge_base, const std::vector<const KnowledgeBase*>& knowledge_bases, const Location& robot_location, const std::vector<Location*>& locations, const std::vector<Object*>& objects, const std::vector<const Box*>& boxes, const TreeNode& root);
		
		/**
		 * Generate the domain file given the set of objects, locations, etc.
		 * @param file_name The path and file name of where the domain file should be written to.
		 * @param current_knowledge_base The global knowledge base that is active in the initial state.
		 * @param knowledge_bases All the knowledge bases the planning problem is factorised in.
		 * @param robot_location The initial location of the robot.
		 * @param locations All the locations that exist in the domain.
		 * @param objects All the objects that exist in the domain that must be classified.
		 * @param boxes All the boxes that exist in the domain.
		 * @param types All the types that exist in the domain.
		 */
		static void generateDomainFile(const std::string& file_name, const KnowledgeBase& current_knowledge_base, const std::vector<const KnowledgeBase*>& knowledge_bases, const Location& robot_location, const std::vector<Location*>& locations, const std::vector<Object*>& objects, const std::vector<const Box*>& boxes, const TreeNode& root);

	public:
		
		/**
		 * Create the PDDL domain and problem file, the name of the domain file is @ref{path}/@ref{domain_file} and the 
		 * name of the problem file is @ref{path}/@ref{problem_path}.
		 * @param path The path where the domain and problem files are stored.
		 * @param domain_file The name of the PDDL file where the domain is stored.
		 * @param problem_file The name of the PDDL file where the problem is stored.
		 * @param robot_location_predicate The predicate name of the waypoint where the robot is.
		 * @param object_to_location_mapping A mapping for each object predicate to the location predicate where it is located.
		 * @param object_to_type_mapping A mapping for each object predicate to the type predicate 
		 * @param box_to_location_mapping A mapping for each box to its location predicate.
		 * @param box_to_type_mapping A mapping for each box to the type predicate of each object type it can contain.
		 * @param near_box_location_mapping A mapping from box location to location near it.
		 */
		static void createPDDL(const std::string& path, const std::string& domain_file, const std::string& problem_file, const std::string& robot_location_predicate, const std::map<std::string, std::string>& object_to_location_mapping, const std::map<std::string, std::string>& box_to_location_mapping);
	};
}
#endif
