#ifndef KCL_ROSPLAN_CONTINGENTPDDLGENERATOR_H
#define KCL_ROSPLAN_CONTINGENTPDDLGENERATOR_H

/**
 * This file defines the RPSquirrelRecursion class.
 * RPSquirrelRecursion is used to execute strategic PDDL actions
 * that correspond to a tactical problem. This is done through
 * instantiating a new planning system node.
 */
namespace KCL_rosplan {

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

	struct Object
	{
		Object(const std::string& name, const Location& location, const std::vector<const Location*>& observable_locations)
			: name_(name), location_(&location), observable_locations_(observable_locations)
		{
			
		}
		
		std::string name_;
		const Location* location_;
		std::vector<const Location*> observable_locations_;
	};

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
	
	class ContingentPDDLGenerator
	{
	private:

		static void generateClassifyObjectTacticalProblem(const std::string& file_name, const KnowledgeBase& current_knowledge_base, const std::vector<const KnowledgeBase*>& knowledge_base, const Location& robot_location, const std::vector<const Location*>& locations, const std::vector<const Object*>& objects);
		static void generateClassifyObjectTacticalDomain(const std::string& file_name, const KnowledgeBase& current_knowledge_base, const std::vector<const KnowledgeBase*>& knowledge_bases, const Location& robot_location, const std::vector<const Location*>& locations, const std::vector<const Object*>& objects);
		
		//static void generateClassifyMultipleObjectsProblem(const std::string& file_name, const KnowledgeBase& current_knowledge_base, const std::vector<const KnowledgeBase*>& knowledge_base, const Location& robot_location, const std::vector<const Location*>& locations, const std::vector<const Object*>& objects);
		//static void generateClassifyMultipleObjectsDomain(const std::string& file_name, const KnowledgeBase& current_knowledge_base, const std::vector<const KnowledgeBase*>& knowledge_bases, const Location& robot_location, const std::vector<const Location*>& locations, const std::vector<const Object*>& objects);

	public:

		static void createClassifyObjectTacticalPDDL(const std::string& path, const std::string& domain_file, const std::string& problem_file, const std::string& robot_location_predicate, const std::vector<std::string>& location_predicates, const std::string& object_predicate, const std::string& object_location_predicate);
		//static void createClassifyMultipleObjectsPDDL(const std::string& path, const std::string& domain_file, const std::string& problem_file, const std::string& robot_location_predicate, const std::vector<std::string>& location_predicates, const std::vector<std::pair<std::string, std::string> >& object_location_predicates);
	};
}
#endif
