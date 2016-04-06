#ifndef KCL_ROSPLAN_CLASSICALTIDIPDDLGENERATOR_H
#define KCL_ROSPLAN_CLASSICALTIDIPDDLGENERATOR_H

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

	
	class ClassicalTidyPDDLGenerator
	{
	private:
		
		/**
		 * Generate the problem file given the set of objects, locations, etc.
		 * @param file_name The path and file name of where the problem file should be written to.
		 * @param robot_location_predicate The initial location of the robot.
		 * @param object_to_location_mapping Mappings of the objects to the locations of where they are.
		 * @param near_location_mapping Mappings of locations to locations that are near them.
		 * @param object_to_type_mapping Mappings of the objects to their types.
		 * @param box_to_location_mapping Mappings of the boxes to their locations.
		 * @param box_to_type_mapping Mappings of the boxes to their types
		 * @param box_to_location_mapping A mapping for each box to its location predicate.
		 */
		static void generateProblemFile(const std::string& file_name, const std::string& robot_location_predicate, const std::map<std::string, std::string>& object_to_location_mapping, const std::map<std::string, std::vector<std::string > >& near_location_mapping, const std::map<std::string, std::string>& object_to_type_mapping, const std::map<std::string, std::string>& box_to_location_mapping, const std::map<std::string, std::string>& box_to_type_mapping, const std::map<std::string, std::vector<std::string> >& near_box_location_mapping);
		
		/**
		 * Generate the domain file given the set of objects, locations, etc.
		 * @param file_name The path and file name of where the domain file should be written to.
		 * @param robot_location_predicate The initial location of the robot.
		 * @param object_to_location_mapping Mappings of the objects to the locations of where they are.
		 * @param near_location_mapping Mappings of locations to locations that are near them.
		 * @param object_to_type_mapping Mappings of the objects to their types.
		 * @param box_to_location_mapping Mappings of the boxes to their locations.
		 * @param box_to_type_mapping Mappings of the boxes to their types.
		 * @param box_to_location_mapping A mapping for each box to its location predicate.
		 */
		static void generateDomainFile(const std::string& file_name, const std::string& robot_location_predicate, const std::map<std::string, std::string>& object_to_location_mapping, const std::map<std::string, std::vector<std::string > >& near_location_mapping, const std::map<std::string, std::string>& object_to_type_mapping, const std::map<std::string, std::string>& box_to_location_mapping, const std::map<std::string, std::string>& box_to_type_mapping, const std::map<std::string, std::vector<std::string> >& near_box_location_mapping);

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
		 * @param box_to_type_mapping A mapping for each box to the type predicate of each object type it can contain.
		 * @param box_to_location_mapping A mapping for each box to its location predicate.
		 */
		static void createPDDL(const std::string& path, const std::string& domain_file, const std::string& problem_file, const std::string& robot_location_predicate, const std::map<std::string, std::string>& object_to_location_mapping, const std::map<std::string, std::vector<std::string > >& near_location_mapping, const std::map<std::string, std::string>& object_to_type_mapping, const std::map<std::string, std::string>& box_to_location_mapping, const std::map<std::string, std::string>& box_to_type_mapping, const std::map<std::string, std::vector<std::string> >& near_box_location_mapping);
	};
}
#endif
