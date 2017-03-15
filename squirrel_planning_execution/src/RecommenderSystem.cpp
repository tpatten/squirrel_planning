#include <sstream>
#include <fstream>

#include "squirrel_planning_execution/RecommenderSystem.h"
#include <boost/concept_check.hpp>

#include "squirrel_prediction_msgs/RecommendRelations.h"


namespace KCL_rosplan {

	/**
	 * Types.
	 */
	std::map<std::string, const Type*> Type::generated_types_;
	
	const Type& Type::getType(const std::string& name, const Type* parent)
	{
		const Type* t = NULL;
		std::map<std::string, const Type*>::const_iterator mi = generated_types_.find(name);
		if (mi == generated_types_.end())
		{
			t = new Type(name, parent);
			generated_types_[name] = t;
		}
		else
		{
			t = (*mi).second;
		}
		
		// Sanity check, make sure the parents are the same.
		if (parent != t->parent_)
		{
			ROS_ERROR("The types for %s is %s, but the given parent is %s. Stopping!", name.c_str(), t->parent_ == NULL ? "" : t->parent_->getName().c_str(), parent == NULL ? "" : parent->getName().c_str());
			exit(-1);
		}
		
		return *t;
	}
	
	void Type::cleanup()
	{
		for (std::map<std::string, const Type*>::iterator i = generated_types_.begin(); i != generated_types_.end(); ++i)
		{
			delete (*i).second;
		}
	}
	
	/**
	 * Objects.
	 */
	std::map<std::string, const Object*> Object::generated_objects_;
	
	const Object& Object::getObject(const std::string& name, const Type& type)
	{
		const Object* o = getObject(name);
		if (o == NULL)
		{
			o = new Object(name, type);
			generated_objects_[name] = o;
		}
		
		// Sanity check, make sure the types match.
		if (o->type_ != &type)
		{
			ROS_ERROR("Requested the object %s with type %s, but an existing object with the same name exists that has the type %s. Stopping!", name.c_str(), type.getName().c_str(), o->type_->getName().c_str());
			exit(-1);
		}
		
		return *o;
	}
	
	const Object* Object::getObject(const std::string& name)
	{
		Object* o = NULL;
		std::map<std::string, const Object*>::const_iterator mi = generated_objects_.find(name);
		if (mi == generated_objects_.end())
		{
			return NULL;
			
		}
		return (*mi).second;
	}
	
	void Object::getObjects(const Type& type, std::vector<const Object*>& objects)
	{
		for (std::map<std::string, const Object*>::const_iterator ci = generated_objects_.begin(); ci != generated_objects_.end(); ++ci)
		{
			const Object* o = (*ci).second;
			if (type.isChildOf(*o->type_))
			{
				objects.push_back(o);
			}
		}
	}
	
	void Object::cleanup()
	{
		for (std::map<std::string, const Object*>::iterator i = generated_objects_.begin(); i != generated_objects_.end(); ++i)
		{
			delete (*i).second;
		}
	}
	
	/**
	 * Predicates.
	 */
	std::map<std::string, const Predicate*> Predicate::generated_predicates_;
	
	/**
	 * Get a predicate, all predicates are stored in a global cache.
	 */
	const Predicate& Predicate::getPredicate(const std::string& name, const std::vector<const Type*>& types)
	{
		const Predicate* p = getPredicate(name);
		if (p == NULL)
		{
			p = new Predicate(name, types);
			generated_predicates_[name] = p;
		}
		
		// Sanity check, it's illegal in PDDL to have predicates with the same name but seperate arity. The types
		// must also match
		if (types.size() != p->types_.size())
		{
			ROS_ERROR("Predicate detected with different arity! Predicate: %s, expected arity %lu but given arity %lu. Stopping!\n", name.c_str(), p->types_.size(), types.size());
			exit(-1);
			for (unsigned int i = 0; i < types.size(); ++i)
			{
				if (types[i] != p->types_[i])
				{
					ROS_ERROR("Predicate detected with different types! Predicate: %s, has type %s at index %ud but given type %s. Stopping!\n", name.c_str(), p->types_[i]->getName().c_str(), i, types[i]->getName().c_str());
					exit(-1);
				}
			}
		}
		return *p;
	}
	
	/**
		* Get a predicate, if a predicate with the given name does not exist, a new one is not generated!
		*/
	const Predicate* Predicate::getPredicate(const std::string& name)
	{
		Predicate* p = NULL;
		std::map<std::string, const Predicate*>::const_iterator mi = generated_predicates_.find(name);
		if (mi == generated_predicates_.end())
		{
			return NULL;
		}
		
		return (*mi).second;
	}
	
	void Predicate::ground(std::vector<const Fact*>& facts) const
	{
		unsigned int counter[getArity()];
		std::vector<std::vector<const Object*> > objects;
		for (unsigned int i = 0; i < getArity(); ++i)
		{
			counter[i] = 0;
			
			std::vector<const Object*> objects_of_type;
			Object::getObjects(*types_[i], objects_of_type);
			objects.push_back(objects_of_type);
		}
		
		bool done = false;
		while (!done)
		{
			done = true;
			std::vector<const Object*> fact_objects;
			for (unsigned int i = 0; i < getArity(); ++i)
			{
				fact_objects.push_back(objects[i][counter[i]]);
			}
			facts.push_back(&Fact::getFact(*this, fact_objects));
			
			for (unsigned int i = 0; i < getArity(); ++i)
			{
				if (counter[i] + 1 == objects[i].size())
				{
					counter[i] = 0;
				}
				else 
				{
					counter[i] = counter[i] + 1;
					done = false;
					break;
				}
			}
		}
	}
	
	void Predicate::cleanup()
	{
		for (std::map<std::string, const Predicate*>::iterator i = generated_predicates_.begin(); i != generated_predicates_.end(); ++i)
		{
			delete (*i).second;
		}
	}
	
	/**
	 * Facts.
	 */
	std::map<std::pair<const Predicate*, std::vector<const Object*> >, const Fact*> Fact::generated_facts_;
	
	
	const Fact& Fact::getFact(const Predicate& predicate, const std::vector<const Object*>& objects)
	{
		// Sanity check, make sure the number of objects matches the predicate's arity.
		if (objects.size() != predicate.getArity())
		{
			ROS_ERROR("We cannot create a fact with %lu facts, because the predicate %s has an arity of %u. Stopping!\n", objects.size(), predicate.getName().c_str(), predicate.getArity());
			exit(-1);
		}
		
		const Fact* f = NULL;
		std::map<std::pair<const Predicate*, std::vector<const Object*> >, const Fact*>::const_iterator mi = generated_facts_.find(std::make_pair(&predicate, objects));
		if (mi == generated_facts_.end())
		{
			f = new Fact(predicate, objects);
			generated_facts_[std::make_pair(&predicate, objects)] = f;
			return *f;
		}
		f = (*mi).second;
		return *f;
	}
	
	/**
	 * Delete all objects ever created.
	 */
	void Fact::cleanup()
	{
		for (std::map<std::pair<const Predicate*, std::vector<const Object*> >, const Fact*>::iterator i = generated_facts_.begin(); i != generated_facts_.end(); ++i)
		{
			delete (*i).second;
		}
	}
	
	std::ostream& operator<<(std::ostream& os, const Fact& fact)
	{
		os << "(" << fact.getPredicate().getName();
		for (std::vector<const Object*>::const_iterator ci = fact.getObjects().begin(); ci != fact.getObjects().end(); ++ci)
		{
			os << " " << (*ci)->getName();
		}
		os << ")";
		return os;
	}
	
	/*-------------*/
	/* constructor */
	/*-------------*/

	RecommenderSystem::RecommenderSystem(ros::NodeHandle &nh)
		: node_handle(&nh)//, message_store(nh)
	{
		// knowledge interface
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		query_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>("/kcl_rosplan/query_knowledge_base");
		
		get_instance_client = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
		get_attribute_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
		
		// Recommender interface.
		recommender_client = nh.serviceClient<squirrel_prediction_msgs::RecommendRelations>("/squirrel_relations_prediction");
		
		// Load file parameters.
		nh.getParam("/squirrel_planning_execution/recommender/data_path", data_path);
		nh.getParam("/squirrel_planning_execution/recommender/input_file", input_file);
		nh.getParam("/squirrel_planning_execution/recommender/output_file", output_file);
		
		std::stringstream ss;
		ss << data_path << "/" << input_file;
		absolute_input_file = ss.str();
		ss.str(std::string());
		ss << data_path << "/" << output_file;
		absolute_output_file = ss.str();
		
		ROS_INFO("KCL: (RecommenderSystem) Running! Input file: %s; Output file: %s.\n",  absolute_input_file.c_str(), absolute_output_file.c_str());
	}
	
	RecommenderSystem::~RecommenderSystem()
	{
		Fact::cleanup();
		Predicate::cleanup();
		Object::cleanup();
	}
	
	void RecommenderSystem::writeCSV(const std::vector<const Object*>& objects, const std::vector<const Predicate*>& predicates, const std::map<const Fact*, float>& weighted_facts)
	{
		ROS_INFO("KCL: (RecommenderSystem) Write CSV.\n");
		std::ofstream ofs;
		ofs.open(absolute_input_file.c_str());
		
		// Write header.
		ofs << "'object1','object2'";
		for (std::vector<const Predicate*>::const_iterator ci = predicates.begin(); ci != predicates.end(); ++ci)
		{
			ofs << ",'" << (*ci)->getName() << "'";
		}
		ofs << "," << std::endl;
		
		for (std::vector<const Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
		{
			const Object* o1 = *ci;
			for (std::vector<const Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
			{
				const Object* o2 = *ci;
				ofs << "'" << o1->getName() << "'" << ",'" << o2->getName() << "'";
				
				for (std::vector<const Predicate*>::const_iterator ci = predicates.begin(); ci != predicates.end(); ++ci)
				{
					const Predicate* predicate = *ci;
					
					// Try to create a fact with this predicate and the given objects. If this is impossible, mark it as
					// a '0' and move on.
					std::vector<const Object*> objects;
					if (predicate->getArity() > 0)
					{
						if (o1->getType().isChildOf(*predicate->getTypes()[0]))
						{
							objects.push_back(o1);
						}
						else
						{
							ofs <<",";
							continue;
						}
					}
					if (predicate->getArity() > 1)
					{
						if (o2->getType().isChildOf(*predicate->getTypes()[1]))
						{
							objects.push_back(o2);
						}
						else
						{
							ofs <<",";
							continue;
						}
					}
					
					// A fact can be created given these objects.
					const Fact& fact = Fact::getFact(*predicate, objects);
					std::map<const Fact*, float>::const_iterator mi = weighted_facts.find(&fact);
					
					// Check if we know the probability of this fact being true, otherwise leave it empty.
					if (mi == weighted_facts.end())
					{
						ofs << ",0";
					}
					else
					{
						ofs << "," << mi->second;
					}
				}
				ofs << "," << std::endl;
			}
		}
		ofs.close();
		ROS_INFO("KCL: (RecommenderSystem) Finished writing CVS.\n");
	}
		
	void RecommenderSystem::readCSV(std::map<const Fact*, float>& results, const std::vector<const Predicate*>& predicates)
	{
		ROS_INFO("KCL: (RecommenderSystem) Read CSV.\n");
		// Read the header.
		std::ifstream file(absolute_output_file.c_str());
		std::string line;
		unsigned int line_nr = 0;
		unsigned int max_arity = 0;
		
		// Read the header.
		//std::vector<const Predicate*> predicates;
		if (std::getline(file, line))
		{
			ROS_INFO("Process the line: %s\n", line.c_str());
			std::vector<std::string> tokens = split(line, ',');
			
			// Skip the first few tokens as they refer to the object names.
			for (unsigned int i = 0; i < tokens.size(); ++i)
			{
				if (tokens[i].size() == 0)
				{
					continue;
				}
				
				const std::string& name = tokens[i].substr(1, tokens[i].size() - 2);
				ROS_INFO("Process the token: %s\n", name.c_str());
				std::string name_trimmed = trimCopy(name);
				if (name_trimmed.substr(0, 6) == "object" && name_trimmed.size() > 6 && 
				   (::atoi(name_trimmed.substr(6).c_str()) != 0 || name_trimmed == "object0"))
				{
					++max_arity;
					ROS_INFO("Object founded %s.\n", name_trimmed.c_str());
					continue;
				}
				
				const Predicate* p = Predicate::getPredicate(name_trimmed);
				if (p == NULL)
				{
					ROS_ERROR("Could not find the predicate: %s. Stopping!", name_trimmed.c_str());
					exit(-1);
				}
				//predicates.push_back(p);
			}
		}
		
		// Read the properties and store the confidence values.
		while (std::getline(file, line))
		{
			ROS_INFO("Process the line: %s\n", line.c_str());
			std::vector<std::string> tokens = split(line, ',');
			std::vector<const Object*> objects;
			
			// Fetch the objects from the file.
			for (unsigned int i = 0; i < max_arity; ++i)
			{
				if (tokens[i].size() == 0)
				{
					continue;
				}
				
				const std::string& name = tokens[i].substr(1, tokens[i].size() - 2);
				std::string name_trimmed = trimCopy(name);
				
				// Find the object with the given name.
				const Object* o = Object::getObject(name_trimmed);
				if (o == NULL)
				{
					ROS_ERROR("Could not find the object: %s. Stopping!", name_trimmed.c_str());
					exit(-1);
				}
				ROS_INFO("Added the object: %s.", name_trimmed.c_str());
				objects.push_back(o);
			}
			
			std::cout << "Create the facts: " << predicates.size() << ", " << max_arity << ", " << tokens.size() << std::endl;
			
			// Create the facts and read the values.
			for (unsigned int i = max_arity + predicates.size(); i < predicates.size() + predicates.size() + max_arity; ++i)
			{
				float confidence = ::atof(tokens[i].c_str());
				const Predicate* p = predicates[i - max_arity - predicates.size()];
				std::vector<const Object*> o;
				for (unsigned int j = 0; j < p->getArity(); ++j)
				{
					o.push_back(objects[j]);
				}
				
				const Fact& fact = Fact::getFact(*p, o);
				results[&fact] = confidence;
				
				std::cout << fact << "; p=" << confidence << std::endl;
			}
		}
	}
	
	std::vector<const Fact*> RecommenderSystem::getBestSensingActions(const std::vector<const Object*>& objects, const std::vector<const Predicate*>& predicates, const std::map<const Fact*, float>& weighted_facts, const std::vector<const Fact*>& interesting_facts)                                            
	{
		ROS_INFO("KCL: (RecommenderSystem) Get best sensing actions.");
		writeCSV(objects, predicates, weighted_facts);
		
		squirrel_prediction_msgs::RecommendRelations rr;
		rr.request.data_path = data_path;
		rr.request.input_file = input_file;
		rr.request.output_file = output_file;
		rr.request.number_of_columns = predicates.size();
		
		if (!recommender_client.call(rr))
		{
			ROS_ERROR("Could not call the recommender system. Stopping!\n");
			exit(-1);
		}
		
		if (!rr.response.finished)
		{
			ROS_ERROR("The recommender system did not reported it finished, something went wrong? Stopping!\n");
			exit(-1);
		}
		
		std::map<const Fact*, float> results;
		readCSV(results, predicates);
		
		std::vector<const Fact*> facts_to_sense;
		for (std::map<const Fact*, float>::const_iterator ci = results.begin(); ci != results.end(); ++ci)
		{
			const Fact* fact = (*ci).first;
			float p = (*ci).second;
			
			//ROS_INFO("Best fact: %s, p=%f\n", fact->getPredicate().getName().c_str(), p);
			if (p < 0.95f && p > 0.2f)
			{
				facts_to_sense.push_back(fact);
			}
		}
		return facts_to_sense;
	}
} // close namespace

int main(int argc, char** argv)
{
	ROS_INFO("KCL: (RecommenderSystem) Started!\n");
	
	/* Types */
	const KCL_rosplan::Type object = KCL_rosplan::Type::getType("object", NULL);
	const KCL_rosplan::Type catagory = KCL_rosplan::Type::getType("catagory", NULL);
	const KCL_rosplan::Type robot = KCL_rosplan::Type::getType("robot", NULL);
	const KCL_rosplan::Type box = KCL_rosplan::Type::getType("box", NULL);
	const KCL_rosplan::Type location = KCL_rosplan::Type::getType("location", NULL);
	
	ROS_INFO("KCL: (RecommenderSystem) Types created.\n");
	
	/* Objects */
	std::vector<const KCL_rosplan::Object*> objects;
	objects.push_back(&KCL_rosplan::Object::getObject("toy1", object));
	objects.push_back(&KCL_rosplan::Object::getObject("toy2", object));
	objects.push_back(&KCL_rosplan::Object::getObject("kenny", robot));
	objects.push_back(&KCL_rosplan::Object::getObject("car", catagory));
	objects.push_back(&KCL_rosplan::Object::getObject("dinosaur", catagory));
	objects.push_back(&KCL_rosplan::Object::getObject("box1", box));
	objects.push_back(&KCL_rosplan::Object::getObject("box2", box));
	objects.push_back(&KCL_rosplan::Object::getObject("wp1", location));
	objects.push_back(&KCL_rosplan::Object::getObject("wp2", location));
	objects.push_back(&KCL_rosplan::Object::getObject("wp3", location));
	objects.push_back(&KCL_rosplan::Object::getObject("wp4", location));
	objects.push_back(&KCL_rosplan::Object::getObject("wp5", location));
	ROS_INFO("KCL: (RecommenderSystem) Objects created %zd.\n", objects.size());
	
	/* Predicates */
	std::vector<const KCL_rosplan::Predicate*> predicates;
	std::vector<const KCL_rosplan::Type*> robot_at_types;
	robot_at_types.push_back(&robot);
	robot_at_types.push_back(&location);
	predicates.push_back(&KCL_rosplan::Predicate::getPredicate("robot_at", robot_at_types));
	
	std::vector<const KCL_rosplan::Type*> object_at_types;
	object_at_types.push_back(&object);
	object_at_types.push_back(&location);
	predicates.push_back(&KCL_rosplan::Predicate::getPredicate("object_at", object_at_types));
	
	std::vector<const KCL_rosplan::Type*> box_at_types;
	box_at_types.push_back(&box);
	box_at_types.push_back(&location);
	predicates.push_back(&KCL_rosplan::Predicate::getPredicate("box_at", box_at_types));
	
	std::vector<const KCL_rosplan::Type*> connected_types;
	connected_types.push_back(&location);
	connected_types.push_back(&location);
	predicates.push_back(&KCL_rosplan::Predicate::getPredicate("connected", connected_types));
	
	std::vector<const KCL_rosplan::Type*> in_types;
	in_types.push_back(&object);
	in_types.push_back(&box);
	predicates.push_back(&KCL_rosplan::Predicate::getPredicate("in", in_types));
	
	std::vector<const KCL_rosplan::Type*> belongs_in_types;
	belongs_in_types.push_back(&object);
	belongs_in_types.push_back(&box);
	predicates.push_back(&KCL_rosplan::Predicate::getPredicate("belongs_in", belongs_in_types));
	ROS_INFO("KCL: (RecommenderSystem) Predicats created %zd.\n", predicates.size());
	
	/* Facts */
	std::vector<const KCL_rosplan::Fact*> all_facts;
	for (std::vector<const KCL_rosplan::Predicate*>::const_iterator ci = predicates.begin(); ci != predicates.end(); ++ci)
	{
		const KCL_rosplan::Predicate* predicate = *ci;
		predicate->ground(all_facts);
	}
	ROS_INFO("KCL: (RecommenderSystem) Facts created %zd.\n", all_facts.size());
	
	for (std::vector<const KCL_rosplan::Fact*>::const_iterator ci = all_facts.begin(); ci != all_facts.end(); ++ci)
	{
		std::cout << **ci << std::endl;
	}
	
	/* Mark all facts that are valid with -1 (uncertain). */
	std::map<const KCL_rosplan::Fact*, float> weighted_facts;
	for (std::vector<const KCL_rosplan::Fact*>::const_iterator ci = all_facts.begin(); ci != all_facts.end(); ++ci)
	{
		float p =((float)rand() / (float)RAND_MAX) * 2.0f - 0.5f;
		
		if (p < 0)
			weighted_facts[*ci] = -1;
		else 
			weighted_facts[*ci] = 1.0;
	}
	ROS_INFO("KCL: (RecommenderSystem) Weighted facts created.\n");
	
	ros::init(argc, argv, "rosplan_RecommanderSystem");
	ros::NodeHandle nh;
	
	KCL_rosplan::RecommenderSystem rs(nh);
	
	ROS_INFO("KCL: (RecommenderSystem) Recommender System started.\n");
	
	std::vector<const KCL_rosplan::Fact*> facts_to_sense = rs.getBestSensingActions(objects, predicates, weighted_facts, all_facts);
	
	while (ros::ok())
	{
		ros::spinOnce();
	}
	return 0;
}
