#include <sstream>
#include <fstream>
#include <queue>

#include <actionlib/client/simple_action_client.h>
#include <rosplan_planning_system/PlanningEnvironment.h>

#include <rosplan_knowledge_msgs/DomainFormula.h>
#include <rosplan_knowledge_msgs/KnowledgeItem.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_knowledge_msgs/GenerateProblemService.h>
#include <rosplan_knowledge_msgs/KnowledgeQueryService.h>
#include <rosplan_knowledge_msgs/GetDomainTypeService.h>
#include <rosplan_knowledge_msgs/GetDomainAttributeService.h>
#include <rosplan_dispatch_msgs/PlanAction.h>

#include <rosplan_knowledge_msgs/GenerateProblemService.h>

#include "pddl_actions/ListenToFeedbackPDDLAction.h"
#include "squirrel_planning_execution/RecommenderSystem.h"
#include "squirrel_prediction_msgs/RecommendRelations.h"
#include "squirrel_planning_execution/PlanToSensePDDLGenerator.h"
#include "squirrel_planning_execution/PlanToAskPDDLGenerator.h"


//#define RECOMMENDER_SYSTEM_DEBUG
namespace KCL_rosplan {

	/**
	 * Types.
	 */
	std::map<std::string, const Type*> Type::generated_types_;
	
	const Type& Type::createType(const std::string& name, const Type* parent)
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
		ROS_INFO("KCL: (RecommenderSystem) Created the type %s (super type=%s)\n", name.c_str(), (parent == NULL ? "NULL" : parent->getName().c_str()));
		return *t;
	}
	
	const Type& Type::getType(const std::string& name)
	{
		const Type* t = NULL;
		std::map<std::string, const Type*>::const_iterator mi = generated_types_.find(name);
		if (mi == generated_types_.end())
		{
			ROS_ERROR("KCL: (RecommanderSystem) Could not find the type with the name %s.\n", name.c_str());
			exit(1);
		}
		return *(*mi).second;
	}
	
	void Type::getAllTypes(std::vector<const Type*>& all_types)
	{
		for (std::map<std::string, const Type*>::const_iterator ci = generated_types_.begin(); ci != generated_types_.end(); ++ci)
		{
			all_types.push_back(ci->second);
		}
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
	
	const Object& Object::createObject(const std::string& name, const Type& type)
	{
		const Object* o = getObject(name);
		if (o == NULL)
		{
			o = new Object(name, type);
			generated_objects_[name] = o;
		}
		
		// Check if we need to update the type of object.
		if (type.isChildOf(o->getType()))
		{
			Object* no = const_cast<Object*>(o);
			no->type_ = &type;
		}
		// Saniny check.
		else if (!o->type_->isChildOf(type) && o->type_ != &type)
		{
			ROS_ERROR("Requested the object %s with type %s, but an existing object with the same name exists that has the type %s. Stopping!", name.c_str(), type.getName().c_str(), o->type_->getName().c_str());
			exit(-1);
		}
		ROS_INFO("KCL: (RecommenderSystem) Created the object %s (type=%s)\n", name.c_str(), type.getName().c_str());
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
	
	void Fact::getAllFacts(std::vector<const Fact*>& all_facts)
	{
		for (std::map<std::pair<const Predicate*, std::vector<const Object*> >, const Fact*>::iterator i = generated_facts_.begin(); i != generated_facts_.end(); ++i)
		{
			all_facts.push_back(i->second);
		}
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
	
	std::ostream& operator<<(std::ostream& os, const UtilityFact& utility_fact)
	{
		os << "[" << utility_fact.getUtility() << "] " << utility_fact.getFact();
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
		
		get_domain_type_client = nh.serviceClient<rosplan_knowledge_msgs::GetDomainTypeService>("/kcl_rosplan/get_domain_types");
		get_domain_attribute_client = nh.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>("/kcl_rosplan/get_domain_predicates");
		
		// Initialise the domain.
		//initialise();
		
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
#ifdef RECOMMENDER_SYSTEM_DEBUG
		ROS_INFO("KCL: (RecommenderSystem) Running! Input file: %s; Output file: %s.\n",  absolute_input_file.c_str(), absolute_output_file.c_str());
#endif
	}
	
	RecommenderSystem::~RecommenderSystem()
	{
		Fact::cleanup();
		Predicate::cleanup();
		Object::cleanup();
	}
	
	void RecommenderSystem::initialise()
	{
		ROS_INFO("KCL: (RecommenderSystem) Initialise.\n");
		// Get all the types.
		rosplan_knowledge_msgs::GetDomainTypeService typeSrv;
		if (!get_domain_type_client.call(typeSrv))
		{
			ROS_ERROR("KCL: (RecommanderSystem) Failed to call the get_domain_type_client.\n");
			exit(1);
		}
		std::vector<std::string> types = typeSrv.response.types;
		std::vector<std::string> super_types = typeSrv.response.super_types;
		
		ROS_INFO("KCL: (RecommenderSystem) Found %zd types and %zd super types..\n", types.size(), super_types.size());
		
		// Create all the types.
		std::set<std::string> processed_types;
		while (processed_types.size() < types.size())
		{
			for (unsigned int i = 0; i < types.size(); ++i)
			{
				if ((super_types[i] == "" || processed_types.count(super_types[i]) == 1) && processed_types.count(types[i]) == 0)
				{
					Type::createType(types[i], &Type::getType(super_types[i]));
					processed_types.insert(types[i]);
				}
			}
		}
		
		std::vector<const Type*> all_types;
		Type::getAllTypes(all_types);
		for(std::vector<const Type*>::const_iterator ci = all_types.begin(); ci != all_types.end(); ++ci)
		{
			const Type* type = *ci;

			// Fetch all instances of this type.
			rosplan_knowledge_msgs::GetInstanceService instanceSrv;
			instanceSrv.request.type_name = type->getName();
			if (!get_instance_client.call(instanceSrv))
			{
				ROS_ERROR("KCL: (RecommanderSystem) Could not call the instance server.\n");
				exit(-1);
			}
			
			for(std::vector<std::string>::iterator iit = instanceSrv.response.instances.begin(); iit!=instanceSrv.response.instances.end(); iit++)
			{
				Object::createObject(*iit, *type);
			}
		}
		
		// Get all the predicates.
		rosplan_knowledge_msgs::GetDomainAttributeService predSrv;
		if (!get_domain_attribute_client.call(predSrv))
		{
			ROS_ERROR("Could not call the get_domain_attribute_client.\n");
			exit(1);
		}
		
		for (std::vector<rosplan_knowledge_msgs::DomainFormula>::const_iterator ci = predSrv.response.items.begin(); ci != predSrv.response.items.end(); ++ci)
		{
			const std::string& name = ci->name;
			const std::vector<diagnostic_msgs::KeyValue>& parameters = ci->typed_parameters;
			
			std::vector<const Type*> types;
			for (std::vector<diagnostic_msgs::KeyValue>::const_iterator p_ci = parameters.begin(); p_ci != parameters.end(); ++p_ci)
			{
				types.push_back(&Type::getType(p_ci->value));
			}
			
			// Create the predicate and ground it to get all the facts.
			const Predicate& predicate = Predicate::getPredicate(name, types);
			std::vector<const Fact*> grounded_facts;
			predicate.ground(grounded_facts);
		}
	}
	
	void RecommenderSystem::writeCSV(const std::vector<const Object*>& objects, const std::vector<const Predicate*>& predicates, const std::map<const Fact*, float>& weighted_facts)
	{
#ifdef RECOMMENDER_SYSTEM_DEBUG
		ROS_INFO("KCL: (RecommenderSystem) Write CSV.\n");
#endif
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
#ifdef RECOMMENDER_SYSTEM_DEBUG
		ROS_INFO("KCL: (RecommenderSystem) Finished writing CVS.\n");
#endif
	}
		
	void RecommenderSystem::readCSV(std::map<const Fact*, float>& results, const std::vector<const Predicate*>& predicates)
	{
#ifdef RECOMMENDER_SYSTEM_DEBUG
		ROS_INFO("KCL: (RecommenderSystem) Read CSV.\n");
#endif
		// Read the header.
		std::ifstream file(absolute_output_file.c_str());
		std::string line;
		unsigned int line_nr = 0;
		unsigned int max_arity = 0;
		
		// Read the header.
		//std::vector<const Predicate*> predicates;
		if (std::getline(file, line))
		{
#ifdef RECOMMENDER_SYSTEM_DEBUG
			ROS_INFO("Process the line: %s\n", line.c_str());
#endif
			std::vector<std::string> tokens = split(line, ',');
			
			// Skip the first few tokens as they refer to the object names.
			for (unsigned int i = 0; i < tokens.size(); ++i)
			{
				if (tokens[i].size() == 0)
				{
					continue;
				}
				
				const std::string& name = tokens[i].substr(1, tokens[i].size() - 2);
#ifdef RECOMMENDER_SYSTEM_DEBUG
				ROS_INFO("Process the token: %s\n", name.c_str());
#endif
				std::string name_trimmed = trimCopy(name);
				if (name_trimmed.substr(0, 6) == "object" && name_trimmed.size() > 6 && 
				   (::atoi(name_trimmed.substr(6).c_str()) != 0 || name_trimmed == "object0"))
				{
					++max_arity;
#ifdef RECOMMENDER_SYSTEM_DEBUG
					ROS_INFO("Object found %s.\n", name_trimmed.c_str());
#endif
					continue;
				}
				
				//const Predicate* p = Predicate::getPredicate(name_trimmed);
				//if (p == NULL)
				//{
				//	ROS_ERROR("Could not find the predicate: %s. Stopping!", name_trimmed.c_str());
				//	exit(-1);
				//}
			}
		}
		
		// Read the properties and store the confidence values.
		while (std::getline(file, line))
		{
#ifdef RECOMMENDER_SYSTEM_DEBUG
			ROS_INFO("Process the line: %s\n", line.c_str());
#endif
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
#ifdef RECOMMENDER_SYSTEM_DEBUG
				ROS_INFO("Added the object: %s.", name_trimmed.c_str());
#endif
				objects.push_back(o);
			}
#ifdef RECOMMENDER_SYSTEM_DEBUG
			std::cout << "Create the facts: " << predicates.size() << ", " << max_arity << ", " << tokens.size() << std::endl;
#endif
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
#ifdef RECOMMENDER_SYSTEM_DEBUG
				std::cout << fact << "; p=" << confidence << std::endl;
#endif
			}
		}
	}
	
	float RecommenderSystem::calculateKnowledge(const std::map<const Fact*, float>& results, const std::vector<const Fact*>& interesting_facts) const
	{
#ifdef RECOMMENDER_SYSTEM_DEBUG
		ROS_INFO("KCL: (RecommenderSystem) Run recommender.");
#endif
		// Check the current values of the interesting facts. This forms the baseline for determining which observation
		// actions are best to improve these.
		float value = 0;
		
		std::vector<const Fact*> facts_to_sense;
		for (std::map<const Fact*, float>::const_iterator ci = results.begin(); ci != results.end(); ++ci)
		{
			const Fact* fact = (*ci).first;
			float p = (*ci).second;
			
			for (unsigned int i = 0; i < interesting_facts.size(); ++i)
			{
				if (interesting_facts[i] == fact)
				{
					value += p;
#ifdef RECOMMENDER_SYSTEM_DEBUG
					std::cout << *fact << " -- Increase knowledge gain by " << p << " total value: " << value << std::endl;
#endif
				}
			}
		}
		return value;
	}
	
	std::map<const Fact*, float> RecommenderSystem::runRecommender(const std::vector<const Object*>& objects, const std::vector<const Predicate*>& predicates, const std::map<const Fact*, float>& weighted_facts)
	{
#ifdef RECOMMENDER_SYSTEM_DEBUG
		ROS_INFO("KCL: (RecommenderSystem) Run recommender.");
#endif
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
		
#ifdef RECOMMENDER_SYSTEM_DEBUG
		for (std::map<const Fact*, float>::const_iterator ci = results.begin(); ci != results.end(); ++ci)
		{
			const Fact* fact = ci->first;
			float p = ci->second;
			std::cout << "KCL: (RecommenderSystem) " << *fact << " = " << p << std::endl;
		}
#endif
		return results;
	}
	
	std::pair<float, float> RecommenderSystem::calculateKnowledgeIncrease(const std::vector<const Object*>& objects, const std::vector<const Predicate*>& predicates, std::map<const Fact*, float> weighted_facts, const std::vector<const Fact*>& interesting_facts, const Fact& sensing_action)
	{
		// 0 = unknown
		// 1 = false
		// 2 = true
		
		float knowledge_value_baseline = calculateKnowledge(weighted_facts, interesting_facts);
		
#ifdef RECOMMENDER_SYSTEM_DEBUG
		ROS_INFO("KCL: (RecommenderSystem) Simulate fact to be true.");
#endif
		weighted_facts[&sensing_action] = 2;
		std::map<const Fact*, float> results = runRecommender(objects, predicates, weighted_facts);
		
		float knowledge_value_if_true = calculateKnowledge(results, interesting_facts) - knowledge_value_baseline;
#ifdef RECOMMENDER_SYSTEM_DEBUG
		ROS_INFO("KCL: (RecommenderSystem) Simulate fact to be false.");
#endif
		weighted_facts[&sensing_action] = 1;
		results = runRecommender(objects, predicates, weighted_facts);
		
		float knowledge_value_if_false = calculateKnowledge(results, interesting_facts) - knowledge_value_baseline;
		
		return std::make_pair(knowledge_value_if_true, knowledge_value_if_false);
	}
	
	std::vector<const Fact*> RecommenderSystem::getBestSensingActions(const std::vector<const Object*>& objects, const std::vector<const Predicate*>& predicates, const std::map<const Fact*, float>& weighted_facts, const std::vector<const Fact*>& interesting_facts, unsigned int max_depth)
	{
		ROS_INFO("KCL: (RecommenderSystem) Get best sensing actions.");
		std::vector<const Fact*> best_facts_to_sense;
		
		std::map<const Fact*, float> results = runRecommender(objects, predicates, weighted_facts);
		
		float knowledge_value = calculateKnowledge(results, interesting_facts);
		
		// Given the baseline, we now check all facts we can observe and pick the one that increases it the most.
		float max_gain = 0;
		std::priority_queue<const UtilityFact*> queue;
		for (std::map<const Fact*, float>::const_iterator ci = weighted_facts.begin(); ci != weighted_facts.end(); ++ci)
		{
			if (ci->second != 0)
			{
				continue;
			}
			
			std::pair<float, float> knowledge_gain = calculateKnowledgeIncrease(objects, predicates, weighted_facts, interesting_facts, *ci->first);
			float d = (knowledge_gain.first + knowledge_gain.second) / 2.0f;
			if (d > max_gain)
			{
				best_facts_to_sense.clear();
				best_facts_to_sense.push_back(ci->first);
				max_gain = d;
				std::cout << "Sensing " << *ci->first << " gives us <" << knowledge_gain.first << ", " << knowledge_gain.second << ")" << std::endl;
			}
			queue.push(new UtilityFact(*ci->first, d));
		}
		
		ROS_INFO("KCL: (RecommenderSystem) Baseline results:");
		while (!queue.empty())
		{
			const UtilityFact* uf = queue.top();
			queue.pop();
			std::cout <<  *uf << std::endl;
		}
		
		return best_facts_to_sense;
	}
	/*
	void RecommenderSystem::updateKnowledgeBase(const Fact& last_observed_fact, const Fact& new_observed_fact, bool true_branch)
	{
		rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		
		// Set kenny at it's starting waypoint.
		rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_item.attribute_name = "dependency";
		knowledge_item.is_negative = false;
		
		diagnostic_msgs::KeyValue kv;
		kv.key = "o1";
		kv.value = last_observed_fact.getObjects()[0]->getName();
		knowledge_item.values.push_back(kv);
		
		kv.key = "b1";
		kv.value = last_observed_fact.getObjects()[1]->getName();
		knowledge_item.values.push_back(kv);
		
		kv.key = "o2";
		kv.value = new_observed_fact.getObjects()[0]->getName();
		knowledge_item.values.push_back(kv);
		
		kv.key = "b2";
		kv.value = new_observed_fact.getObjects()[1]->getName();
		knowledge_item.values.push_back(kv);
		
		kv.key = "is_true";
		kv.value = true_branch ? "TRUE" : "FALSE";
		knowledge_item.values.push_back(kv);
		
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the fact (dependency %s %s %s %s %s) to the knowledge base.", last_observed_fact.getObjects()[0]->getName().c_str(), last_observed_fact.getObjects()[1]->getName().c_str(), new_observed_fact.getObjects()[0]->getName().c_str(), new_observed_fact.getObjects()[1]->getName().c_str(), true_branch ? "TRUE" : "FALSE");
			exit(-1);
		}
		ROS_INFO("KCL: (RPSquirrelRecursion) Added the fact (dependency %s %s %s %s %s) to the knowledge base.", last_observed_fact.getObjects()[0]->getName().c_str(), last_observed_fact.getObjects()[1]->getName().c_str(), new_observed_fact.getObjects()[0]->getName().c_str(), new_observed_fact.getObjects()[1]->getName().c_str(), true_branch ? "TRUE" : "FALSE");
		knowledge_item.values.clear();
	}
	*/
	
	void RecommenderSystem::processMutualExclusive(const Fact& last_sensed_fact, std::map<const Fact*, float>& weighted_facts_copy, const std::vector<const Fact*>& interesting_facts)
	{
		std::cout << "callRecogniser:" << std::endl;
		for (std::vector<const Fact*>::const_iterator ci = interesting_facts.begin(); ci != interesting_facts.end(); ++ci)
		{
			std::cout << "\t-" << **ci << " -> " << weighted_facts_copy[*ci] << std::endl;
		}
		
		std::map<const Object*, std::vector<const Fact*>* > variable_domains;
		
		std::cout << "Build variable domains. Fact: " << last_sensed_fact << std::endl;
		for (std::vector<const Fact*>::const_iterator ci = interesting_facts.begin(); ci != interesting_facts.end(); ++ci)
		{
			std::cout << "Interesting fact: " << **ci << " = " << weighted_facts_copy[*ci] << std::endl;
			
			if ((*ci)->getPredicate().getName() == last_sensed_fact.getPredicate().getName() &&
			    weighted_facts_copy[*ci] == 0)
			{
				std::vector<const Fact*>* domain = NULL;
				
				if (variable_domains.find((*ci)->getObjects()[0]) == variable_domains.end())
				{
					domain = new std::vector<const Fact*>();
					variable_domains[(*ci)->getObjects()[0]] = domain;
				}
				else
				{
					domain = variable_domains[(*ci)->getObjects()[0]];
				}
				domain->push_back(*ci);
			}
		}
		
		// Check if the domain of a variable has only a single entry. If so make that fact true.
		std::cout << "Variable domains: " << std::endl;
		for (std::map<const Object*, std::vector<const Fact*>* >::const_iterator ci = variable_domains.begin(); ci != variable_domains.end(); ++ci)
		{
			const Object* object = ci->first;
			std::vector<const Fact*>* facts = ci->second;
			
			std::cout << "- " << object->getName() << " = {";
			for (std::vector<const Fact*>::const_iterator ci = facts->begin(); ci != facts->end(); ++ci)
			{
				std::cout << **ci << ", ";
			}
			std::cout << "}" << std::endl;
			
			if (facts->size() == 1)
			{
				weighted_facts_copy[(*facts)[0]] = 2.0f;
			}
		}
		
		// If a single entry is alraedy true, make the rest false.
		for (std::map<const Object*, std::vector<const Fact*>* >::const_iterator ci = variable_domains.begin(); ci != variable_domains.end(); ++ci)
		{
			const Object* object = ci->first;
			std::vector<const Fact*>* facts = ci->second;
			bool value_found = false;
			
			for (std::vector<const Fact*>::const_iterator ci = facts->begin(); ci != facts->end(); ++ci)
			{
				if (weighted_facts_copy[*ci] == 2.0f)
				{
					value_found = true;
					break;
				}
			}
			
			if (value_found)
			{
				for (std::vector<const Fact*>::const_iterator ci = facts->begin(); ci != facts->end(); ++ci)
				{
					if (weighted_facts_copy[*ci] != 2.0f)
					{
						weighted_facts_copy[*ci] = 1.0f;
					}
				}
			}
		}
		
		std::cout << "Final facts: " << std::endl;
		for (std::vector<const Fact*>::const_iterator ci = interesting_facts.begin(); ci != interesting_facts.end(); ++ci)
		{
			std::cout << "\t-" << **ci << " -> " << weighted_facts_copy[*ci] << std::endl;
		}
	}

	
	void RecommenderSystem::callRecogniser(FactObserveTree& node, const Fact& last_sensed_fact, unsigned int current_depth, unsigned int max_depth, const std::vector<const Object*>& objects, const std::vector<const Predicate*>& predicates, const std::map<const Fact*, float>& weighted_facts, const std::vector<const Fact*>& interesting_facts)
	{
		if (current_depth == max_depth) return;
		
		std::map<const Fact*, float> weighted_facts_copy(weighted_facts);
		processMutualExclusive(last_sensed_fact, weighted_facts_copy, interesting_facts);
		
		// Make the best sensing action true and run the system again.
		weighted_facts_copy[&last_sensed_fact] = 2.0;
		
		std::cout << "Get sensing actions for TRUE branch: " << std::endl;
		for (std::vector<const Fact*>::const_iterator ci = interesting_facts.begin(); ci != interesting_facts.end(); ++ci)
		{
			std::cout << "\t-" << **ci << " -> " << weighted_facts_copy[*ci] << std::endl;
		}
		
		std::vector<const KCL_rosplan::Fact*> facts_to_sense = getBestSensingActions(objects, predicates, weighted_facts_copy, interesting_facts, 3);
		
		ROS_INFO("KCL: (RecommanderSystem) [%u] TRUE BRANCH Find %zd possible facts to sense.\n", current_depth, facts_to_sense.size());
		//for (std::vector<const KCL_rosplan::Fact*>::const_iterator ci = facts_to_sense.begin(); ci != facts_to_sense.end(); ++ci)
		for (unsigned int i = 0; i < std::min((int)facts_to_sense.size(), 5); ++i)
		{
			std::cout << *facts_to_sense[i] << std::endl;
		}
		
		if (facts_to_sense.size() > 0)
		{
			FactObserveTree* true_branch = new FactObserveTree(*facts_to_sense[0]);
			node.true_branch_ = true_branch;
			//updateKnowledgeBase(last_sensed_fact, *facts_to_sense[0], true);
			callRecogniser(*true_branch, *facts_to_sense[0], current_depth + 1, max_depth, objects, predicates, weighted_facts_copy, interesting_facts);
		}
		
		// Make the best sensing action false and run the system again.
		weighted_facts_copy = weighted_facts;
		processMutualExclusive(last_sensed_fact, weighted_facts_copy, interesting_facts);
		weighted_facts_copy[&last_sensed_fact] = 1.0;
		
		std::cout << "Get sensing actions for FALSE branch: " << std::endl;
		for (std::vector<const Fact*>::const_iterator ci = interesting_facts.begin(); ci != interesting_facts.end(); ++ci)
		{
			std::cout << "\t-" << **ci << " -> " << weighted_facts_copy[*ci] << std::endl;
		}
		
		facts_to_sense = getBestSensingActions(objects, predicates, weighted_facts_copy, interesting_facts, 3);
		
		ROS_INFO("KCL: (RecommanderSystem) [%u] FALSE BRANCH Find %zd possible facts to sense.\n", current_depth, facts_to_sense.size());
		//for (std::vector<const KCL_rosplan::Fact*>::const_iterator ci = facts_to_sense.begin(); ci != facts_to_sense.end(); ++ci)
		for (unsigned int i = 0; i < std::min((int)facts_to_sense.size(), 5); ++i)
		{
			std::cout << *facts_to_sense[i] << std::endl;
		}
		if (facts_to_sense.size() > 0)
		{
			FactObserveTree* false_branch = new FactObserveTree(*facts_to_sense[0]);
			node.false_branch_ = false_branch;
			//updateKnowledgeBase(last_sensed_fact, *facts_to_sense[0], false);
			callRecogniser(*false_branch, *facts_to_sense[0], current_depth + 1, max_depth, objects, predicates, weighted_facts_copy, interesting_facts);
		}
	}
	
}; // close namespace

/**
 * Dummy PDDL problem generator.
 */
bool generatePDDLProblemFile(rosplan_knowledge_msgs::GenerateProblemService::Request &req, rosplan_knowledge_msgs::GenerateProblemService::Response &res) {
	return true;
}

int main(int argc, char** argv)
{
	ROS_INFO("KCL: (RecommenderSystem) Started!\n");
	
	
	/* Types */
	const KCL_rosplan::Type object = KCL_rosplan::Type::createType("object", NULL);
	const KCL_rosplan::Type catagory = KCL_rosplan::Type::createType("catagory", NULL);
	const KCL_rosplan::Type robot = KCL_rosplan::Type::createType("robot", NULL);
	const KCL_rosplan::Type box = KCL_rosplan::Type::createType("box", NULL);
	const KCL_rosplan::Type location = KCL_rosplan::Type::createType("location", NULL);
	
	ROS_INFO("KCL: (RecommenderSystem) Types created.\n");
	
	/* Objects */
	std::vector<const KCL_rosplan::Object*> objects;
	objects.push_back(&KCL_rosplan::Object::createObject("toy1", object));
	objects.push_back(&KCL_rosplan::Object::createObject("toy2", object));
	objects.push_back(&KCL_rosplan::Object::createObject("kenny", robot));
	objects.push_back(&KCL_rosplan::Object::createObject("car", catagory));
	objects.push_back(&KCL_rosplan::Object::createObject("dinosaur", catagory));
	objects.push_back(&KCL_rosplan::Object::createObject("box1", box));
	objects.push_back(&KCL_rosplan::Object::createObject("box2", box));
	objects.push_back(&KCL_rosplan::Object::createObject("wp1", location));
	objects.push_back(&KCL_rosplan::Object::createObject("wp2", location));
	objects.push_back(&KCL_rosplan::Object::createObject("box1_wp", location));
	objects.push_back(&KCL_rosplan::Object::createObject("box2_wp", location));
	objects.push_back(&KCL_rosplan::Object::createObject("toy1_wp", location));
	objects.push_back(&KCL_rosplan::Object::createObject("toy2_wp", location));
	objects.push_back(&KCL_rosplan::Object::createObject("end_follow_wp", location));
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
	ROS_INFO("KCL: (RecommenderSystem) Predicates created %zd.\n", predicates.size());
	
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
	
	/* Mark all facts that are valid with 0 (uncertain). */
	std::map<const KCL_rosplan::Fact*, float> weighted_facts;
	std::vector<const KCL_rosplan::Fact*> interesting_facts;
	for (std::vector<const KCL_rosplan::Fact*>::const_iterator ci = all_facts.begin(); ci != all_facts.end(); ++ci)
	{
		// All facts that are certain are set true.
		if ((*ci)->getPredicate().getName() != "belongs_in")
		{
			weighted_facts[*ci] = 2.0;
		}
		// Others are set to unsure.
		else
		{
			interesting_facts.push_back(*ci);
			weighted_facts[*ci] = 0.0;
		}
	}
	ROS_INFO("KCL: (RecommenderSystem) Weighted facts created.\n");
	
	ros::init(argc, argv, "rosplan_RecommanderSystem");
	ros::NodeHandle nh;
	
	// Start the listen to feedback action.
	KCL_rosplan::ListenToFeedbackPDDLAction listener(nh);
	
	std::string domain_path;
	nh.getParam("/rosplan/domain", domain_path);
	
	std::string data_path;
	nh.getParam("/rosplan/data_path", data_path);
	std::cout << "Data path: " << data_path << std::endl;
	
	ros::ServiceServer pddl_generation_service = nh.advertiseService("/kcl_rosplan/generate_planning_problem", &generatePDDLProblemFile);
	
	// Start the recommender system and initialise all the facts.
	KCL_rosplan::RecommenderSystem rs(nh);
	ROS_INFO("KCL: (RecommenderSystem) Recommender System started.\n");

	// Fetch all objects, predicates, etc. and start the recommender.
	std::vector<const KCL_rosplan::Fact*> facts_to_sense = rs.getBestSensingActions(objects, predicates, weighted_facts, interesting_facts, 3);
	
	ROS_INFO("KCL: (RecommenderSystem) Find %zd possible facts to sense.\n", facts_to_sense.size());
	
	const KCL_rosplan::Fact* previous_fact = facts_to_sense[0];
	for (unsigned int i = 0; i < std::min((int)facts_to_sense.size(), 5); ++i)
	{
		std::cout << *facts_to_sense[i] << std::endl;
	}
	
	KCL_rosplan::FactObserveTree root(*previous_fact);
	rs.callRecogniser(root, *previous_fact, 0, 3, objects, predicates, weighted_facts, interesting_facts);
	
	ROS_INFO("KCL: (RecommenderSystem) Recogniser is finished, create the planning problem.");
	
	// TEST.
	std::map<std::string, std::string> object_to_location_mapping;
	object_to_location_mapping["toy1"] = "toy1_wp";
	object_to_location_mapping["toy2"] = "toy2_wp";
	
	std::map<std::string, std::string> box_to_location_mapping;
	box_to_location_mapping["box1"] = "box1_wp";
	box_to_location_mapping["box2"] = "box2_wp";
	
	//KCL_rosplan::PlanToSensePDDLGenerator pts;
	KCL_rosplan::PlanToAskPDDLGenerator pts;
	pts.createPDDL(root, data_path, "domain.pddl", "problem.pddl", "kenny_wp", object_to_location_mapping, box_to_location_mapping);
	
	// Start the planner using the generated domain / problem files.
	
	std::string planner_path;
	nh.getParam("/planner_path", planner_path);
	
	std::stringstream ss;
	ss << data_path << "domain.pddl";
	std::string domain_path2 = ss.str();
	
	ss.str(std::string());
	ss << data_path << "problem.pddl";
	std::string problem_path = ss.str();
	std::cout << problem_path << std::endl;
	
	std::string planner_command;
	nh.getParam("/squirrel_planning_execution/planner_command", planner_command);
	
	rosplan_dispatch_msgs::PlanGoal psrv;
	psrv.domain_path = domain_path2;
	psrv.problem_path = problem_path;
	psrv.data_path = data_path;
	psrv.planner_command = planner_command;
	psrv.start_action_id = 0;

	ROS_INFO("KCL: (RobotKnowsGame) Start plan action");
	actionlib::SimpleActionClient<rosplan_dispatch_msgs::PlanAction> plan_action_client("/kcl_rosplan/start_planning", true);

	plan_action_client.waitForServer();
	ROS_INFO("KCL: (RobotKnowsGame) Start planning server found");
	
	// send goal
	plan_action_client.sendGoal(psrv);
	ROS_INFO("KCL: (RobotKnowsGame) Goal sent");

	while (ros::ok())
	{
		ros::spinOnce();
	}
	return 0;
}
