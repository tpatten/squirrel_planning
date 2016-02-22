#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <stdlib.h>
#include <map>
#include <set>
#include <string>

#include "squirrel_planning_execution/ContingentTacticalClassifyPDDLGenerator.h"

namespace KCL_rosplan {

void ContingentTacticalClassifyPDDLGenerator::generateProblemFile(const std::string& file_name, const KnowledgeBase& current_knowledge_base, const std::vector<const KnowledgeBase*>& knowledge_base, const Location& robot_location, const std::vector<Location*>& locations, const std::vector<Object*>& objects)
{
	std::vector<const State*> states;
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_base.begin(); ci != knowledge_base.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			const State* state = *ci;
			states.push_back(state);
		}
	}
	
	std::ofstream myfile;
	myfile.open(file_name.c_str());
	myfile << "(define (problem squirrel)" << std::endl;
	myfile << "(:domain classify_objects)" << std::endl;
	myfile << "(:objects" << std::endl;
	
	myfile << "\tl0 - LEVEL" << std::endl;
	myfile << "\tl1 - LEVEL" << std::endl;
	myfile << "\tl2 - LEVEL" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	myfile << "(:init" << std::endl;
	myfile << "\t(resolve-axioms)" << std::endl;
	myfile << "\t(lev l0)" << std::endl;
	myfile << "\t(next l0 l1)" << std::endl;
	myfile << "\t(next l1 l2)" << std::endl;
	
	myfile << "\t(current_kb " << current_knowledge_base.name_ << ")" << std::endl;
	
	for (std::vector<const State*>::const_iterator ci = current_knowledge_base.states_.begin(); ci != current_knowledge_base.states_.end(); ++ci)
	{
		const State* state = *ci;
		myfile << "\t(part-of " << state->state_name_ << " " << current_knowledge_base.name_ << ")" << std::endl;
		myfile << "\t(m " << state->state_name_ << ")" << std::endl;
		myfile << "\t(robot_at robot " << robot_location.name_ << " " << state->state_name_ << ")" << std::endl;
		//myfile << "\t(gripper_empty robot " << " " << state->state_name_ << ")" << std::endl;
		
		// All the objects are clear inially.
		for (std::vector<Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
		{
			const Object* object = *ci;
			//myfile << "\t(cleared " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t(object_at " << object->name_ << " " << object->location_->name_ << " " << state->state_name_ << ")" << std::endl;
			/*
			for (std::vector<const Location*>::const_iterator ci = object->observable_locations_.begin(); ci != object->observable_locations_.end() - 1; ++ci)
			{
				const Location* location = *ci;
				myfile << "\t(can_see_waypoint " << location->name_ << " " << clear_location->name_ << " " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			}
			*/
		}
	}
	
	for (std::vector<Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
	{
		const Location* location = *ci;
		for (std::vector<const Location*>::const_iterator ci = location->connected_locations_.begin(); ci != location->connected_locations_.end(); ++ci)
		{
			const Location* location2 = *ci;
			if (location == location2) continue;
			myfile << "\t(connected " << location->name_ << " " << location2->name_ << ")" << std::endl;
		}
		/*
		if (location->is_clear_)
		{
			myfile << "\t(clear_area " << location->name_ << ")" << std::endl;
		}
		*/
	}
	
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_base.begin(); ci != knowledge_base.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			const State* state = *ci;
			myfile << "\t(part-of " << state->state_name_ << " " << knowledge_base->name_ << ")" << std::endl;
			
			for (std::map<const Object*, const Location*>::const_iterator ci = state->classifiable_from_.begin(); ci != state->classifiable_from_.end(); ++ci)
			{
				const Object* object = (*ci).first;
				const Location* location = (*ci).second;
				
				if ("nowhere" == location->name_)
				{
					myfile << "\t(classifiable_from nowhere nowhere " << object->name_ << " " << state->state_name_ << ")" << std::endl;
				}
				else
				{
					myfile << "\t(classifiable_from " << location->name_ << " " << object->location_->name_ << " " << object->name_ << " " << state->state_name_ << ")" << std::endl;
				}
			}
		}
		
		for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_base->children_.begin(); ci != knowledge_base->children_.end(); ++ci)
		{
			myfile << "\t(parent " << knowledge_base->name_ << " " << (*ci)->name_ << ")" << std::endl;
		}
	}
	myfile << ")" << std::endl;
	myfile << "(:goal (and" << std::endl;
	for (std::vector<Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
	{
		const Object* object = *ci;
		for (std::vector<const State*>::const_iterator ci = current_knowledge_base.states_.begin(); ci != current_knowledge_base.states_.end(); ++ci)
		{
			myfile << "\t(classified " << object->name_ << " " << (*ci)->state_name_ << ")" << std::endl;
		}
	}
	myfile << ")" << std::endl;
	myfile << ")" << std::endl;
	myfile << ")" << std::endl;
	myfile.close();
}

void ContingentTacticalClassifyPDDLGenerator::generateDomainFile(const std::string& file_name, const KnowledgeBase& current_knowledge_base, const std::vector<const KnowledgeBase*>& knowledge_bases, const Location& robot_location, const std::vector<Location*>& locations, const std::vector<Object*>& objects)
{
	std::vector<const State*> states;
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			const State* state = *ci;
			states.push_back(state);
		}
	}
	
	std::cout << "generateClassifyObjectTacticalDomain:: file name= " << file_name << std::endl;
	
	std::ofstream myfile;
	myfile.open (file_name.c_str());
	myfile << "(define (domain classify_objects)" << std::endl;
	myfile << "(:requirements :typing :conditional-effects :negative-preconditions :disjunctive-preconditions)" << std::endl;
	myfile << std::endl;
	myfile << "(:types" << std::endl;
	myfile << "\twaypoint robot object" << std::endl;
	myfile << "\tlevel" << std::endl;
	myfile << "\tstate" << std::endl;
	myfile << "\tknowledgebase" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	myfile << "(:predicates" << std::endl;
	myfile << "\t(robot_at ?r - robot ?wp - waypoint ?s - state)" << std::endl;
	myfile << "\t(Rrobot_at ?r - robot ?wp - waypoint ?s - state)" << std::endl;
	myfile << "\t(object_at ?o - object ?wp - waypoint ?s - state)" << std::endl;
	myfile << "\t(Robject_at ?o - object ?wp - waypoint ?s - state)" << std::endl;
	myfile << "\t(classified ?o - object ?s - state)" << std::endl;
	myfile << "\t(Rclassified ?o - object ?s - state)" << std::endl;
	myfile << "\t(classification_failed ?o - object ?s - state)" << std::endl;
	myfile << "\t(Rclassification_failed ?o - object ?s - state)" << std::endl;
	myfile << "\t(classifiable_from ?from - waypoint ?view - waypoint ?o - object ?s - state)" << std::endl;
	myfile << "\t(Rclassifiable_from ?from - waypoint ?view - waypoint ?o - object ?s - state)" << std::endl;
	
	myfile << "\t(connected ?from ?to - waypoint)" << std::endl;
	//myfile << "\t(clear_area ?w - waypoint)" << std::endl;
	
	
	myfile << "\t(part-of ?s - state ?kb - knowledgebase)" << std::endl;
	myfile << "\t(current_kb ?kb - knowledgebase)" << std::endl;
	myfile << "\t(parent ?kb ?kb2 - knowledgebase)" << std::endl;
	
	myfile << std::endl;
	myfile << "\t;; Bookkeeping predicates." << std::endl;
	myfile << "\t(next ?l ?l2 - level)" << std::endl;
	myfile << "\t(lev ?l - LEVEL)" << std::endl;
	myfile << "\t(m ?s - STATE)" << std::endl;
	myfile << "\t(stack ?s - STATE ?l - LEVEL)" << std::endl;
	
	myfile << "\t(resolve-axioms)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	/*
	myfile << "(:functions" << std::endl;
	myfile << "\t(remaining_examination_attempts ?o - object ?s - state)" << std::endl;
	myfile << ")" << std::endl;
	*/
	myfile << "(:constants" << std::endl;
	myfile << "\t; All the waypoints." << std::endl;
	
	for (std::vector<Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
	{
		myfile << "\t" << (*ci)->name_ << " - waypoint" << std::endl;
	}

	myfile << "\t; The objects." << std::endl;
	for (std::vector<Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
	{
		myfile << "\t" << (*ci)->name_ << " - object" << std::endl;
	}
	
	myfile << "\t; The robot." << std::endl;
	myfile << "\trobot - robot" << std::endl;
	
	myfile << "\t; All the states." << std::endl;
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			myfile << "\t" << (*ci)->state_name_ << " - state" << std::endl;
		}
	}

	myfile << "\t; The knowledge bases" << std::endl;
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		myfile << "\t" << knowledge_base->name_ << " - knowledgebase" << std::endl;
	}

	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**
	 * GOTO WAYPOINT.
	 */
	myfile << "(:action goto_waypoint" << std::endl;
	myfile << "\t:parameters (?r - robot ?from ?to - waypoint)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(connected ?from ?to)" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			myfile << "\t\t(Rrobot_at ?r ?from " << (*ci)->state_name_ << ")" << std::endl;
		}
	}
	
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(when (m " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t(and" << std::endl;

		myfile << "\t\t\t\t(not (robot_at ?r ?from " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (Rrobot_at ?r ?from " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(robot_at ?r ?to " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(Rrobot_at ?r ?to " << (*ci)->state_name_ << ")" << std::endl;

		myfile << "\t\t\t)" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**********************
	 * SENSE actions.     *
	 *********************/
	
	/**
	 * Classify sension action.
	 */
	myfile << ";; Attempt to classify the object." << std::endl;
	myfile << "(:action classify_object" << std::endl;
	
	myfile << "\t:parameters (?o - object ?r - robot ?from ?view - waypoint ?l ?l2 - level ?kb - knowledgebase)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	myfile << "\t\t(next ?l ?l2)" << std::endl;
	myfile << "\t\t(lev ?l)" << std::endl;
	
	myfile << "\t\t(current_kb ?kb)" << std::endl;
	myfile << "\t\t(not (current_kb basis_kb))" << std::endl;
	myfile << std::endl;
	
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			myfile << "\t\t(Rrobot_at ?r ?from " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Robject_at ?o ?view " << (*ci)->state_name_ << ")" << std::endl;
		}
	}

	myfile << "\t\t;; This action is only applicable if there are world states where the outcome can be different." << std::endl;
	
	myfile << "\t\t(exists (?s - state) (and (m ?s) (classifiable_from ?from ?view ?o ?s) (part-of ?s ?kb)))" << std::endl;
	myfile << "\t\t(exists (?s - state) (and (m ?s) (not (classifiable_from ?from ?view ?o ?s)) (part-of ?s ?kb)))" << std::endl;;
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t(not (lev ?l))" << std::endl;
	myfile << "\t\t(lev ?l2)" << std::endl;
	myfile << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(when (and (m " << (*ci)->state_name_ << ") (not (classifiable_from ?from ?view ?o " << (*ci)->state_name_ << ")))" << std::endl;
		myfile << "\t\t\t(and (stack " << (*ci)->state_name_ << " ?l) (not (m " << (*ci)->state_name_ << ")))" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	myfile << "\t\t(resolve-axioms)" << std::endl;
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**
	 * Action that gets called when an object cannot be classified at all.
	 */
	myfile << "(:action finalise_classification_nowhere" << std::endl;
	myfile << "\t:parameters (?ob - object)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			const State* state = *ci;
			myfile << "\t\t(or " << std::endl;
			myfile << "\t\t\t(not (m " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t(classifiable_from nowhere nowhere ?ob " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t)" << std::endl;
		}
	}
	
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		const State* state = *ci;
		myfile << "\t\t(when (m " << state->state_name_ << ")" << std::endl;
		myfile << "\t\t\t(and" << std::endl;
		myfile << "\t\t\t\t(classified ?ob " << state->state_name_ << ")" << std::endl;
		myfile << "\t\t\t)" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**
	 * Action that gets called to confirm the classification.
	 */
	myfile << "(:action finalise_classification" << std::endl;
	myfile << "\t:parameters (?ob - object ?from ?view - waypoint)" << std::endl;// ?l ?l2 - level ?kb - knowledgebase)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			const State* state = *ci;
			myfile << "\t\t(or " << std::endl;
			myfile << "\t\t\t(not (m " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t(classifiable_from ?from ?view ?ob " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t)" << std::endl;
		}
	}
	
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	myfile << "\t\t(resolve-axioms)" << std::endl;
	
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		const State* state = *ci;
		myfile << "\t\t(when (m " << state->state_name_ << ")" << std::endl;
		myfile << "\t\t\t(classified ?ob " << state->state_name_ << ")" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	
	/**
	 * POP action.
	 */
	myfile << ";; Exit the current branch." << std::endl;
	myfile << "(:action pop" << std::endl;
	myfile << "\t:parameters (?l ?l2 - level)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(lev ?l)" << std::endl;
	myfile << "\t\t(next ?l2 ?l)" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and " << std::endl;
	myfile << "\t\t(not (lev ?l))" << std::endl;
	myfile << "\t\t(lev ?l2)" << std::endl;
	myfile << "\t\t(resolve-axioms)" << std::endl;

	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(when (m " << (*ci)->state_name_ << ") " << std::endl;
		myfile << "\t\t\t(not (m " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t)" << std::endl;
		myfile << "\t\t(when (stack " << (*ci)->state_name_ << " ?l2)" << std::endl;
		myfile << "\t\t\t(and " << std::endl;
		myfile << "\t\t\t\t(m " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(not (stack " << (*ci)->state_name_ << " ?l2))" << std::endl;
		myfile << "\t\t\t)" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;

	myfile << ";; Resolve the axioms manually." << std::endl;
	myfile << "(:action ramificate" << std::endl;
	myfile << "\t:parameters ()" << std::endl;
	myfile << "\t:precondition (resolve-axioms)" << std::endl;
	myfile << "\t:effect (and " << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		const State* state = *ci;
		/*
		myfile << "\t\t(when (or (gripper_empty robot " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
		myfile << "\t\t\t(Rgripper_empty robot " << state->state_name_ << ")" << std::endl;
		myfile << "\t\t)" << std::endl;
		
		myfile << "\t\t(when (and (not (gripper_empty robot " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
		myfile << "\t\t\t(not (Rgripper_empty robot " << state->state_name_ << "))" << std::endl;
		myfile << "\t\t)" << std::endl;
		*/
		for (std::vector<Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
		{
			const Location* location = *ci;
			
			myfile << "\t\t(when (or (robot_at robot " << location->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
			myfile << "\t\t\t(Rrobot_at robot " << location->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			myfile << "\t\t(when (and (not (robot_at robot " << location->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t(not (Rrobot_at robot " << location->name_ << " " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t)" << std::endl;
		}
		
		for (std::vector<Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
		{
			const Object* object = *ci;
			/*
			myfile << "\t\t(when (or (holding robot " << object->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
			myfile << "\t\t\t(Rholding robot " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			myfile << "\t\t(when (and (not (holding robot " << object->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t(not (Rholding robot " << object->name_ << " " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			myfile << "\t\t(when (or (cleared " << object->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
			myfile << "\t\t\t(Rcleared " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			myfile << "\t\t(when (and (not (cleared " << object->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t(not (Rcleared " << object->name_ << " " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t)" << std::endl;
			*/
			myfile << "\t\t(when (or (classified " << object->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
			myfile << "\t\t\t(Rclassified " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			myfile << "\t\t(when (and (not (classified " << object->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t(not (Rclassified " << object->name_ << " " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			for (std::vector<Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
			{
				const Location* location = *ci;
				
				myfile << "\t\t(when (or (object_at " << object->name_ << " " << location->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
				myfile << "\t\t\t(Robject_at " << object->name_ << " " << location->name_ << " " << state->state_name_ << ")" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				myfile << "\t\t(when (and (not (object_at " << object->name_ << " " << location->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t(not (Robject_at " << object->name_ << " " << location->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t)" << std::endl;
				for (std::vector<Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
				{
					const Location* other_location = *ci;
					/*
					myfile << "\t\t(when (or (can_see_waypoint " << location->name_ << " " << other_location->name_ << " " << object->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
					myfile << "\t\t\t(Rcan_see_waypoint " << location->name_ << " " << other_location->name_ << " " << object->name_ << " " << state->state_name_ << ")" << std::endl;
					myfile << "\t\t)" << std::endl;
					
					myfile << "\t\t(when (and (not (can_see_waypoint " << location->name_ << " " << other_location->name_ << " " << object->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
					myfile << "\t\t\t(not (Rcan_see_waypoint " << location->name_ << " " << other_location->name_ << " " << object->name_ << " " << state->state_name_ << "))" << std::endl;
					myfile << "\t\t)" << std::endl;
					*/
					myfile << "\t\t(when (or (classifiable_from " << location->name_ << " " << other_location->name_ << " " << object->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
					myfile << "\t\t\t(Rclassifiable_from " << location->name_ << " " << other_location->name_ << " " << object->name_ << " " << state->state_name_ << ")" << std::endl;
					myfile << "\t\t)" << std::endl;
					
					myfile << "\t\t(when (and (not (classifiable_from " << location->name_ << " " << other_location->name_ << " " << object->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
					myfile << "\t\t\t(not (Rclassifiable_from " << location->name_ << " " << other_location->name_ << " " << object->name_ << " " << state->state_name_ << "))" << std::endl;
					myfile << "\t\t)" << std::endl;
				}
			}
		}
	}
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	
	myfile << ";; Move 'down' into the knowledge base." << std::endl;
	myfile << "(:action assume_knowledge" << std::endl;
	myfile << "\t:parameters (?old_kb ?new_kb - knowledgebase)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	myfile << "\t\t(current_kb ?old_kb)" << std::endl;
	myfile << "\t\t(parent ?old_kb ?new_kb)" << std::endl;
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t(not (current_kb ?old_kb))" << std::endl;
	myfile << "\t\t(current_kb ?new_kb)" << std::endl;
	myfile << "\t\t(resolve-axioms)" << std::endl;
	myfile << std::endl;
	
	myfile << "\t\t;; Now we need to delete all knowledge from the old_kb and insert it to" << std::endl;
	myfile << "\t\t;; the new_kb level." << std::endl;

	myfile << "\t\t;; For every state ?s, ?s2" << std::endl;
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		const State* state = *ci;
		
		// Get rid of all states that are not part of this knowledge base.
		myfile << "\t\t(when (and (m " << state->state_name_ << ") (not (part-of " << state->state_name_ << " ?new_kb)))" << std::endl;
		myfile << "\t\t\t(not (m " << state->state_name_ << "))" << std::endl;
		myfile << "\t\t)" << std::endl;
		
		// Enable the states that are encapsulated in this knowledge base.
		myfile << "\t\t(when (part-of " << state->state_name_ << " ?new_kb)" << std::endl;
		myfile << "\t\t\t(and (m " << state->state_name_ << "))" << std::endl;
		myfile << "\t\t)" << std::endl;
		
		// Copy all knowledge that is part of ?old_kb to all the new states.
		for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
		{
			const State* state2 = *ci;
			/*
			myfile << "\t\t(when (and (part-of " << state->state_name_ << " ?old_kb) (gripper_empty robot " << state->state_name_ << ") (part-of " << state2->state_name_ << " ?new_kb))" << std::endl;
			myfile << "\t\t\t(and " << std::endl;
			myfile << "\t\t\t\t(not (Rgripper_empty robot " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t\t(not (gripper_empty robot " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t\t(Rgripper_empty robot " << state2->state_name_ << ")" << std::endl;
			myfile << "\t\t\t\t(gripper_empty robot " << state2->state_name_ << ")" << std::endl;
			myfile << "\t\t\t)" << std::endl;
			myfile << "\t\t)" << std::endl;
			*/
			for (std::vector<Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
			{
				const Object* object = *ci;
				/*
				myfile << "\t\t(when (and (part-of " << state->state_name_ << " ?old_kb) (holding robot " << object->name_ << " " << state->state_name_ << ") (part-of " << state2->state_name_ << " ?new_kb))" << std::endl;
				myfile << "\t\t\t(and " << std::endl;
				myfile << "\t\t\t\t(not (Rholding robot " << object->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(not (holding robot " << object->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(Rholding robot " << object->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t\t(holding robot " << object->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t)" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				myfile << "\t\t(when (and (part-of " << state->state_name_ << " ?old_kb) (cleared " << object->name_ << " " << state->state_name_ << ") (part-of " << state2->state_name_ << " ?new_kb))" << std::endl;
				myfile << "\t\t\t(and " << std::endl;
				myfile << "\t\t\t\t(not (Rcleared " << object->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(not (cleared " << object->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(Rcleared " << object->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t\t(cleared " << object->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t)" << std::endl;
				myfile << "\t\t)" << std::endl;
				*/
				myfile << "\t\t(when (and (part-of " << state->state_name_ << " ?old_kb) (classified " << object->name_ << " " << state->state_name_ << ") (part-of " << state2->state_name_ << " ?new_kb))" << std::endl;
				myfile << "\t\t\t(and " << std::endl;
				myfile << "\t\t\t\t(not (Rclassified " << object->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(not (classified " << object->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(Rclassified " << object->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t\t(classified " << object->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t)" << std::endl;
				myfile << "\t\t)" << std::endl;
			}
			
			for (std::vector<Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
			{
				const Location* location = *ci;
				
				myfile << "\t\t(when (and (part-of " << state->state_name_ << " ?old_kb) (robot_at robot " << location->name_ << " " << state->state_name_ << ") (part-of " << state2->state_name_ << " ?new_kb))" << std::endl;
				myfile << "\t\t\t(and " << std::endl;
				myfile << "\t\t\t\t(not (Rrobot_at robot " << location->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(not (robot_at robot " << location->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(Rrobot_at robot " << location->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t\t(robot_at robot " << location->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t)" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				for (std::vector<Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
				{
					const Object* object = *ci;
					myfile << "\t\t(when (and (part-of " << state->state_name_ << " ?old_kb) (object_at " << object->name_ << " " << location->name_ << " " << state->state_name_ << ") (part-of " << state2->state_name_ << " ?new_kb))" << std::endl;
					myfile << "\t\t\t(and " << std::endl;
					myfile << "\t\t\t\t(not (Robject_at " << object->name_ << " " << location->name_ << " " << state->state_name_ << "))" << std::endl;
					myfile << "\t\t\t\t(not (object_at " << object->name_ << " " << location->name_ << " " << state->state_name_ << "))" << std::endl;
					myfile << "\t\t\t\t(Robject_at " << object->name_ << " " << location->name_ << " " << state2->state_name_ << ")" << std::endl;
					myfile << "\t\t\t\t(object_at " << object->name_ << " " << location->name_ << " " << state2->state_name_ << ")" << std::endl;
					myfile << "\t\t\t)" << std::endl;
					myfile << "\t\t)" << std::endl;
				}
			}
		}
	}
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;

	myfile << ";; Move 'up' into the knowledge base." << std::endl;;
	myfile << "(:action shed_knowledge" << std::endl;
	myfile << "\t:parameters (?old_kb ?new_kb - knowledgebase ?o - object)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	myfile << "\t\t(current_kb ?old_kb)" << std::endl;
	myfile << "\t\t(parent ?new_kb ?old_kb)" << std::endl;
	
	// We can only move back up the knowledge base if there are not states that belong to this knowledge base.
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(or " << std::endl;
		myfile << "\t\t\t(not (part-of " << (*ci)->state_name_ << " ?old_kb))" << std::endl;
		myfile << "\t\t\t(not (exists (?l - level ) (stack " << (*ci)->state_name_ << " ?l)))" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	
	// Make sure the robot is in the same location.
	myfile << "\t\t(or" << std::endl;
	for (std::vector<Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
	{
		const Location* location = *ci;
		myfile << "\t\t\t(and" << std::endl;
		for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
		{
			const State* state = *ci;
			// Make sure the state of the toilets are the same.
			myfile << "\t\t\t\t(or " << std::endl;
			myfile << "\t\t\t\t\t(not (part-of " << (*ci)->state_name_ << " ?old_kb))" << std::endl;
			myfile << "\t\t\t\t\t(robot_at robot " << location->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t\t\t)" << std::endl;
		}
		myfile << "\t\t\t)" << std::endl;
	}
	myfile << "\t\t)" << std::endl;
	
	// Make sure the object is classified.
	myfile << "\t\t(and" << std::endl;
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		const State* state = *ci;
		// Make sure the state of the toilets are the same.
		myfile << "\t\t\t(or " << std::endl;
		myfile << "\t\t\t\t(not (part-of " << state->state_name_ << " ?old_kb))" << std::endl;
		myfile << "\t\t\t\t(classified ?o " << state->state_name_ << ")" << std::endl;
		myfile << "\t\t\t)" << std::endl;
	}
	myfile << "\t\t)" << std::endl;
	/*
	// Make sure the location of the objects is identical.
	for (std::vector<const Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
	{
		const Object* object = *ci;
		myfile << "\t\t(or" << std::endl;
		for (std::vector<const Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
		{
			const Location* location = *ci;
			myfile << "\t\t\t(and" << std::endl;
			for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
			{
				const State* state = *ci;
				myfile << "\t\t\t\t(or " << std::endl;
				myfile << "\t\t\t\t\t(not (part-of " << (*ci)->state_name_ << " ?old_kb))" << std::endl;
				myfile << "\t\t\t\t\t(object_at " << object->name_ << " " <<  location->name_ << " " << state->state_name_ << ")" << std::endl;
				myfile << "\t\t\t\t)" << std::endl;
			}
			myfile << "\t\t\t)" << std::endl;
		}
		
		myfile << "\t\t\t(and" << std::endl;
		for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
		{
			const State* state = *ci;
			// Make sure the state of the toilets are the same.
			myfile << "\t\t\t\t(or " << std::endl;
			myfile << "\t\t\t\t\t(not (part-of " << (*ci)->state_name_ << " ?old_kb))" << std::endl;
			myfile << "\t\t\t\t\t(holding robot " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t\t\t)" << std::endl;
		}
		myfile << "\t\t\t)" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	*/
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t(not (current_kb ?old_kb))" << std::endl;
	myfile << "\t\t(current_kb ?new_kb)" << std::endl;
	myfile << "\t\t(resolve-axioms)" << std::endl;
	myfile << std::endl;
	
	myfile << "\t\t;; Now we need to push all knowledge that is true for all states part of " << std::endl;
	myfile << "\t\t;; kb_old up to kb_new." << std::endl;
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		const State* state = *ci;
		
		// Make the states that were held in suspection active again.
		myfile << "\t\t(when (part-of " << state->state_name_ << " ?new_kb)" << std::endl;
		myfile << "\t\t\t(and (m " << state->state_name_ << "))" << std::endl;
		myfile << "\t\t)" << std::endl;
		
		// And those that were active, inactive.
		myfile << "\t\t(when (not (part-of " << state->state_name_ << " ?new_kb))" << std::endl;
		myfile << "\t\t\t(and (not (m " << state->state_name_ << ")))" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	/*
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		const State* state = *ci;
		myfile << "\t\t(when (and " << std::endl;
		myfile << "\t\t\t\t;; For every state ?s, ?s2" << std::endl;
	
		for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
		{
			const State* state2 = *ci;
			myfile << "\t\t\t\t\t(or " << std::endl;
			myfile << "\t\t\t\t\t\t(not (part-of " << state2->state_name_ << " ?old_kb))" << std::endl;
			myfile << "\t\t\t\t\t\t(gripper_empty robot " << state2->state_name_ << ")" << std::endl;
			myfile << "\t\t\t\t\t)" << std::endl;
		}
		myfile << "\t\t\t\t\t(part-of " << state->state_name_ << " ?new_kb)" << std::endl;
		myfile << "\t\t\t)" << std::endl;

		myfile << "\t\t\t;; Conditional effects" << std::endl;
		myfile << "\t\t\t(and " << std::endl;
		for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
		{
			const State* state2 = *ci;
			myfile << "\t\t\t\t(not (gripper_empty robot " << state2->state_name_ << "))" << std::endl;
		}
		myfile << "\t\t\t\t(gripper_empty robot " << state->state_name_ << ")" << std::endl;
		myfile << "\t\t\t)" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	*/
	for (std::vector<Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
	{
		const Object* object = *ci;
		for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
		{
			// Deal with the location of the robot.
			const State* state = *ci;
			/*
			myfile << "\t\t(when (and " << std::endl;
			myfile << "\t\t\t\t;; For every state ?s, ?s2" << std::endl;
		
			// Holding.
			for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
			{
				const State* state2 = *ci;
				myfile << "\t\t\t\t\t(or " << std::endl;
				myfile << "\t\t\t\t\t\t(not (part-of " << state2->state_name_ << " ?old_kb))" << std::endl;
				myfile << "\t\t\t\t\t\t(holding robot " << object->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t\t\t)" << std::endl;
				
			}
			myfile << "\t\t\t\t\t(part-of " << state->state_name_ << " ?new_kb)" << std::endl;
			myfile << "\t\t\t)" << std::endl;

			myfile << "\t\t\t;; Conditional effects" << std::endl;
			myfile << "\t\t\t(and " << std::endl;
			for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
			{
				const State* state2 = *ci;
				myfile << "\t\t\t\t(not (holding robot " << object->name_ << " " << state2->state_name_ << "))" << std::endl;
			}
			myfile << "\t\t\t\t(holding robot " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t\t)" << std::endl;
			myfile << "\t\t)" << std::endl;
			*/
			// Classified.
			myfile << "\t\t(when (and " << std::endl;
			myfile << "\t\t\t\t;; For every state ?s, ?s2" << std::endl;
		
			for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
			{
				const State* state2 = *ci;
				myfile << "\t\t\t\t\t(or " << std::endl;
				myfile << "\t\t\t\t\t\t(not (part-of " << state2->state_name_ << " ?old_kb))" << std::endl;
				myfile << "\t\t\t\t\t\t(classified " << object->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t\t\t)" << std::endl;
				
			}
			myfile << "\t\t\t\t\t(part-of " << state->state_name_ << " ?new_kb)" << std::endl;
			myfile << "\t\t\t)" << std::endl;

			myfile << "\t\t\t;; Conditional effects" << std::endl;
			myfile << "\t\t\t(and " << std::endl;
			for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
			{
				const State* state2 = *ci;
				myfile << "\t\t\t\t(not (classified " << object->name_ << " " << state2->state_name_ << "))" << std::endl;
			}
			myfile << "\t\t\t\t(classified " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t\t)" << std::endl;
			myfile << "\t\t)" << std::endl;
			/*
			// Cleared.
			myfile << "\t\t(when (and " << std::endl;
			myfile << "\t\t\t\t;; For every state ?s, ?s2" << std::endl;
		
			for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
			{
				const State* state2 = *ci;
				myfile << "\t\t\t\t\t(or " << std::endl;
				myfile << "\t\t\t\t\t\t(not (part-of " << state2->state_name_ << " ?old_kb))" << std::endl;
				myfile << "\t\t\t\t\t\t(cleared " << object->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t\t\t)" << std::endl;
				
			}
			myfile << "\t\t\t\t\t(part-of " << state->state_name_ << " ?new_kb)" << std::endl;
			myfile << "\t\t\t)" << std::endl;

			myfile << "\t\t\t;; Conditional effects" << std::endl;
			myfile << "\t\t\t(and " << std::endl;
			for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
			{
				const State* state2 = *ci;
				myfile << "\t\t\t\t(not (cleared " << object->name_ << " " << state2->state_name_ << "))" << std::endl;
			}
			myfile << "\t\t\t\t(cleared " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t\t)" << std::endl;
			myfile << "\t\t)" << std::endl;
			*/
		}
	}
	
	// Location of the agent.
	for (std::vector<Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
	{
		const Location* location = *ci;
		for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
		{
			// Deal with the location of the robot.
			const State* state = *ci;
			myfile << "\t\t(when (and " << std::endl;
			myfile << "\t\t\t\t;; For every state ?s, ?s2" << std::endl;
		
			for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
			{
				const State* state2 = *ci;
				myfile << "\t\t\t\t(and " << std::endl;
				myfile << "\t\t\t\t\t(or " << std::endl;
				myfile << "\t\t\t\t\t\t(not (part-of " << state2->state_name_ << " ?old_kb))" << std::endl;
				myfile << "\t\t\t\t\t\t(robot_at robot " << location->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t\t\t)" << std::endl;
				myfile << "\t\t\t\t)" << std::endl;
			}
			myfile << "\t\t\t\t\t(part-of " << state->state_name_ << " ?new_kb)" << std::endl;
			myfile << "\t\t\t)" << std::endl;

			myfile << "\t\t\t;; Conditional effects" << std::endl;
			myfile << "\t\t\t(and " << std::endl;
			
			for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
			{
				const State* state2 = *ci;
				myfile << "\t\t\t\t(not (robot_at robot " << location->name_ << " " << state2->state_name_ << "))" << std::endl;
			}
			myfile << "\t\t\t\t(robot_at robot " << location->name_ << " " << state->state_name_ << ")" << std::endl;
			
			myfile << "\t\t\t)" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			// The locations of the objects.
			for (std::vector<Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
			{
				const Object* object = *ci;
				
				// Deal with the location of the robot.
				myfile << "\t\t(when (and " << std::endl;
				myfile << "\t\t\t\t;; For every state ?s, ?s2" << std::endl;
			
				for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
				{
					const State* state2 = *ci;
					myfile << "\t\t\t\t(and " << std::endl;
					myfile << "\t\t\t\t\t(or " << std::endl;
					myfile << "\t\t\t\t\t\t(not (part-of " << state2->state_name_ << " ?old_kb))" << std::endl;
					myfile << "\t\t\t\t\t\t(object_at " << object->name_ << " " << location->name_ << " " << state2->state_name_ << ")" << std::endl;
					myfile << "\t\t\t\t\t)" << std::endl;
					myfile << "\t\t\t\t)" << std::endl;
				}
				myfile << "\t\t\t\t\t(part-of " << state->state_name_ << " ?new_kb)" << std::endl;
				myfile << "\t\t\t)" << std::endl;

				myfile << "\t\t\t;; Conditional effects" << std::endl;
				myfile << "\t\t\t(and " << std::endl;
				
				for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
				{
					const State* state2 = *ci;
					myfile << "\t\t\t\t(not (object_at " << object->name_ << " " << location->name_ << " " << state2->state_name_ << "))" << std::endl;
				}
				myfile << "\t\t\t\t(object_at " << object->name_ << " " << location->name_ << " " << state->state_name_ << ")" << std::endl;
				
				myfile << "\t\t\t)" << std::endl;
				myfile << "\t\t)" << std::endl;
			}
		}
	}
	
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << ")" << std::endl;
	myfile.close();
}

void ContingentTacticalClassifyPDDLGenerator::createPDDL(const std::string& path, const std::string& domain_file, const std::string& problem_file, const std::string& robot_location_predicate, const std::vector<std::string>& location_predicates, const std::string& object_predicate, const std::string& object_location_predicate)
{
	// Now generate the waypoints / boxes / toys / etc.
	std::vector<Location*> locations;
	std::vector<Object*> objects;
	
	// Initialise the robot's location.
	Location* robot_location = new Location(robot_location_predicate, false);
	locations.push_back(robot_location);
	
	// Initialise the object's location.
	Location* location = new Location(object_location_predicate, false);
	locations.push_back(location);
	Object* object = new Object(object_predicate, *location);
	objects.push_back(object);
	
	// Add the observation waypoints from where we want to classify the object.
	for (std::vector<std::string>::const_iterator ci = location_predicates.begin(); ci != location_predicates.end(); ++ci)
	{
		const std::string& location_predicate = *ci;
		Location* location = new Location(location_predicate, true);
		locations.push_back(location);
		object->observable_locations_.push_back(location);
	}
	
	// Make all locations fully connected.
	for (std::vector<Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
	{
		Location* location = *ci;
		for (std::vector<Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
		{
			Location* other_location = *ci;
			location->connected_locations_.push_back(other_location);
			other_location->connected_locations_.push_back(location);
		}
	}
	
	std::cout << "Creating all possible states..." << std::endl;

	std::vector<const KnowledgeBase*> knowledge_bases;
	std::map<const Object*, const Location*> empty_stacked_objects_mapping;
	State basic_state("basic", empty_stacked_objects_mapping);
	
	KnowledgeBase basis_kb("basis_kb");
	basis_kb.addState(basic_state);
	knowledge_bases.push_back(&basis_kb);
	
	unsigned int state_id = 0;
	
	// Create a new knowledge base for each object.
	std::stringstream ss;
	for (std::vector<Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
	{
		const Object* object = *ci;
		ss.str(std::string());
		ss << "kb_" << object->name_;
		
		KnowledgeBase* kb_location = new KnowledgeBase(ss.str());
		basis_kb.addChild(*kb_location);
		knowledge_bases.push_back(kb_location);
		
		std::map<const Object*, const Location*> stacked_objects_mapping;
		
		// Create a new knowledge base for each object.
		for (std::vector<Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
		{
			const Object* object = *ci;
			ss.str(std::string());
			ss << "kb_" << object->name_;
			
			KnowledgeBase* kb_location = new KnowledgeBase(ss.str());
			basis_kb.addChild(*kb_location);
			knowledge_bases.push_back(kb_location);
			
			std::map<const Object*, const Location*> classifiable_from;
			for (std::vector<const Location*>::const_iterator ci = object->observable_locations_.begin(); ci != object->observable_locations_.end(); ++ci)
			{
				const Location* location = *ci;
				classifiable_from.clear();
				classifiable_from[object] = location;
				
				std::cout << object->name_ << " is visable from " << location->name_ << std::endl;
			
				ss.str(std::string());
				ss << "s" << state_id;
				State* state_pickupable = new State(ss.str(), classifiable_from);
				kb_location->addState(*state_pickupable);
				++state_id;
			}
		}
	}
	
	ss.str(std::string());
	ss << path << domain_file;
	std::cout << "Generate domain... " << ss.str() << std::endl;
	generateDomainFile(ss.str(), basis_kb, knowledge_bases, *robot_location, locations, objects);
	ss.str(std::string());
	ss << path  << problem_file;
	std::cout << "Generate problem... " << ss.str() << std::endl;
	generateProblemFile(ss.str(), basis_kb, knowledge_bases, *robot_location,locations, objects);
}

};
