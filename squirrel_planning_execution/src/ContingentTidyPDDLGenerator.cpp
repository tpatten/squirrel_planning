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

#include "squirrel_planning_execution/ContingentTidyPDDLGenerator.h"

namespace KCL_rosplan {

void ContingentTidyPDDLGenerator::generateProblemFile(const std::string& file_name, const KnowledgeBase& current_knowledge_base, const std::vector<const KnowledgeBase*>& knowledge_base, const Location& robot_location, const std::vector<const Location*>& locations, const std::vector<const Object*>& objects, const std::vector<const Box*>& boxes, const std::vector<const Type*>& types)
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
	myfile << "(define (problem Keys-0)" << std::endl;
	myfile << "(:domain find_key)" << std::endl;
	myfile << "(:objects" << std::endl;
	
	myfile << "\tl0 - LEVEL" << std::endl;
	myfile << "\tl1 - LEVEL" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	myfile << "(:init" << std::endl;
	myfile << "\t(resolve-axioms)" << std::endl;
	myfile << "\t(lev l0)" << std::endl;
	
	myfile << "\t(next l0 l1)" << std::endl;
	
	myfile << "\t(current_kb " << current_knowledge_base.name_ << ")" << std::endl;
	
	// Location of the robot.
	for (std::vector<const State*>::const_iterator ci = current_knowledge_base.states_.begin(); ci != current_knowledge_base.states_.end(); ++ci)
	{
		const State* state = *ci;
		myfile << "\t(part-of " << state->state_name_ << " " << current_knowledge_base.name_ << ")" << std::endl;
		myfile << "\t(m " << state->state_name_ << ")" << std::endl;
		myfile << "\t(robot_at robot " << robot_location.name_ << " " << state->state_name_ << ")" << std::endl;
		myfile << "\t(gripper_empty robot " << " " << state->state_name_ << ")" << std::endl;
		
		// All the objects are clear inially.
		for (std::vector<const Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
		{
			const Object* object = *ci;
			//myfile << "\t(clear " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t(object_at " << object->name_ << " " << object->location_->name_ << " " << state->state_name_ << ")" << std::endl;
		}
		
		for (std::vector<const Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
		{
			const Location* location = *ci;
			bool is_blocked = location->is_blocked_ || location == &robot_location;
			
			for (std::vector<const Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
			{
				const Object* object = *ci;
				if (object->location_ == location)
				{
					is_blocked = true;
					break;
				}
			}
			
			for (std::vector<const Box*>::const_iterator ci = boxes.begin(); ci != boxes.end(); ++ci)
			{
				const Box* box = *ci;
				if (box->location_ == location)
				{
					is_blocked = true;
					break;
				}
			}
			
			if (!is_blocked)
			{
				myfile << "\t(is_not_occupied " << location->name_ << " " << state->state_name_ << ")" << std::endl;
			}
		}
	}
	
	// Location of the boxes.
	for (std::vector<const Box*>::const_iterator ci = boxes.begin(); ci != boxes.end(); ++ci)
	{
		const Box* box = *ci;
		myfile << "\t(box_at " << box->name_ << " " << box->location_->name_ << ")" << std::endl;
		for (std::vector<const Type*>::const_iterator ci = box->types_that_fit_.begin(); ci != box->types_that_fit_.end(); ++ci)
		{
			const Type* type = *ci;
			for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_base.begin(); ci != knowledge_base.end(); ++ci)
			{
				const KnowledgeBase* knowledge_base = *ci;
				
				for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
				{
					const State* state = *ci;
					myfile << "\t(can_fit_inside " << type->name_ << " " << box->name_ << " " << state->state_name_ << ")" << std::endl;
				}
			}
		}
	}
	/*
	// Location constants.
	for (std::vector<const Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
	{
		const Location* location = *ci;
		std::cout << "Process the location: " << location->name_ << std::endl;
		//for (std::vector<const Location*>::const_iterator ci = location->connected_locations_.begin(); ci != location->connected_locations_.end(); ++ci)
		for (std::vector<const Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
		{
			const Location* location2 = *ci;
			//if (location == location2) continue;
			myfile << "\t(connected " << location->name_ << " " << location2->name_ << ")" << std::endl;
			
			//std::cout << "\t is connected to " << location2->name_ << std::endl;
			//myfile << "\t(connected " << location2->name_ << " " << location->name_ << ")" << std::endl;
		}
	}
	*/
	// Locations of the objects.
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_base.begin(); ci != knowledge_base.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			const State* state = *ci;
			myfile << "\t(part-of " << state->state_name_ << " " << knowledge_base->name_ << ")" << std::endl;
			
			/*
			for (std::map<const Object*, const Object*>::const_iterator ci = state->stackable_mapping_.begin(); ci != state->stackable_mapping_.end(); ++ci)
			{
				myfile << "\t(on " << (*ci).first->name_ << " " << (*ci).second->name_ << " " << state->state_name_ << ")" << std::endl;
			}
			*/
			for (std::map<const Object*, const Type*>::const_iterator ci = state->type_mapping_.begin(); ci != state->type_mapping_.end(); ++ci)
			{
				myfile << "\t(is_of_type " << (*ci).first->name_ << " " << (*ci).second->name_ << " " << state->state_name_ << ")" << std::endl;
			}
			
			for (std::vector<const Type*>::const_iterator ci = state->pushable_objects_.begin(); ci != state->pushable_objects_.end(); ++ci)
			{
				myfile << "\t(can_push robot " << (*ci)->name_ << " " << state->state_name_ << ")" << std::endl;
			}
			
			for (std::vector<const Type*>::const_iterator ci = state->pickupable_objects_.begin(); ci != state->pickupable_objects_.end(); ++ci)
			{
				myfile << "\t(can_pickup robot " << (*ci)->name_ << " " << state->state_name_ << ")" << std::endl;
			}
		}
		
		for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_base->children_.begin(); ci != knowledge_base->children_.end(); ++ci)
		{
			myfile << "\t(parent " << knowledge_base->name_ << " " << (*ci)->name_ << ")" << std::endl;
		}
	}
	myfile << ")" << std::endl;
	myfile << "(:goal (and" << std::endl;
	for (std::vector<const Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
	{
		const Object* object = *ci;
		for (std::vector<const State*>::const_iterator ci = current_knowledge_base.states_.begin(); ci != current_knowledge_base.states_.end(); ++ci)
		{
			myfile << "\t(tidy " << object->name_ << " " << (*ci)->state_name_ << ")" << std::endl;
		}
	}
	myfile << ")" << std::endl;
	myfile << ")" << std::endl;
	myfile << ")" << std::endl;
	myfile.close();
}

void ContingentTidyPDDLGenerator::generateDomainFile(const std::string& file_name, const KnowledgeBase& current_knowledge_base, const std::vector<const KnowledgeBase*>& knowledge_bases, const Location& robot_location, const std::vector<const Location*>& locations, const std::vector<const Object*>& objects, const std::vector<const Box*>& boxes, const std::vector<const Type*>& types)
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
	
	std::ofstream myfile;
	myfile.open (file_name.c_str());
	myfile << "(define (domain find_key)" << std::endl;
	myfile << "(:requirements :typing :conditional-effects :negative-preconditions :disjunctive-preconditions)" << std::endl;
	myfile << std::endl;
	myfile << "(:types" << std::endl;
	myfile << "\twaypoint robot object box type" << std::endl;
	myfile << "\tlevel" << std::endl;
	myfile << "\tstate" << std::endl;
	myfile << "\tknowledgebase" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	myfile << "(:predicates" << std::endl;
	myfile << "\t(robot_at ?v - robot ?wp - waypoint ?s - state)" << std::endl;
	myfile << "\t(Rrobot_at ?v - robot ?wp - waypoint ?s - state)" << std::endl;
	myfile << "\t(object_at ?o - object ?wp - waypoint ?s - state)" << std::endl;
	myfile << "\t(Robject_at ?o - object ?wp - waypoint ?s - state)" << std::endl;
	myfile << "\t(box_at ?b - box ?wp - waypoint)" << std::endl;
	myfile << "\t(Rbox_at ?b - box ?wp - waypoint)" << std::endl;
	myfile << "\t(gripper_empty ?v - robot ?s - state)" << std::endl;
	myfile << "\t(Rgripper_empty ?v - robot ?s - state)" << std::endl;
	myfile << "\t(holding ?v - robot ?o - object ?s - state)" << std::endl;
	myfile << "\t(Rholding ?v - robot ?o - object ?s - state)" << std::endl;
	myfile << "\t(is_not_occupied ?wp - waypoint ?s - state)" << std::endl;
	myfile << "\t(Ris_not_occupied ?wp - waypoint ?s - state)" << std::endl;
	myfile << "\t(tidy ?o - object ?s - state)" << std::endl;
	myfile << "\t(Rtidy ?o - object ?s - state)" << std::endl;
	myfile << "\t(tidy_location ?t - type ?wp - waypoint ?s - state)" << std::endl;
	myfile << "\t(Rtidy_location ?t - type ?wp - waypoint ?s - state)" << std::endl;
	//myfile << "\t(push_location ?o - object ?wp - waypoint ?s - state)" << std::endl;
	//myfile << "\t(Rpush_location ?o - object ?wp - waypoint ?s - state)" << std::endl;
	myfile << "\t(can_pickup ?v - robot ?t - type ?s - state)" << std::endl;
	myfile << "\t(Rcan_pickup ?v - robot ?t - type ?s - state)" << std::endl;
	myfile << "\t(can_push ?v - robot ?t - type ?s - state)" << std::endl;
	myfile << "\t(Rcan_push ?v - robot ?t - type ?s - state)" << std::endl;
	myfile << "\t(can_fit_inside ?t - type ?b - box ?s - state)" << std::endl;
	myfile << "\t(Rcan_fit_inside ?t - type ?b - box ?s - state)" << std::endl;
	myfile << "\t(can_stack_on ?o1 ?o2 - object ?s - state)" << std::endl;
	myfile << "\t(Rcan_stack_on ?o1 ?o2 - object ?s - state)" << std::endl;
	myfile << "\t(inside ?o - object ?b - box ?s - state)" << std::endl;
	myfile << "\t(Rinside ?o - object ?b - box ?s - state)" << std::endl;
	
	myfile << "\t(connected ?from ?to - waypoint)" << std::endl;
	myfile << "\t(is_of_type ?o - object ?t -type ?s - state)" << std::endl;
	myfile << "\t(Ris_of_type ?o - object ?t -type ?s - state)" << std::endl;
	
	myfile << "\t(part-of ?s - state ?kb - knowledgebase)" << std::endl;
	myfile << "\t(current_kb ?kb - knowledgebase)" << std::endl;
	myfile << "\t(parent ?kb ?kb2 - knowledgebase)" << std::endl;
	
	// Test, only allow an observation action once.
	myfile << "\t(has_checked_type ?o - object ?t - type)" << std::endl;
	
	myfile << std::endl;
	myfile << "\t;; Bookkeeping predicates." << std::endl;
	myfile << "\t(next ?l ?l2 - level)" << std::endl;
	myfile << "\t(lev ?l - LEVEL)" << std::endl;
	myfile << "\t(m ?s - STATE)" << std::endl;
	myfile << "\t(stack ?s - STATE ?l - LEVEL)" << std::endl;
	
	myfile << "\t(resolve-axioms)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	myfile << "(:constants" << std::endl;
	myfile << "\t; All the balls." << std::endl;
	
	for (std::vector<const Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
	{
		myfile << "\t" << (*ci)->name_ << " - waypoint" << std::endl;
	}

	myfile << "\t; The objects." << std::endl;
	for (std::vector<const Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
	{
		myfile << "\t" << (*ci)->name_ << " - object" << std::endl;
	}
	
	myfile << "\t; The box." << std::endl;
	for (std::vector<const Box*>::const_iterator ci = boxes.begin(); ci != boxes.end(); ++ci)
	{
		myfile << "\t" << (*ci)->name_ << " - box" << std::endl;
	}
	
	myfile << "\t; The types." << std::endl;
	for (std::vector<const Type*>::const_iterator ci = types.begin(); ci != types.end(); ++ci)
	{
		myfile << "\t" << (*ci)->name_ << " - type" << std::endl;
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
	 * Put object in a box.
	 */
	myfile << "(:action put_object_in_box" << std::endl;
	myfile << "\t:parameters (?v - robot ?wp ?wp2 - waypoint ?o1 - object ?b - box ?t - type)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	myfile << "\t\t(box_at ?b ?wp)" << std::endl;
	myfile << "\t\t(connected ?wp ?wp2)" << std::endl;
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			myfile << "\t\t(Rrobot_at ?v ?wp2 " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Rholding ?v ?o1 " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Rcan_fit_inside ?t ?b " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Ris_of_type ?o1 ?t " << (*ci)->state_name_ << ")" << std::endl;
		}
	}
	
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;

	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(when (m " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t(and" << std::endl;
		myfile << "\t\t\t\t(not (holding ?v ?o1 " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (Rholding ?v ?o1 " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(gripper_empty ?v " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(Rgripper_empty ?v " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(inside ?o1 ?b " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(Rinside ?o1 ?b " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t)" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**
	 * PICK-UP OBJECT.
	 */
	myfile << "(:action pickup_object" << std::endl;
	myfile << "\t:parameters (?v - robot ?wp ?wp2 - waypoint ?o - object ?t - type)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(connected ?wp ?wp2)" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			myfile << "\t\t(Rrobot_at ?v ?wp2 " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Robject_at ?o ?wp " << (*ci)->state_name_ << ")" << std::endl;
			//myfile << "\t\t(Rclear ?o " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Rgripper_empty ?v " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Rcan_pickup ?v ?t " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Ris_of_type ?o ?t " << (*ci)->state_name_ << ")" << std::endl;
		}
	}
	
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(when (m " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t(and" << std::endl;
		myfile << "\t\t\t\t(not (gripper_empty ?v " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (Rgripper_empty ?v " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (object_at ?o ?wp " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (Robject_at ?o ?wp " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(is_not_occupied ?wp " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(Ris_not_occupied ?wp " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(holding ?v ?o " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(Rholding ?v ?o " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t)" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**
	 * PUT-DOWN OBJECT.
	 */
	myfile << "(:action putdown_object" << std::endl;
	myfile << "\t:parameters (?v - robot ?wp ?wp2 - waypoint ?o - object)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	myfile << "\t\t(connected ?wp ?wp2)" << std::endl;
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			myfile << "\t\t(Rrobot_at ?v ?wp " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Rholding ?v ?o " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Ris_not_occupied ?wp2 " << (*ci)->state_name_ << ")" << std::endl;
		}
	}
	
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(when (m " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t(and" << std::endl;
		myfile << "\t\t\t\t(not (holding ?v ?o " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (Rholding ?v ?o " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (is_not_occupied ?wp2 " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (Ris_not_occupied ?wp2 " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(gripper_empty ?v " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(Rgripper_empty ?v " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(object_at ?o ?wp2 " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(Robject_at ?o ?wp2 " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t)" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**
	 * GOTO WAYPOINT.
	 */
	myfile << "(:action goto_waypoint" << std::endl;
	myfile << "\t:parameters (?v - robot ?from ?to - waypoint)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	myfile << "\t\t(connected ?from ?to)" << std::endl;
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			myfile << "\t\t(Rrobot_at ?v ?from " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Ris_not_occupied ?to " << (*ci)->state_name_ << ")" << std::endl;
		}
	}
	
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(when (m " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t(and" << std::endl;
		myfile << "\t\t\t\t(not (robot_at ?v ?from " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (Rrobot_at ?v ?from " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (is_not_occupied ?to " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (Ris_not_occupied ?to " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(robot_at ?v ?to " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(Rrobot_at ?v ?to " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(is_not_occupied ?from " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(Ris_not_occupied ?from " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t)" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**
	 * PUSH OBJECT.
	 *
	myfile << "(:action push_object" << std::endl;
	myfile << "\t:parameters (?v - robot ?ob - object ?t - type ?from ?to ?obw - waypoint)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			myfile << "\t\t(Rrobot_at ?v ?from " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Robject_at ?ob ?obw " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Rpush_location ?ob ?from " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Rcan_push ?v ?t " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Ris_of_type ?ob ?t " << (*ci)->state_name_ << ")" << std::endl;
		}
	}
	
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(when (m " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t(and" << std::endl;
		myfile << "\t\t\t\t(not (robot_at ?v ?from " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (Rrobot_at ?v ?from " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (object_at ?ob ?from " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (Robject_at ?ob ?from " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(robot_at ?v ?to " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(Rrobot_at ?v ?to " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(object_at ?ob ?to " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(Robject_at ?ob ?to " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t)" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	*/
	
	/**
	 * TIDY OBJECT.
	 */
	myfile << "(:action tidy_object" << std::endl;
	myfile << "\t:parameters (?v - robot ?o - object ?b - box ?t - type)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			myfile << "\t\t(Ris_of_type ?o ?t " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Rinside ?o ?b " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Rcan_fit_inside ?t ?b " << (*ci)->state_name_ << ")" << std::endl;
		}
	}
	
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(when (m " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t(and" << std::endl;
		myfile << "\t\t\t\t(tidy ?o " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(Rtidy ?o " << (*ci)->state_name_ << ")" << std::endl;
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
	 * Sense the type of an object.
	 */
	myfile << ";; Sense the type of object." << std::endl;
	myfile << "(:action observe-is_of_type" << std::endl;

	myfile << "\t:parameters (?o - object ?t - type ?v - robot ?wp ?wp2 - waypoint ?l ?l2 - level ?kb - knowledgebase)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	myfile << "\t\t(next ?l ?l2)" << std::endl;
	myfile << "\t\t(lev ?l)" << std::endl;
	
	myfile << "\t\t(current_kb ?kb)" << std::endl;
	myfile << "\t\t(not (current_kb basis_kb))" << std::endl;
	myfile << "\t\t(not (has_checked_type ?o ?t))" << std::endl;
	myfile << std::endl;
	myfile << "\t\t(connected ?wp ?wp2)" << std::endl;
	
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			myfile << "\t\t(Rrobot_at ?v ?wp " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Robject_at ?o ?wp2 " << (*ci)->state_name_ << ")" << std::endl;
		}
	}

	myfile << "\t\t;; This action is only applicable if there are world states where the outcome can be different." << std::endl;
	
	myfile << "\t\t(exists (?s - state) (and (m ?s) (is_of_type ?o ?t ?s) (part-of ?s ?kb)))" << std::endl;
	myfile << "\t\t(exists (?s - state) (and (m ?s) (not (is_of_type ?o ?t ?s)) (part-of ?s ?kb)))" << std::endl;;
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t(not (lev ?l))" << std::endl;
	myfile << "\t\t(lev ?l2)" << std::endl;
	myfile << "\t\t(has_checked_type ?o ?t)" << std::endl;
	myfile << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(when (and (m " << (*ci)->state_name_ << ") (not (is_of_type ?o ?t " << (*ci)->state_name_ << ")))" << std::endl;
		myfile << "\t\t\t(and (stack " << (*ci)->state_name_ << " ?l) (not (m " << (*ci)->state_name_ << ")))" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	myfile << "\t\t(resolve-axioms)" << std::endl;
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**
	 * Recall a previously performed observation.
	 */
	myfile << ";; Sense the type of object." << std::endl;
	myfile << "(:action recall-observe-is_of_type" << std::endl;

	myfile << "\t:parameters (?o - object ?t - type ?l ?l2 - level ?kb - knowledgebase)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	myfile << "\t\t(next ?l ?l2)" << std::endl;
	myfile << "\t\t(lev ?l)" << std::endl;
	
	myfile << "\t\t(current_kb ?kb)" << std::endl;
	myfile << "\t\t(not (current_kb basis_kb))" << std::endl;
	myfile << "\t\t(has_checked_type ?o ?t)" << std::endl;

	myfile << "\t\t;; This action is only applicable if there are world states where the outcome can be different." << std::endl;
	
	myfile << "\t\t(exists (?s - state) (and (m ?s) (is_of_type ?o ?t ?s) (part-of ?s ?kb)))" << std::endl;
	myfile << "\t\t(exists (?s - state) (and (m ?s) (not (is_of_type ?o ?t ?s)) (part-of ?s ?kb)))" << std::endl;;
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t(not (lev ?l))" << std::endl;
	myfile << "\t\t(lev ?l2)" << std::endl;
	myfile << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(when (and (m " << (*ci)->state_name_ << ") (not (is_of_type ?o ?t " << (*ci)->state_name_ << ")))" << std::endl;
		myfile << "\t\t\t(and (stack " << (*ci)->state_name_ << " ?l) (not (m " << (*ci)->state_name_ << ")))" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	myfile << "\t\t(resolve-axioms)" << std::endl;
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**
	 * Sense whether a type of object can be pushed.
	 *
	myfile << ";; Sense the type of object." << std::endl;
	myfile << "(:action test-push-affordability" << std::endl;

	myfile << "\t:parameters (?t - type ?o - object ?v - robot ?wp - waypoint ?l ?l2 - level ?kb - knowledgebase)" << std::endl;
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
			myfile << "\t\t(Rrobot_at ?v ?wp " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Robject_at ?o ?wp " << (*ci)->state_name_ << ")" << std::endl;
		}
	}

	myfile << "\t\t;; This action is only applicable if there are world states where the outcome can be different." << std::endl;
	
	myfile << "\t\t(exists (?s - state) (and (m ?s) (can_push ?v ?t ?s) (part-of ?s ?kb)))" << std::endl;
	myfile << "\t\t(exists (?s - state) (and (m ?s) (not (can_push ?v ?t ?s)) (part-of ?s ?kb)))" << std::endl;;
	
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t(not (lev ?l))" << std::endl;
	myfile << "\t\t(lev ?l2)" << std::endl;
	myfile << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(when (and (m " << (*ci)->state_name_ << ") (not (can_push ?v ?t " << (*ci)->state_name_ << ")))" << std::endl;
		myfile << "\t\t\t(and (stack " << (*ci)->state_name_ << " ?l) (not (m " << (*ci)->state_name_ << ")))" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	myfile << "\t\t(resolve-axioms)" << std::endl;
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	**
	 * Sense whether a type of object can be picked up.
	 *
	myfile << ";; Sense the type of object." << std::endl;
	myfile << "(:action test-pickup-affordability" << std::endl;

	myfile << "\t:parameters (?t - type ?o - object ?v - robot ?wp - waypoint ?l ?l2 - level ?kb - knowledgebase)" << std::endl;

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
			myfile << "\t\t(Rrobot_at ?v ?wp " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Robject_at ?o ?wp " << (*ci)->state_name_ << ")" << std::endl;
		}
	}

	myfile << "\t\t;; This action is only applicable if there are world states where the outcome can be different." << std::endl;
	
	myfile << "\t\t(exists (?s - state) (and (m ?s) (can_pickup ?v ?t ?s) (part-of ?s ?kb)))" << std::endl;
	myfile << "\t\t(exists (?s - state) (and (m ?s) (not (can_pickup ?v ?t ?s)) (part-of ?s ?kb)))" << std::endl;;
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t(not (lev ?l))" << std::endl;
	myfile << "\t\t(lev ?l2)" << std::endl;
	myfile << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(when (and (m " << (*ci)->state_name_ << ") (not (can_pickup ?v ?t " << (*ci)->state_name_ << ")))" << std::endl;
		myfile << "\t\t\t(and (stack " << (*ci)->state_name_ << " ?l) (not (m " << (*ci)->state_name_ << ")))" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	myfile << "\t\t(resolve-axioms)" << std::endl;
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	**
	 * Sense whether an object can be stacked on top of another object.
	 *
	myfile << ";; Sense the type of object." << std::endl;
	myfile << "(:action observe-stackable-affordability" << std::endl;

	myfile << "\t:parameters (?o1 ?o2 - object ?v - robot ?wp - waypoint ?l ?l2 - level ?kb - knowledgebase)" << std::endl;
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
			myfile << "\t\t(Rrobot_at ?v ?wp " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Robject_at ?o1 ?wp " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Robject_at ?o2 ?wp " << (*ci)->state_name_ << ")" << std::endl;
		}
	}

	myfile << "\t\t;; This action is only applicable if there are world states where the outcome can be different." << std::endl;
	
	myfile << "\t\t(exists (?s - state) (and (m ?s) (can_stack_on ?o1 ?o2 ?s) (part-of ?s ?kb)))" << std::endl;
	myfile << "\t\t(exists (?s - state) (and (m ?s) (not (can_stack_on ?o1 ?o2 ?s)) (part-of ?s ?kb)))" << std::endl;;
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t(not (lev ?l))" << std::endl;
	myfile << "\t\t(lev ?l2)" << std::endl;
	myfile << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(when (and (m " << (*ci)->state_name_ << ") (not (can_stack_on ?o1 ?o2 " << (*ci)->state_name_ << ")))" << std::endl;
		myfile << "\t\t\t(and (stack " << (*ci)->state_name_ << " ?l) (not (m " << (*ci)->state_name_ << ")))" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	myfile << "\t\t(resolve-axioms)" << std::endl;
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	**
	 * Sense whether an object is at a location.
	 *
	myfile << ";; Sense the location of an object." << std::endl;
	myfile << "(:action observe-object-location" << std::endl;

	myfile << "\t:parameters (?o - object ?v - robot ?wp - waypoint ?l ?l2 - level ?kb - knowledgebase)" << std::endl;
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
			myfile << "\t\t(Rrobot_at ?v ?wp " << (*ci)->state_name_ << ")" << std::endl;
		}
	}

	myfile << "\t\t;; This action is only applicable if there are world states where the outcome can be different." << std::endl;
	
	myfile << "\t\t(exists (?s - state) (and (m ?s) (object_at ?o ?wp ?s) (part-of ?s ?kb)))" << std::endl;
	myfile << "\t\t(exists (?s - state) (and (m ?s) (not (object_at ?o ?wp ?s)) (part-of ?s ?kb)))" << std::endl;;
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t(not (lev ?l))" << std::endl;
	myfile << "\t\t(lev ?l2)" << std::endl;
	myfile << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(when (and (m " << (*ci)->state_name_ << ") (not (object_at ?o ?wp " << (*ci)->state_name_ << ")))" << std::endl;
		myfile << "\t\t\t(and (stack " << (*ci)->state_name_ << " ?l) (not (m " << (*ci)->state_name_ << ")))" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	myfile << "\t\t(resolve-axioms)" << std::endl;
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	*/
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
	myfile << "(:action raminificate" << std::endl;
	myfile << "\t:parameters ()" << std::endl;
	myfile << "\t:precondition (resolve-axioms)" << std::endl;
	myfile << "\t:effect (and " << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		const State* state = *ci;
		
		myfile << "\t\t(when (or (gripper_empty robot " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
		myfile << "\t\t\t(Rgripper_empty robot " << state->state_name_ << ")" << std::endl;
		myfile << "\t\t)" << std::endl;
		
		myfile << "\t\t(when (and (not (gripper_empty robot " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
		myfile << "\t\t\t(not (Rgripper_empty robot " << state->state_name_ << "))" << std::endl;
		myfile << "\t\t)" << std::endl;
		
		for (std::vector<const Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
		{
			const Location* location = *ci;
			
			myfile << "\t\t(when (or (is_not_occupied " << location->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
			myfile << "\t\t\t(Ris_not_occupied " << location->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			myfile << "\t\t(when (and (not (is_not_occupied " << location->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t(not (Ris_not_occupied " << location->name_ << " " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t)" << std::endl;
		}
		
		for (std::vector<const Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
		{
			const Object* object = *ci;
			myfile << "\t\t(when (or (holding robot " << object->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
			myfile << "\t\t\t(Rholding robot " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			myfile << "\t\t(when (and (not (holding robot " << object->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t(not (Rholding robot " << object->name_ << " " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t)" << std::endl;
			/*
			myfile << "\t\t(when (or (clear " << object->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
			myfile << "\t\t\t(Rclear " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			myfile << "\t\t(when (and (not (clear " << object->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t(not (Rclear " << object->name_ << " " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t)" << std::endl;
			*/
			myfile << "\t\t(when (or (tidy " << object->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
			myfile << "\t\t\t(Rtidy " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			myfile << "\t\t(when (and (not (tidy " << object->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t(not (Rtidy " << object->name_ << " " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t)" << std::endl;
			/*
			for (std::vector<const Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
			{
				const Object* other_object = *ci;
				myfile << "\t\t(when (or (on " << object->name_ << " " << other_object->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
				myfile << "\t\t\t(Ron " << object->name_ << " " << other_object->name_ << " " << state->state_name_ << ")" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				myfile << "\t\t(when (and (not (on " << object->name_ << " " << other_object->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t(not (Ron " << object->name_ << " " << other_object->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				myfile << "\t\t(when (or (can_stack_on " << object->name_ << " " << other_object->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
				myfile << "\t\t\t(Rcan_stack_on " << object->name_ << " " << other_object->name_ << " " << state->state_name_ << ")" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				myfile << "\t\t(when (and (not (can_stack_on " << object->name_ << " " << other_object->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t(not (Rcan_stack_on " << object->name_ << " " << other_object->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t)" << std::endl;
			}
			*/
			for (std::vector<const Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
			{
				const Location* location = *ci;
				/*
				myfile << "\t\t(when (or (push_location " << object->name_ << " " << location->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
				myfile << "\t\t\t(Rpush_location " << object->name_ << " " << location->name_ << " " << state->state_name_ << ")" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				myfile << "\t\t(when (and (not (push_location " << object->name_ << " " << location->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t(not (Rpush_location " << object->name_ << " " << location->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t)" << std::endl;
				*/
				myfile << "\t\t(when (or (object_at " << object->name_ << " " << location->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
				myfile << "\t\t\t(Robject_at " << object->name_ << " " << location->name_ << " " << state->state_name_ << ")" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				myfile << "\t\t(when (and (not (object_at " << object->name_ << " " << location->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t(not (Robject_at " << object->name_ << " " << location->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t)" << std::endl;
			}
			
			for (std::vector<const Box*>::const_iterator ci = boxes.begin(); ci != boxes.end(); ++ci)
			{
				const Box* box = *ci;
				myfile << "\t\t(when (or (inside " << object->name_ << " " << box->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
				myfile << "\t\t\t(Rinside " << object->name_ << " " << box->name_ << " " << state->state_name_ << ")" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				myfile << "\t\t(when (and (not (inside " << object->name_ << " " << box->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t(not (Rinside " << object->name_ << " " << box->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t)" << std::endl;
			}
			
			for (std::vector<const Type*>::const_iterator ci = types.begin(); ci != types.end(); ++ci)
			{
				const Type* type = *ci;
				myfile << "\t\t(when (or (is_of_type " << object->name_ << " " << type->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
				myfile << "\t\t\t(Ris_of_type " << object->name_ << " " << type->name_ << " " << state->state_name_ << ")" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				myfile << "\t\t(when (and (not (is_of_type " << object->name_ << " " << type->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t(not (Ris_of_type " << object->name_ << " " << type->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t)" << std::endl;
			}
		}
		
		for (std::vector<const Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
		{
			const Location* location = *ci;
			myfile << "\t\t(when (or (robot_at robot " << location->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
			myfile << "\t\t\t(Rrobot_at robot " << location->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			myfile << "\t\t(when (and (not (robot_at robot " << location->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t(not (Rrobot_at robot " << location->name_ << " " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t)" << std::endl;
			/*
			myfile << "\t\t(when (or (box_at " << location->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
			myfile << "\t\t\t(Rbox_at " << location->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			myfile << "\t\t(when (and (not (box_at " << location->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t(not (Rbox_at " << location->name_ << " " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t)" << std::endl;
			*/
		}
		
		for (std::vector<const Type*>::const_iterator ci = types.begin(); ci != types.end(); ++ci)
		{
			const Type* type = *ci;
			myfile << "\t\t(when (or (can_pickup robot " << type->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
			myfile << "\t\t\t(Rcan_pickup robot " << type->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			myfile << "\t\t(when (and (not (can_pickup robot " << type->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t(not (Rcan_pickup robot " << type->name_ << " " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			myfile << "\t\t(when (or (can_push robot " << type->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
			myfile << "\t\t\t(Rcan_push robot " << type->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			myfile << "\t\t(when (and (not (can_push robot " << type->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t(not (Rcan_push robot " << type->name_ << " " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			for (std::vector<const Box*>::const_iterator ci = boxes.begin(); ci != boxes.end(); ++ci)
			{
				const Box* box = *ci;
								
				myfile << "\t\t(when (or (can_fit_inside " << type->name_ << " " << box->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
				myfile << "\t\t\t(Rcan_fit_inside " << type->name_ << " " << box->name_ << " " << state->state_name_ << ")" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				myfile << "\t\t(when (and (not (can_fit_inside " << type->name_ << " " << box->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t(not (Rcan_fit_inside " << type->name_ << " " << box->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t)" << std::endl;
			}
			
			for (std::vector<const Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
			{
				const Location* location = *ci;
				myfile << "\t\t(when (or (tidy_location " << type->name_ << " " << location->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
				myfile << "\t\t\t(Rtidy_location " << type->name_ << " " << location->name_ << " " << state->state_name_ << ")" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				myfile << "\t\t(when (and (not (tidy_location " << type->name_ << " " << location->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t(not (Rtidy_location " << type->name_ << " " << location->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t)" << std::endl;
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
			
			myfile << "\t\t(when (and (part-of " << state->state_name_ << " ?old_kb) (gripper_empty robot " << state->state_name_ << ") (part-of " << state2->state_name_ << " ?new_kb))" << std::endl;
			myfile << "\t\t\t(and " << std::endl;
			myfile << "\t\t\t\t(not (Rgripper_empty robot " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t\t(not (gripper_empty robot " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t\t(Rgripper_empty robot " << state2->state_name_ << ")" << std::endl;
			myfile << "\t\t\t\t(gripper_empty robot " << state2->state_name_ << ")" << std::endl;
			myfile << "\t\t\t)" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			for (std::vector<const Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
			{
				const Object* object = *ci;
				myfile << "\t\t(when (and (part-of " << state->state_name_ << " ?old_kb) (holding robot " << object->name_ << " " << state->state_name_ << ") (part-of " << state2->state_name_ << " ?new_kb))" << std::endl;
				myfile << "\t\t\t(and " << std::endl;
				myfile << "\t\t\t\t(not (Rholding robot " << object->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(not (holding robot " << object->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(Rholding robot " << object->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t\t(holding robot " << object->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t)" << std::endl;
				myfile << "\t\t)" << std::endl;
				/*
				myfile << "\t\t(when (and (part-of " << state->state_name_ << " ?old_kb) (clear " << object->name_ << " " << state->state_name_ << ") (part-of " << state2->state_name_ << " ?new_kb))" << std::endl;
				myfile << "\t\t\t(and " << std::endl;
				myfile << "\t\t\t\t(not (Rclear " << object->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(not (clear " << object->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(Rclear " << object->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t\t(clear " << object->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t)" << std::endl;
				myfile << "\t\t)" << std::endl;
				*/
				myfile << "\t\t(when (and (part-of " << state->state_name_ << " ?old_kb) (tidy " << object->name_ << " " << state->state_name_ << ") (part-of " << state2->state_name_ << " ?new_kb))" << std::endl;
				myfile << "\t\t\t(and " << std::endl;
				myfile << "\t\t\t\t(not (Rtidy " << object->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(not (tidy " << object->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(Rtidy " << object->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t\t(tidy " << object->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t)" << std::endl;
				myfile << "\t\t)" << std::endl;
				/*
				for (std::vector<const Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
				{
					const Object* other_object = *ci;
					myfile << "\t\t(when (and (part-of " << state->state_name_ << " ?old_kb) (on " << object->name_ << " " << other_object->name_ << " " << state->state_name_ << ") (part-of " << state2->state_name_ << " ?new_kb))" << std::endl;
					myfile << "\t\t\t(and " << std::endl;
					myfile << "\t\t\t\t(not (Ron " << object->name_ << " " << other_object->name_ << " " << state->state_name_ << "))" << std::endl;
					myfile << "\t\t\t\t(not (on " << object->name_ << " " << other_object->name_ << " " << state->state_name_ << "))" << std::endl;
					myfile << "\t\t\t\t(Ron " << object->name_ << " " << other_object->name_ << " " << state2->state_name_ << ")" << std::endl;
					myfile << "\t\t\t\t(on " << object->name_ << " " << other_object->name_ << " " << state2->state_name_ << ")" << std::endl;
					myfile << "\t\t\t)" << std::endl;
					myfile << "\t\t)" << std::endl;
					
					myfile << "\t\t(when (and (part-of " << state->state_name_ << " ?old_kb) (can_stack_on " << object->name_ << " " << other_object->name_ << " " << state->state_name_ << ") (part-of " << state2->state_name_ << " ?new_kb))" << std::endl;
					myfile << "\t\t\t(and " << std::endl;
					myfile << "\t\t\t\t(not (Rcan_stack_on " << object->name_ << " " << other_object->name_ << " " << state->state_name_ << "))" << std::endl;
					myfile << "\t\t\t\t(not (can_stack_on " << object->name_ << " " << other_object->name_ << " " << state->state_name_ << "))" << std::endl;
					myfile << "\t\t\t\t(Rcan_stack_on " << object->name_ << " " << other_object->name_ << " " << state2->state_name_ << ")" << std::endl;
					myfile << "\t\t\t\t(can_stack_on " << object->name_ << " " << other_object->name_ << " " << state2->state_name_ << ")" << std::endl;
					myfile << "\t\t\t)" << std::endl;
					myfile << "\t\t)" << std::endl;
				}
				*/
				for (std::vector<const Box*>::const_iterator ci = boxes.begin(); ci != boxes.end(); ++ci)
				{
					const Box* box = *ci;
					myfile << "\t\t(when (and (part-of " << state->state_name_ << " ?old_kb) (inside " << object->name_ << " " << box->name_ << " " << state->state_name_ << ") (part-of " << state2->state_name_ << " ?new_kb))" << std::endl;
					myfile << "\t\t\t(and " << std::endl;
					myfile << "\t\t\t\t(not (Rinside " << object->name_ << " " << box->name_ << " " << state->state_name_ << "))" << std::endl;
					myfile << "\t\t\t\t(not (inside " << object->name_ << " " << box->name_ << " " << state->state_name_ << "))" << std::endl;
					myfile << "\t\t\t\t(Rinside " << object->name_ << " " << box->name_ << " " << state2->state_name_ << ")" << std::endl;
					myfile << "\t\t\t\t(inside " << object->name_ << " " << box->name_ << " " << state2->state_name_ << ")" << std::endl;
					myfile << "\t\t\t)" << std::endl;
					myfile << "\t\t)" << std::endl;
				}
			}
			
			for (std::vector<const Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
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
				
				myfile << "\t\t(when (and (part-of " << state->state_name_ << " ?old_kb) (is_not_occupied " << location->name_ << " " << state->state_name_ << ") (part-of " << state2->state_name_ << " ?new_kb))" << std::endl;
				myfile << "\t\t\t(and " << std::endl;
				myfile << "\t\t\t\t(not (Ris_not_occupied " << location->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(not (is_not_occupied " << location->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(Ris_not_occupied " << location->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t\t(is_not_occupied " << location->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t)" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				
				/*
				myfile << "\t\t(when (and (part-of " << state->state_name_ << " ?old_kb) (box_at robot " << location->name_ << " " << state->state_name_ << ") (part-of " << state2->state_name_ << " ?new_kb))" << std::endl;
				myfile << "\t\t\t(and " << std::endl;
				myfile << "\t\t\t\t(not (Rbox_at robot " << location->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(not (box_at robot " << location->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(Rbox_at robot " << location->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t\t(box_at robot " << location->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t)" << std::endl;
				myfile << "\t\t)" << std::endl;
				*/
				for (std::vector<const Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
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
					/*
					myfile << "\t\t(when (and (part-of " << state->state_name_ << " ?old_kb) (push_location " << object->name_ << " " << location->name_ << " " << state->state_name_ << ") (part-of " << state2->state_name_ << " ?new_kb))" << std::endl;
					myfile << "\t\t\t(and " << std::endl;
					myfile << "\t\t\t\t(not (Rpush_location " << object->name_ << " " << location->name_ << " " << state->state_name_ << "))" << std::endl;
					myfile << "\t\t\t\t(not (push_location " << object->name_ << " " << location->name_ << " " << state->state_name_ << "))" << std::endl;
					myfile << "\t\t\t\t(Rpush_location " << object->name_ << " " << location->name_ << " " << state2->state_name_ << ")" << std::endl;
					myfile << "\t\t\t\t(push_location " << object->name_ << " " << location->name_ << " " << state2->state_name_ << ")" << std::endl;
					myfile << "\t\t\t)" << std::endl;
					myfile << "\t\t)" << std::endl;
					*/
				}
			}
			
			for (std::vector<const Type*>::const_iterator ci = types.begin(); ci != types.end(); ++ci)
			{
				const Type* type = *ci;
				myfile << "\t\t(when (and (part-of " << state->state_name_ << " ?old_kb) (can_pickup robot " << type->name_ << " " << state->state_name_ << ") (part-of " << state2->state_name_ << " ?new_kb))" << std::endl;
				myfile << "\t\t\t(and " << std::endl;
				myfile << "\t\t\t\t(not (Rcan_pickup robot " << type->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(not (can_pickup robot " << type->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(Rcan_pickup robot " << type->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t\t(can_pickup robot " << type->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t)" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				myfile << "\t\t(when (and (part-of " << state->state_name_ << " ?old_kb) (can_push robot " << type->name_ << " " << state->state_name_ << ") (part-of " << state2->state_name_ << " ?new_kb))" << std::endl;
				myfile << "\t\t\t(and " << std::endl;
				myfile << "\t\t\t\t(not (Rcan_push robot " << type->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(not (can_push robot " << type->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(Rcan_push robot " << type->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t\t(can_push robot " << type->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t)" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				for (std::vector<const Box*>::const_iterator ci = boxes.begin(); ci != boxes.end(); ++ci)
				{
					const Box* box = *ci;
					
					myfile << "\t\t(when (and (part-of " << state->state_name_ << " ?old_kb) (can_fit_inside " << type->name_ << " " << box->name_ << " " << state->state_name_ << ") (part-of " << state2->state_name_ << " ?new_kb))" << std::endl;
					myfile << "\t\t\t(and " << std::endl;
					myfile << "\t\t\t\t(not (Rcan_fit_inside " << type->name_ << " " << box->name_ << " " << state->state_name_ << "))" << std::endl;
					myfile << "\t\t\t\t(not (can_fit_inside " << type->name_ << " " << box->name_ << " " << state->state_name_ << "))" << std::endl;
					myfile << "\t\t\t\t(Rcan_fit_inside " << type->name_ << " " << box->name_ << " " << state2->state_name_ << ")" << std::endl;
					myfile << "\t\t\t\t(can_fit_inside " << type->name_ << " " << box->name_ << " " << state2->state_name_ << ")" << std::endl;
					myfile << "\t\t\t)" << std::endl;
					myfile << "\t\t)" << std::endl;
				}
				
				for (std::vector<const Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
				{
					const Location* location = *ci;
					myfile << "\t\t(when (and (part-of " << state->state_name_ << " ?old_kb) (tidy_location " << type->name_ << " " << location->name_ << " " << state->state_name_ << ") (part-of " << state2->state_name_ << " ?new_kb))" << std::endl;
					myfile << "\t\t\t(and " << std::endl;
					myfile << "\t\t\t\t(not (Rtidy_location " << type->name_ << " " << location->name_ << " " << state->state_name_ << "))" << std::endl;
					myfile << "\t\t\t\t(not (tidy_location " << type->name_ << " " << location->name_ << " " << state->state_name_ << "))" << std::endl;
					myfile << "\t\t\t\t(Rtidy_location " << type->name_ << " " << location->name_ << " " << state2->state_name_ << ")" << std::endl;
					myfile << "\t\t\t\t(tidy_location " << type->name_ << " " << location->name_ << " " << state2->state_name_ << ")" << std::endl;
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
	myfile << "\t:parameters (?old_kb ?new_kb - knowledgebase)" << std::endl;
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
	for (std::vector<const Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
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
	/*
	// Make sure there is no discrepancy in which locations are accessible and which are not.
	for (std::vector<const Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
	{
		const Location* location = *ci;
		myfile << "\t\t(or" << std::endl;
		myfile << "\t\t\t(and" << std::endl;
		for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
		{
			const State* state = *ci;
			// Make sure the state of the toilets are the same.
			myfile << "\t\t\t\t(or " << std::endl;
			myfile << "\t\t\t\t\t(not (part-of " << (*ci)->state_name_ << " ?old_kb))" << std::endl;
			myfile << "\t\t\t\t\t(is_not_occupied " << location->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t\t\t)" << std::endl;
		}
		myfile << "\t\t\t)" << std::endl;
		
		myfile << "\t\t\t(and" << std::endl;
		for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
		{
			const State* state = *ci;
			// Make sure the state of the toilets are the same.
			myfile << "\t\t\t\t(or " << std::endl;
			myfile << "\t\t\t\t\t(not (part-of " << (*ci)->state_name_ << " ?old_kb))" << std::endl;
			myfile << "\t\t\t\t\t(not (is_not_occupied " << location->name_ << " " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t\t)" << std::endl;
		}
		myfile << "\t\t\t)" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	*/
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
				// Make sure the state of the toilets are the same.
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
		
		myfile << "\t\t\t(and" << std::endl;
		for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
		{
			const State* state = *ci;
			// Make sure the state of the toilets are the same.
			myfile << "\t\t\t\t(or " << std::endl;
			myfile << "\t\t\t\t\t(not (part-of " << (*ci)->state_name_ << " ?old_kb))" << std::endl;
			myfile << "\t\t\t\t\t(tidy " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t\t\t)" << std::endl;
		}
		myfile << "\t\t\t)" << std::endl;
		
		myfile << "\t\t)" << std::endl;
	}*/
	
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
	
	for (std::vector<const Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
	{
		const Object* object = *ci;
		for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
		{
			// Deal with the location of the robot.
			const State* state = *ci;
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
			
			// Tidy.
			myfile << "\t\t(when (and " << std::endl;
			myfile << "\t\t\t\t;; For every state ?s, ?s2" << std::endl;
		
			for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
			{
				const State* state2 = *ci;
				myfile << "\t\t\t\t\t(or " << std::endl;
				myfile << "\t\t\t\t\t\t(not (part-of " << state2->state_name_ << " ?old_kb))" << std::endl;
				myfile << "\t\t\t\t\t\t(tidy " << object->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t\t\t)" << std::endl;
				
			}
			myfile << "\t\t\t\t\t(part-of " << state->state_name_ << " ?new_kb)" << std::endl;
			myfile << "\t\t\t)" << std::endl;

			myfile << "\t\t\t;; Conditional effects" << std::endl;
			myfile << "\t\t\t(and " << std::endl;
			for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
			{
				const State* state2 = *ci;
				myfile << "\t\t\t\t(not (tidy " << object->name_ << " " << state2->state_name_ << "))" << std::endl;
			}
			myfile << "\t\t\t\t(tidy " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t\t)" << std::endl;
			myfile << "\t\t)" << std::endl;
			/*
			// Clear.
			myfile << "\t\t(when (and " << std::endl;
			myfile << "\t\t\t\t;; For every state ?s, ?s2" << std::endl;
		
			for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
			{
				const State* state2 = *ci;
				myfile << "\t\t\t\t\t(or " << std::endl;
				myfile << "\t\t\t\t\t\t(not (part-of " << state2->state_name_ << " ?old_kb))" << std::endl;
				myfile << "\t\t\t\t\t\t(clear " << object->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t\t\t)" << std::endl;
				
			}
			myfile << "\t\t\t\t\t(part-of " << state->state_name_ << " ?new_kb)" << std::endl;
			myfile << "\t\t\t)" << std::endl;

			myfile << "\t\t\t;; Conditional effects" << std::endl;
			myfile << "\t\t\t(and " << std::endl;
			for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
			{
				const State* state2 = *ci;
				myfile << "\t\t\t\t(not (clear " << object->name_ << " " << state2->state_name_ << "))" << std::endl;
			}
			myfile << "\t\t\t\t(clear " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t\t)" << std::endl;
			myfile << "\t\t)" << std::endl;
			*/
		}
		
		// Objects in boxes.
		for (std::vector<const Box*>::const_iterator ci = boxes.begin(); ci != boxes.end(); ++ci)
		{
			const Box* box = *ci;
			for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
			{
				// Deal with the location of the robot.
				const State* state = *ci;
				myfile << "\t\t(when (and " << std::endl;
				myfile << "\t\t\t\t;; For every state ?s, ?s2" << std::endl;
			
				// Holding.
				for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
				{
					const State* state2 = *ci;
					myfile << "\t\t\t\t\t(or " << std::endl;
					myfile << "\t\t\t\t\t\t(not (part-of " << state2->state_name_ << " ?old_kb))" << std::endl;
					myfile << "\t\t\t\t\t\t(inside " << object->name_ << " " << box->name_ << " " << state2->state_name_ << ")" << std::endl;
					myfile << "\t\t\t\t\t)" << std::endl;
					
				}
				myfile << "\t\t\t\t\t(part-of " << state->state_name_ << " ?new_kb)" << std::endl;
				myfile << "\t\t\t)" << std::endl;

				myfile << "\t\t\t;; Conditional effects" << std::endl;
				myfile << "\t\t\t(and " << std::endl;
				for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
				{
					const State* state2 = *ci;
					myfile << "\t\t\t\t(not (inside " << object->name_ << " " << box->name_ << " " << state2->state_name_ << "))" << std::endl;
				}
				myfile << "\t\t\t\t(inside " << object->name_ << " " << box->name_ << " " << state->state_name_ << ")" << std::endl;
				myfile << "\t\t\t)" << std::endl;
				myfile << "\t\t)" << std::endl;
			}
		}
	}
	
	// Location of the agent.
	for (std::vector<const Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
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
			
			// Deal with whether a location is occupied or not.
			myfile << "\t\t(when (and " << std::endl;
			myfile << "\t\t\t\t;; For every state ?s, ?s2" << std::endl;
		
			for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
			{
				const State* state2 = *ci;
				myfile << "\t\t\t\t(and " << std::endl;
				myfile << "\t\t\t\t\t(or " << std::endl;
				myfile << "\t\t\t\t\t\t(not (part-of " << state2->state_name_ << " ?old_kb))" << std::endl;
				myfile << "\t\t\t\t\t\t(is_not_occupied " << location->name_ << " " << state2->state_name_ << ")" << std::endl;
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
				myfile << "\t\t\t\t(not (is_not_occupied " << location->name_ << " " << state2->state_name_ << "))" << std::endl;
			}
			myfile << "\t\t\t\t(is_not_occupied " << location->name_ << " " << state->state_name_ << ")" << std::endl;
			
			myfile << "\t\t\t)" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			// The locations of the objects.
			for (std::vector<const Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
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

void ContingentTidyPDDLGenerator::createPDDL(const std::string& path, const std::string& domain_file, const std::string& problem_file, const std::string& robot_location_predicate, const std::map<std::string, std::string>& object_to_location_mapping, std::map<std::string, std::vector<std::string> >& near_waypoint_mappings, const std::map<std::string, std::string>& object_to_type_mapping, const std::map<std::string, std::string>& box_to_location_mapping, const std::map<std::string, std::string>& box_to_type_mapping)
{
	// Now generate the waypoints / boxes / toys / etc.
	std::vector<const Location*> locations;
	std::vector<const Box*> boxes;
	std::vector<const Object*> objects;
	std::vector<const Type*> types;
	
	// Create all objects and the locations of where they are located.
	for (std::map<std::string, std::string>::const_iterator ci = object_to_location_mapping.begin(); ci != object_to_location_mapping.end(); ++ci) {
		const std::string& object_predicate = (*ci).first;
		const std::string& location_predicate = (*ci).second;
		
		Location* location = new Location(location_predicate, false);
		Object* object = new Object(object_predicate, *location);
		locations.push_back(location);
		objects.push_back(object);
		
		std::map<std::string, std::vector<std::string> >::const_iterator mi = near_waypoint_mappings.find(location_predicate);
		if (mi != near_waypoint_mappings.end())
		{
			const std::vector<std::string>& near_locations = (*mi).second;
			for (std::vector<std::string>::const_iterator ci = near_locations.begin(); ci != near_locations.end(); ++ci)
			{
				const std::string& near_location_name = *ci;
				Location* near_location = new Location(near_location_name, false);
				locations.push_back(near_location);
				location->near_locations_.push_back(near_location);
			}
		}
	}
	
	// Create all the boxes.
	for (std::map<std::string, std::string>::const_iterator ci = box_to_location_mapping.begin(); ci != box_to_location_mapping.end(); ++ci) {
		const std::string& box_predicate = (*ci).first;
		const std::string& location_predicate =(*ci).second;
		
		std::string box_type_predicate = (*box_to_type_mapping.find(box_predicate)).second;
		
		const Type* box_type = NULL;
		for (std::vector<const Type*>::const_iterator ci = types.begin(); ci != types.end(); ++ci) {
			const Type* existing_type = *ci;
			if (existing_type->name_ == box_type_predicate) {
				box_type = existing_type;
				break;
			}
		}
		
		if (box_type == NULL) {
			box_type = new Type(box_type_predicate);
			types.push_back(box_type);
		}
		std::vector<const Type*> box_types;
		box_types.push_back(box_type);
		
		std::vector<const Object*> empty_object_list;
		
		Location* box_location = new Location(location_predicate, true);
		Box* box = new Box(box_predicate, *box_location, box_types, empty_object_list);
		
		locations.push_back(box_location);
		boxes.push_back(box);
	}
	
	// Initialise the robot's location.
	Location* robot_location = new Location(robot_location_predicate, false);
	locations.push_back(robot_location);

	//std::cout << "(ContingentTidyPDDLGenerator) Creating all possible states..." << std::endl;

	std::vector<const KnowledgeBase*> knowledge_bases;
	//std::map<const Object*, const Location*> empty_object_location_mapping;
	std::map<const Object*, const Object*> empty_stacked_objects_mapping;
	std::map<const Object*, const Type*> empty_type_mapping;
	std::vector<const Type*> empty_pushable_types;
	std::vector<const Type*> empty_pickupable_types;
	State basic_state("basic"/*, empty_object_location_mapping*/, empty_stacked_objects_mapping, empty_type_mapping, empty_pushable_types, empty_pickupable_types);
	
	KnowledgeBase basis_kb("basis_kb");
	basis_kb.addState(basic_state);
	knowledge_bases.push_back(&basis_kb);
	
	unsigned int state_id = 0;
	
	// Create a new knowledge base for each object.
	std::stringstream ss;
	for (std::vector<const Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
	{
		const Object* object = *ci;
		ss.str(std::string());
		ss << "kb_" << object->name_;
		
		KnowledgeBase* kb_location = new KnowledgeBase(ss.str());
		basis_kb.addChild(*kb_location);
		knowledge_bases.push_back(kb_location);
		
		//std::map<const Object*, const Location*> object_location_mapping;
		std::map<const Object*, const Object*> stackable_mapping;
		std::map<const Object*, const Type*> type_mapping;
		std::vector<const Type*> pushable_types;
		std::vector<const Type*> pickupable_types;
		
		//for (std::vector<const Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
		//{
		//	const Location* location = *ci;
			for (std::vector<const Type*>::const_iterator ci = types.begin(); ci != types.end(); ++ci)
			{
				const Type* type = *ci;
				//object_location_mapping[object] = location;
				type_mapping[object] = type;
				ss.str(std::string());
				ss << "s" << state_id << "_pick";
				pickupable_types.push_back(type);
				State* state_pickupable = new State(ss.str()/*, object_location_mapping*/, stackable_mapping, type_mapping, pushable_types, pickupable_types);
				kb_location->addState(*state_pickupable);
				pickupable_types.clear();
				/*
				ss.str(std::string());
				ss << "s" << state_id << "_push";
				pickupable_types.clear();
				pushable_types.push_back(type);
				State* state_pushable = new State(ss.str(), object_location_mapping, stackable_mapping, type_mapping, pushable_types, pickupable_types);
				kb_location->addState(*state_pushable);
				pushable_types.clear();
				*/
				++state_id;
			}
		//}
	}
	
	
	
	ss.str(std::string());
	ss << path << domain_file;
	ROS_INFO("KCL: (ContingentTidyPDDLGenerator) Generate domain... %s", ss.str().c_str());
	generateDomainFile(ss.str(), basis_kb, knowledge_bases, *robot_location, locations, objects, boxes, types);
	ss.str(std::string());
	ss << path  << problem_file;
	ROS_INFO("KCL: (ContingentTidyPDDLGenerator) Generate problem... %s", ss.str().c_str());
	generateProblemFile(ss.str(), basis_kb, knowledge_bases, *robot_location, locations, objects, boxes, types);
}

};
