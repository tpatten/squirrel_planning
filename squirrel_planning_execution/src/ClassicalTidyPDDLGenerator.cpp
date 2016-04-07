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

#include "squirrel_planning_execution/ClassicalTidyPDDLGenerator.h"

namespace KCL_rosplan {

void ClassicalTidyPDDLGenerator::generateProblemFile(const std::string& file_name, const std::string& robot_location_predicate, const std::map<std::string, std::string>& object_to_location_mapping, const std::map<std::string, std::vector<std::string > >& near_location_mapping, const std::map<std::string, std::string>& object_to_type_mapping, const std::map<std::string, std::string>& box_to_location_mapping, const std::map<std::string, std::string>& box_to_type_mapping, const std::map<std::string, std::vector<std::string> >& near_box_location_mapping)
{
	std::ofstream myfile;
	myfile.open(file_name.c_str());
	myfile << "(define (problem Keys-0)" << std::endl;
	myfile << "(:domain find_key)" << std::endl;
	myfile << "(:objects" << std::endl;
	
	// Waypoints.
	myfile << robot_location_predicate << " ";
	for (std::map<std::string, std::string>::const_iterator ci = object_to_location_mapping.begin(); ci != object_to_location_mapping.end(); ++ci)
	{
		myfile << (*ci).second << " ";
	}
	for (std::map<std::string, std::string>::const_iterator ci = box_to_location_mapping.begin(); ci != box_to_location_mapping.end(); ++ci)
	{
		myfile << (*ci).second << " ";
	}
	for (std::map<std::string, std::vector<std::string> >::const_iterator ci = near_location_mapping.begin(); ci != near_location_mapping.end(); ++ci)
	{
		const std::vector<std::string>& near_locations = (*ci).second;
		for (std::vector<std::string>::const_iterator ci = near_locations.begin(); ci != near_locations.end(); ++ci)
		{
			myfile << (*ci) << " ";
		}
	}
	for (std::map<std::string, std::vector<std::string> >::const_iterator ci = near_box_location_mapping.begin(); ci != near_box_location_mapping.end(); ++ci)
	{
		const std::vector<std::string>& near_locations = (*ci).second;
		for (std::vector<std::string>::const_iterator ci = near_locations.begin(); ci != near_locations.end(); ++ci)
		{
			myfile << (*ci) << " ";
		}
	}
	myfile << " - waypoint" << std::endl;
	
	// Objects.
	for (std::map<std::string, std::string>::const_iterator ci = object_to_location_mapping.begin(); ci != object_to_location_mapping.end(); ++ci)
	{
		myfile << (*ci).first << " ";
	}
	myfile << " - object" << std::endl;
	
	// Robot
	myfile << "kenny - robot" << std::endl;
	
	// Types.
	for (std::map<std::string, std::string>::const_iterator ci = box_to_type_mapping.begin(); ci != box_to_type_mapping.end(); ++ci)
	{
		myfile << (*ci).second << " ";
	}
	myfile << " - type" << std::endl;
	
	// Boxes.
	for (std::map<std::string, std::string>::const_iterator ci = box_to_location_mapping.begin(); ci != box_to_location_mapping.end(); ++ci)
	{
		myfile << (*ci).first << " ";
	}
	myfile << " - box" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	myfile << "(:init" << std::endl;
	
	myfile << "(robot_at kenny " << robot_location_predicate << ")" << std::endl;
	myfile << "(gripper_empty kenny)" << std::endl;
	for (std::map<std::string, std::string>::const_iterator ci = object_to_location_mapping.begin(); ci != object_to_location_mapping.end(); ++ci)
	{
		myfile << "(object_at " << (*ci).first << " " << (*ci).second << ")" << std::endl;
	}
	for (std::map<std::string, std::string>::const_iterator ci = box_to_location_mapping.begin(); ci != box_to_location_mapping.end(); ++ci)
	{
		myfile << "(box_at " << (*ci).first << " " << (*ci).second << ")" << std::endl;
	}
	for (std::map<std::string, std::vector<std::string> >::const_iterator ci = near_location_mapping.begin(); ci != near_location_mapping.end(); ++ci)
	{
		const std::string& near_loc = (*ci).first;
		const std::vector<std::string>& near_locations = (*ci).second;
		for (std::vector<std::string>::const_iterator ci = near_locations.begin(); ci != near_locations.end(); ++ci)
		{
			myfile << "(near " << (*ci) << " " << near_loc << ")" << std::endl;
		}
	}
	
	for (std::map<std::string, std::vector<std::string> >::const_iterator ci = near_box_location_mapping.begin(); ci != near_box_location_mapping.end(); ++ci)
	{
		const std::string& near_loc = (*ci).first;
		const std::vector<std::string>& near_locations = (*ci).second;
		for (std::vector<std::string>::const_iterator ci = near_locations.begin(); ci != near_locations.end(); ++ci)
		{
			myfile << "(near " << (*ci) << " " << near_loc << ")" << std::endl;
		}
	}
	
	for (std::map<std::string, std::string>::const_iterator ci = box_to_type_mapping.begin(); ci != box_to_type_mapping.end(); ++ci)
	{
		myfile << "(can_fit_inside " << (*ci).second << " " << (*ci).first << ")" << std::endl;
		
		myfile << "\t(can_push kenny " << (*ci).second << ")" << std::endl;
		
		myfile << "\t(can_pickup kenny " << (*ci).second << ")" << std::endl;
	}
	
	for (std::map<std::string, std::string>::const_iterator ci = object_to_type_mapping.begin(); ci != object_to_type_mapping.end(); ++ci)
	{
		myfile << "(is_of_type " << (*ci).first << " " << (*ci).second << ")" << std::endl;
	}
	myfile << ")" << std::endl;
	myfile << "(:goal (and" << std::endl;
	for (std::map<std::string, std::string>::const_iterator ci = object_to_location_mapping.begin(); ci != object_to_location_mapping.end(); ++ci)
	{
		myfile << "(tidy " << (*ci).first << ")" << std::endl;
	}
	myfile << ")" << std::endl;
	myfile << ")" << std::endl;
	myfile << ")" << std::endl;
	myfile.close();
}

void ClassicalTidyPDDLGenerator::generateDomainFile(const std::string& file_name, const std::string& robot_location_predicate, const std::map<std::string, std::string>& object_to_location_mapping, const std::map<std::string, std::vector<std::string > >& near_location_mapping, const std::map<std::string, std::string>& object_to_type_mapping, const std::map<std::string, std::string>& box_to_location_mapping, const std::map<std::string, std::string>& box_to_type_mapping, const std::map<std::string, std::vector<std::string> >& near_box_location_mapping)
{
	std::ofstream myfile;
	myfile.open (file_name.c_str());
	myfile << "(define (domain find_key)" << std::endl;
	myfile << "(:requirements :typing :conditional-effects :negative-preconditions :disjunctive-preconditions)" << std::endl;
	myfile << std::endl;
	myfile << "(:types" << std::endl;
	myfile << "\twaypoint robot object box type" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	myfile << "(:predicates" << std::endl;
	myfile << "\t(robot_at ?r - robot ?wp - waypoint)" << std::endl;
	myfile << "\t(object_at ?o - object ?wp - waypoint)" << std::endl;
	myfile << "\t(box_at ?b - box ?wp - waypoint)" << std::endl;
	myfile << "\t(gripper_empty ?r - robot)" << std::endl;
	myfile << "\t(holding ?r - robot ?o - object)" << std::endl;
//	myfile << "\t(is_not_occupied ?wp - waypoint)" << std::endl;
	myfile << "\t(tidy ?o - object)" << std::endl;
	myfile << "\t(push_location ?o - object ?wp - waypoint)" << std::endl;
	myfile << "\t(can_pickup ?r - robot ?t - type)" << std::endl;
	myfile << "\t(can_push ?r - robot ?t - type)" << std::endl;
	myfile << "\t(can_fit_inside ?t - type ?b - box)" << std::endl;
	myfile << "\t(inside ?o - object ?b - box)" << std::endl;
	myfile << "\t(near ?wp1 ?wp2 - waypoint)" << std::endl;
	
	///myfile << "\t(connected ?from ?to - waypoint)" << std::endl;
	myfile << "\t(is_of_type ?o - object ?t -type)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**
	 * Put object in a box.
	 */
	myfile << "(:action put_object_in_box" << std::endl;
	myfile << "\t:parameters (?r - robot ?wp ?near_wp - waypoint ?o1 - object ?b - box ?t - type)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(box_at ?b ?wp)" << std::endl;
	myfile << "\t\t(robot_at ?r ?near_wp)" << std::endl;
	myfile << "\t\t(near ?near_wp ?wp)" << std::endl;
	myfile << "\t\t(holding ?r ?o1)" << std::endl;
	myfile << "\t\t(can_fit_inside ?t ?b)" << std::endl;
	myfile << "\t\t(is_of_type ?o1 ?t)" << std::endl;
	
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;

	myfile << "\t\t(and" << std::endl;
	myfile << "\t\t\t(not (holding ?r ?o1))" << std::endl;
	myfile << "\t\t\t(gripper_empty ?r)" << std::endl;
	myfile << "\t\t\t(inside ?o1 ?b)" << std::endl;
	myfile << "\t\t)" << std::endl;
	myfile << "\t)" << std::endl;
	
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**
	 * PICK-UP OBJECT.
	 */
	myfile << "(:action pickup_object" << std::endl;
	myfile << "\t:parameters (?r - robot ?wp ?near_wp - waypoint ?o - object ?t - type)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(robot_at ?r ?near_wp)" << std::endl;
	myfile << "\t\t(object_at ?o ?wp)" << std::endl;
	myfile << "\t\t(gripper_empty ?r)" << std::endl;
	myfile << "\t\t(can_pickup ?r ?t)" << std::endl;
	myfile << "\t\t(is_of_type ?o ?t)" << std::endl;
	myfile << "\t\t(near ?near_wp ?wp)" << std::endl;
	
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	
	myfile << "\t\t\t(and" << std::endl;
	myfile << "\t\t\t\t(not (gripper_empty ?r))" << std::endl;
	myfile << "\t\t\t\t(not (object_at ?o ?wp))" << std::endl;
	myfile << "\t\t\t\t(holding ?r ?o)" << std::endl;
	myfile << "\t\t\t)" << std::endl;

	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**
	 * PUT-DOWN OBJECT.
	 */
	myfile << "(:action putdown_object" << std::endl;
	myfile << "\t:parameters (?r - robot ?wp ?near_wp - waypoint ?o - object)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;

	myfile << "\t\t(robot_at ?r ?near_wp)" << std::endl;
	myfile << "\t\t(near ?near_wp ?wp)" << std::endl;
	myfile << "\t\t(holding ?r ?o)" << std::endl;
	
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	

	myfile << "\t\t\t(and" << std::endl;
	myfile << "\t\t\t\t(not (holding ?r ?o))" << std::endl;
	myfile << "\t\t\t\t(gripper_empty ?r)" << std::endl;
	myfile << "\t\t\t\t(object_at ?o ?wp)" << std::endl;
	myfile << "\t\t\t)" << std::endl;

	
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**
	 * GOTO WAYPOINT.
	 */
	myfile << "(:action goto_waypoint" << std::endl;
	myfile << "\t:parameters (?r - robot ?from ?to - waypoint)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;

	myfile << "\t\t(robot_at ?r ?from)" << std::endl;
	
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;

	myfile << "\t\t\t(and" << std::endl;
	myfile << "\t\t\t\t(not (robot_at ?r ?from))" << std::endl;
	myfile << "\t\t\t\t(robot_at ?r ?to)" << std::endl;
	myfile << "\t\t\t)" << std::endl;
	
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**
	 * PUSH OBJECT.
	 */
	myfile << "(:action push_object" << std::endl;
	myfile << "\t:parameters (?r - robot ?ob - object ?t - type ?from ?to ?near_wp - waypoint)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(robot_at ?r ?near_wp)" << std::endl;
	myfile << "\t\t(object_at ?ob ?from)" << std::endl;
	myfile << "\t\t(is_of_type ?ob ?t)" << std::endl;
	myfile << "\t\t(can_push ?r ?t)" << std::endl;
	myfile << "\t\t(near ?near_wp ?from)" << std::endl;
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	myfile << "\t\t(not (robot_at ?r ?from))" << std::endl;
	myfile << "\t\t(not (object_at ?ob ?from))" << std::endl;
	myfile << "\t\t(robot_at ?r ?to)" << std::endl;
	myfile << "\t\t(object_at ?ob ?to)" << std::endl;
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**
	 * TIDY OBJECT.
	 */
	myfile << "(:action tidy_object" << std::endl;
	myfile << "\t:parameters (?r - robot ?o - object ?b - box ?t - type)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(is_of_type ?o ?t)" << std::endl;
	myfile << "\t\t(inside ?o ?b)" << std::endl;
	myfile << "\t\t(can_fit_inside ?t ?b)" << std::endl;

	
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	
	myfile << "\t\t\t(and" << std::endl;
	myfile << "\t\t\t\t(tidy ?o)" << std::endl;
	myfile << "\t\t\t)" << std::endl;
	myfile << "\t\t)" << std::endl;
	
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	
	myfile << ")" << std::endl;
	myfile.close();
}

void ClassicalTidyPDDLGenerator::createPDDL(const std::string& path, const std::string& domain_file, const std::string& problem_file, const std::string& robot_location_predicate, const std::map<std::string, std::string>& object_to_location_mapping, const std::map<std::string, std::vector<std::string > >& near_location_mapping, const std::map<std::string, std::string>& object_to_type_mapping, const std::map<std::string, std::string>& box_to_location_mapping, const std::map<std::string, std::string>& box_to_type_mapping, const std::map<std::string, std::vector<std::string> >& near_box_location_mapping)
{
	std::stringstream ss;
	ss << path << domain_file;
	ROS_INFO("KCL: (ContingentTidyPDDLGenerator) Generate domain... %s", ss.str().c_str());
	generateDomainFile(ss.str(), robot_location_predicate, object_to_location_mapping, near_location_mapping, object_to_type_mapping, box_to_location_mapping, box_to_type_mapping, near_box_location_mapping);
	ss.str(std::string());
	ss << path  << problem_file;
	ROS_INFO("KCL: (ContingentTidyPDDLGenerator) Generate problem... %s", ss.str().c_str());
	generateProblemFile(ss.str(), robot_location_predicate, object_to_location_mapping, near_location_mapping, object_to_type_mapping, box_to_location_mapping, box_to_type_mapping, near_box_location_mapping);
}

};
