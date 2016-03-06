(define (domain squirrel_blocks)

(:requirements :strips :typing :fluents :disjunctive-preconditions :negative-preconditions :durative-actions)

(:types
	waypoint
	robot
	object
	box
)

(:predicates
	(explored ?wp - waypoint)
	(robot_at ?v - robot ?wp - waypoint)
	(object_at ?o - object ?wp - waypoint)
	(box_at ?b - box ?wp - waypoint)
	(connected ?from ?to - waypoint)

	(gripper_empty ?r - robot)
	(holding ?r - robot ?o - object)

	(on ?o1 ?o2 - object)
	(clear ?o - object)

	(tidy ?o - object)
	(tidy_location_unknown ?o - object)
	(push_location ?o - object ?wp -waypoint)
	(tidy_location ?o - object ?wp -waypoint)

	(inside ?o - object ?b - box)

	(can_pickup ?v - robot ?o - object)
	(can_push ?v - robot ?o - object)

	(can_fit_inside ?o - object ?b - box)
	(can_stack_on ?o1 ?o2 - object)
)


(:functions
	(distance ?wp1 ?wp2 - waypoint) 
)


;; Put object in a box
(:durative-action stack_object
	:parameters (?v - robot ?wp - waypoint ?b - box ?o - object)
	:duration ( = ?duration 10)
	:condition (and
		(over all (robot_at ?v ?wp))
		(over all (box_at ?b ?wp))
		(at start (holding ?v ?o))
		(at start (can_fit_inside ?o ?b)))
	:effect (and
		(at start (not (holding ?v ?o2)))
		(at end (gripper_empty ?v))
		(at end (inside ?o ?b)))
)


;; Unstack object
(:durative-action unstack_object
	:parameters (?v - robot ?wp - waypoint ?o1 ?o2 - object)
	:duration ( = ?duration 10)
	:condition (and
		(over all (robot_at ?v ?wp))
		(at start (object_at ?o1 ?wp))
		(at start (on ?o2 ?o1))
		(at start (gripper_empty ?v))
		(at start (can_pickup ?o2)))
	:effect (and
		(at start (not (gripper_empty ?v)))
		(at start (not (on ?o2 ?o1)))
		(at start (clear ?o1))
		(at end (holding ?v ?o2)))
)


;; Stack object
(:durative-action stack_object
	:parameters (?v - robot ?wp - waypoint ?o1 ?o2 - object)
	:duration ( = ?duration 10)
	:condition (and
		(over all (robot_at ?v ?wp))
		(over all (object_at ?o1 ?wp))
		(at start (clear ?o1))
		(at start (holding ?v ?o2))
		(at start (can_stack_on ?o2 ?o1)))
	:effect (and
		(at start (not (clear ?o1)))
		(at start (not (holding ?v ?o2)))
		(at end (gripper_empty ?v))
		(at end (on ?o2 ?o1)))
)


;; Pick-up object
(:durative-action pickup_object
	:parameters (?v - robot ?wp - waypoint ?o - object)
	:duration ( = ?duration 10)
	:condition (and
		(over all (robot_at ?v ?wp))
		(at start (object_at ?o ?wp))
		(at start (gripper_empty ?v))
		(at start (can_pickup ?o)))
	:effect (and
		(at start (not (gripper_empty ?v)))
		(at start (not (object-at ?o ?wp)))
		(at end (holding ?v ?o)))
)


;; Put-down object
(:durative-action putdown_object
	:parameters (?v - robot ?wp - waypoint ?o - object)
	:duration ( = ?duration 10)
	:condition (and
		(over all (robot_at ?v ?wp))
		(at start (holding ?v ?o)))
	:effect (and
		(at start (not (holding ?v ?o)))
		(at end (gripper_empty ?v))
		(at end (object_at ?o ?wp)))
)


;; Use perception actions to search for objects at the current waypoint
(:durative-action explore_waypoint
	:parameters (?v - robot ?wp - waypoint)
	:duration ( = ?duration 10)
	:condition (and
		(over all (robot_at ?v ?wp)))
	:effect (and
		(at end (explored ?wp)))
)


;; Move between any two waypoints, avoiding terrain
(:durative-action goto_waypoint
	:parameters (?v - robot ?from ?to - waypoint)
	:duration ( = ?duration 10);;(* (distance ?from ?to) 10))
	:condition (and
		(at start (robot_at ?v ?from)))
	:effect (and
		(at start (not (robot_at ?v ?from)))
		(at end (robot_at ?v ?to)))
)


;; Push an object between two waypoints in an unobstructed straight line
(:durative-action push_object
	:parameters (?v - robot ?ob - object ?from ?to ?obw - waypoint)
	:duration ( = ?duration 80)
	:condition (and
		(at start (robot_at ?v ?from))
		(at start (object_at ?ob ?obw))
		(at start (push_location ?ob ?from)))
	:effect (and
		(at start (not (robot_at ?v ?from)))
		(at start (not (object_at ?ob ?obw)))
		(at end (robot_at ?v ?to))
		(at end (object_at ?ob ?to)))
)


;; Tidy a classified object at its goal location
(:durative-action tidy_object
	:parameters (?v - robot ?o - object ?wp - waypoint)
	:duration ( = ?duration 5)
	:condition (and
		(at start (robot_at ?v ?wp))
		(at start (object_at ?o ?wp))
		(at start (tidy_location ?o ?wp)))
	:effect (and
		(at end (tidy ?o)))
)
)
