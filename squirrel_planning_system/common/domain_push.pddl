(define (domain squirrel_blocks)

(:requirements :strips :typing :fluents :disjunctive-preconditions :negative-preconditions :durative-actions)

(:types
	waypoint 
	robot
	object
)

(:predicates
	(explored ?wp - waypoint)
	(robot_at ?v - robot ?wp - waypoint)
	(object_at ?o - object ?wp - waypoint)
	(connected ?from ?to - waypoint)
	(classified ?o - object)
	(tidy ?o - object)
)

(:functions
	(distance ?wp1 ?wp2 - waypoint) 
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

;; Use perception actions to classify an object
(:durative-action classify_object
	:parameters (?v - robot ?wp - waypoint ?o - object)
	:duration ( = ?duration 10)
	:condition (and
		(over all (robot_at ?v ?wp))
		(over all (object_at ?o ?wp)))
	:effect (and
		(at end (classified ?o)))
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
	:parameters (?v - robot ?ob - object ?from ?to - waypoint)
	:duration ( = ?duration 10);;(* (distance ?from ?to) 120))
	:condition (and
		(at start (robot_at ?v ?from))
		(at start (object_at ?ob ?from))
		(at start (connected ?from ?to)))
	:effect (and
		(at start (not (robot_at ?v ?from)))
		(at start (not (object_at ?ob ?from)))
		(at end (robot_at ?v ?to))
		(at end (object_at ?ob ?to)))
)

;; Tidy a classified object -- an abstract action in place of push_object
(:durative-action make_tidy
	:parameters (?v - robot ?o - object ?wp - waypoint)
	:duration ( = ?duration 10)
	:condition (and
		(at start (robot_at ?v ?wp))
		(at start (object_at ?o ?wp))
		(at start (classified ?o)))
	:effect (and
		(at end (tidy ?o)))
)
)
