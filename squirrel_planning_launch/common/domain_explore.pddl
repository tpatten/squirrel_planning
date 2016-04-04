(define (domain squirrel_explore)

(:requirements :strips :typing :fluents :disjunctive-preconditions :negative-preconditions :durative-actions)

(:types
	waypoint
	robot
)

(:predicates
	(explored ?wp - waypoint)
	(robot_at ?r - robot ?wp - waypoint)
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
)
