(define (domain squirrel_explore)

(:requirements :strips :typing  :disjunctive-preconditions :negative-preconditions)

(:types
	waypoint
	robot
)

(:predicates
	(explored ?wp - waypoint)
	(robot_at ?v - robot ?wp - waypoint)
)

;; Use perception actions to search for objects at the current waypoint
(:action explore_waypoint
	:parameters (?v - robot ?wp - waypoint)
	:precondition (and
		(robot_at ?v ?wp)
	)
	:effect (and
		(explored ?wp)
	)
)

;; Move between any two waypoints, avoiding terrain
(:action goto_waypoint
	:parameters (?v - robot ?from ?to - waypoint)
	:precondition (and
		(robot_at ?v ?from))
	:effect (and
		(not (robot_at ?v ?from))
		(robot_at ?v ?to)
	)
)
)

