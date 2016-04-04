(define (domain squirrel_explore)

(:requirements :strips :typing  :disjunctive-preconditions :negative-preconditions)

(:types
	waypoint
	robot
)

(:predicates
	(explored ?wp - waypoint)
	(robot_at ?r - robot ?wp - waypoint)
)

;; Use perception actions to search for objects at the current waypoint
(:action explore_waypoint
	:parameters (?r - robot ?wp - waypoint)
	:precondition (and
		(robot_at ?r ?wp)
	)
	:effect (and
		(explored ?wp)
	)
)

;; Move between any two waypoints, avoiding terrain
(:action goto_waypoint
	:parameters (?r - robot ?from ?to - waypoint)
	:precondition (and
		(robot_at ?r ?from))
	:effect (and
		(not (robot_at ?r ?from))
		(robot_at ?r ?to)
	)
)
)

