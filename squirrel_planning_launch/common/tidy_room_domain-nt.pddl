(define (domain squirrel_tidy_room)

(:requirements :strips :typing :disjunctive-preconditions :negative-preconditions)

(:types
	robot
	area
)

(:predicates
	(explored ?a - area)
	(examined ?a - area)
	(robot_in ?r - robot ?a - area)
	(connected ?from ?to - area)
	(tidy ?a - area)
	(accessible ?from ?to - area)
)

(:action move
	:parameters (?r - robot ?from ?to - area)
	:precondition (and
		(robot_in ?r ?from)
		(accessible ?from ?to)
	)
	:effect (and
		(not (robot_in ?r ?from))
		(robot_in ?r ?to)
	)
)

(:action clear_connection
	:parameters (?r - robot ?from ?to - area)
	:precondition (and
		(robot_in ?r ?from)
		(connected ?from ?to)
	)
	:effect (and
		(accessible ?from ?to)
	)
)

(:action explore_area
	:parameters (?r - robot ?a - area)
	:precondition (and
		(robot_in ?r ?a)
	)
	:effect (and
		(explored ?a)
	)
)

(:action examine_area
	:parameters (?r - robot ?a - area)
	:precondition (and
		(robot_in ?r ?a)
		(explored ?a)
	)
	:effect (and
		(examined ?a)
	)
)

(:action tidy_area
	:parameters (?r - robot ?a - area)
	:precondition (and
		(robot_in ?r ?a)
		(examined ?a)
	)
	:effect (and
		(tidy ?a)
	)
)

)

