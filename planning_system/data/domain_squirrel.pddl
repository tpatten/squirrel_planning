(define (domain simple_tidy)

(:requirements :strips :typing :fluents :negative-preconditions :durative-actions)

(:types
	object
	room
)

(:predicates
	(classified ?o - object)
	(untidy ?o - object)
	(tidy ?o - object)
	(explored ?r - room)
)

(:durative-action push
	:parameters (?o - object)
	:duration ( = ?duration 10)
	:effect (at end (and
		(tidy ?o)
		(not (untidy ?o))))
)

(:durative-action classify
	:parameters (?o - object)
	:duration ( = ?duration 1)
	:effect (at end (classified ?o))
)

(:durative-action explore
	:parameters (?r - room)
	:duration ( = ?duration 120)
	:effect (at end (explored ?r))
)
)
