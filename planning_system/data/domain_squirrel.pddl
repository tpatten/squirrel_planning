(define (domain simple_tidy)

(:requirements :strips :typing :fluents :negative-preconditions :durative-actions)

(:types
	object
	toy - object
	trash - object
	room
)

(:predicates
	(tidy ?t - toy)
	(categorised ?o - object)
	(explored ?r - room)
)

(:durative-action push
	:parameters (?t - toy)
	:duration ( = ?duration 10)
	:effect (at end (tidy ?t))
)

(:durative-action classify
	:parameters (?o - object)
	:duration ( = ?duration 1)
	:effect (at end (categorised ?o))
)

(:durative-action explore
	:parameters (?r - room)
	:duration ( = ?duration 120)
	:effect (at end (explored ?r))
)
)
