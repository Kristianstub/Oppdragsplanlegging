; Types — what kinds of objects exist
; Predicates — what facts can be true/false about those objects
; Functions — numeric values associated with objects (like speed, distance)
; Actions — how the world changes (conditions + effects on predicates)
(define (domain robplan)
    (:requirements :typing :durative-actions :strips :fluents)

    (:types
        turtlebot - vehicle
        valve charger pump - object
        waypoint - location
        route
    )

    (:predicates
        (at ?v - vehicle ?wp - waypoint)
        (at_obj ?o - object ?wp - waypoint)
        (charged ?v - vehicle)
        (valve_manipulated ?val - valve)
        (picture_taken ?wp - waypoint)
        (connects ?r - route ?wp1 - waypoint ?wp2 - waypoint)
    )

    (:functions
        (route-length ?r - route)
        (speed ?v - vehicle)
        (duration_picture)
        (duration_manipulate)
        (duration_charge)
    )

    (:durative-action move
        :parameters (?v - turtlebot ?from - waypoint ?to - waypoint ?r - route)
        :duration (= ?duration (/ (route-length ?r) (speed ?v)))
        :condition (and
            (at start (at ?v ?from))
            (at start (connects ?r ?from ?to))
        )
        :effect (and
            (at start (not (at ?v ?from)))
            (at end (at ?v ?to))
        )
    )

    (:durative-action take_picture
        :parameters (?v - turtlebot ?wp - waypoint)
        :duration (= ?duration (duration_picture))
        :condition (and
            (over all (at ?v ?wp))
        )
        :effect (and
            (at end (picture_taken ?wp))
        )
    )

    (:durative-action manipulate_valve
        :parameters (?v - turtlebot ?wp - waypoint ?val - valve)
        :duration (= ?duration (duration_manipulate))
        :condition (and
            (over all (at ?v ?wp))
            (at start (at_obj ?val ?wp))
        )
        :effect (and
            (at end (valve_manipulated ?val))
        )
    )

    (:durative-action charge
        :parameters (?v - turtlebot ?wp - waypoint ?c - charger)
        :duration (= ?duration (duration_charge))
        :condition (and
            (over all (at ?v ?wp))
            (at start (at_obj ?c ?wp))
        )
        :effect (and
            (at end (charged ?v))
        )
    )
)