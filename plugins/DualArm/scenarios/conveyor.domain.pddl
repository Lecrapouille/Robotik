; Two arms sort the parts a conveyor brings. The objects are the world
; entities with a kind (arm, part, feeder, zone, bin); the state is read on
; the shared beliefs by the observers of the mission (ConveyorMission.cpp).
;
; Nothing here says what runs in parallel: the executor finds it from what
; each step reads and changes, and from the resources of its skills (each
; arm, its cup, the conveyor).
(define (domain conveyor)
  (:requirements :strips :typing :negative-preconditions)

  (:types
    arm part - object
    place - object
    feeder zone bin - place)

  (:predicates
    (free ?r - arm)                    ; the cup of the arm holds nothing
    (holding ?r - arm ?p - part)       ; the cup of the arm holds the part
    (at ?p - part ?l - place)          ; the part rests on the place
    (zone_free ?z - zone)              ; no part on the pick zone
    (reaches ?r - arm ?z - zone)       ; static: the arm can pick there
    (near ?r - arm ?b - bin))          ; static: the arm can drop there

  ; The belt brings a part from the feeder to a free pick zone: off the
  ; infeed table, across to the lane of the zone, down to its end stop.
  (:action convey
    :parameters (?p - part ?f - feeder ?z - zone)
    :precondition (and (at ?p ?f) (zone_free ?z))
    :effect (and (at ?p ?z) (not (at ?p ?f)) (not (zone_free ?z))))

  ; Approach, reach, grasp, lift.
  (:action pick
    :parameters (?r - arm ?p - part ?z - zone)
    :precondition (and (free ?r) (at ?p ?z) (reaches ?r ?z))
    :effect (and (holding ?r ?p) (not (free ?r)) (not (at ?p ?z)) (zone_free ?z)))

  ; Carry above the bin, release, and come back over the pick zone: the
  ; arm waits where the next part arrives.
  (:action place
    :parameters (?r - arm ?p - part ?b - bin ?z - zone)
    :precondition (and (holding ?r ?p) (near ?r ?b) (reaches ?r ?z))
    :effect (and (at ?p ?b) (free ?r) (not (holding ?r ?p)))))
