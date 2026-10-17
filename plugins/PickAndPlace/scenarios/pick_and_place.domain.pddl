; Pick and place with one arm and a vacuum cup. The objects are the world
; entities with a kind (cube, bin); the state is read on the beliefs of the
; robot by the observers of the mission (PickPlacePlanning.cpp).
(define (domain pick_and_place)
  (:requirements :strips :typing :negative-preconditions)

  (:types
    graspable bin - object
    cube - graspable)

  (:predicates
    (seen ?o - object)                 ; observed by the camera at least once
    (holding ?o - graspable)           ; held by the vacuum cup
    (hand_empty)                       ; the cup holds nothing
    (in ?o - graspable ?b - bin))      ; resting inside the bin

  ; Look at the object: refines its position in the beliefs.
  (:action detect
    :parameters (?o - object)
    :precondition (not (seen ?o))
    :effect (seen ?o))

  ; Approach, reach, grasp, lift.
  (:action pick
    :parameters (?o - graspable)
    :precondition (and (seen ?o) (hand_empty))
    :effect (and (holding ?o) (not (hand_empty))))

  ; Carry above the bin and release. The bin does not move: its position in
  ; the world file is enough, as in the behavior tree.
  (:action place
    :parameters (?o - graspable ?b - bin)
    :precondition (holding ?o)
    :effect (and (in ?o ?b) (hand_empty) (not (holding ?o)))))
