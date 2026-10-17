; The painter: dip the brush in a pot, lay the stroke of that colour, rinse.
; The objects are the pots and the water bucket of the world (entities with
; a kind); the brush and the canvas are read from the memory of the robot
; (PainterMission.cpp). The order of the layers is a fact of the scenario.
(define (domain painter)
  (:requirements :adl)

  (:types pot bucket - object)

  (:predicates
    (clean)                       ; nothing on the brush
    (loaded ?c - pot)             ; the brush carries this colour
    (painted ?c - pot)            ; the stroke of this colour is on the canvas
    (before ?a - pot ?b - pot))   ; layer a goes under layer b (fact)

  (:action dip
    :parameters (?c - pot)
    :precondition (clean)
    :effect (and (loaded ?c) (not (clean))))

  ; A layer waits for the layers under it.
  (:action mark
    :parameters (?c - pot)
    :precondition (and (loaded ?c) (not (painted ?c))
                       (forall (?p - pot) (imply (before ?p ?c) (painted ?p))))
    :effect (painted ?c))

  (:action rinse
    :parameters (?c - pot ?w - bucket)
    :precondition (loaded ?c)
    :effect (and (clean) (not (loaded ?c)))))
