; One arm carries a tray of glasses, holding it from below; the other arm
; drills a path on the tray while it moves. A temporal domain: drill needs
; the tray in motion over all its duration, so the plan holds the tray
; during the drilling (required concurrency). The state is read on the
; shared beliefs by the observers of the mission (TrayMission.cpp).
; Durations are estimates in seconds: the executor waits for the skills,
; not for the clock.
(define (domain tray)
  (:requirements :strips :typing :durative-actions :negative-preconditions)

  (:types arm tray pad)

  (:predicates
    (free ?r - arm)                    ; the arm holds nothing
    (holding ?r - arm ?t - tray)       ; the cup of the arm holds the tray
    (rests ?t - tray ?p - pad)         ; the tray lies on the stand of the pad
    (above ?t - tray ?p - pad)         ; the held tray is over the pad
    (in_motion ?t - tray)              ; a plan token: true during a carry
    (drilled ?t - tray)                ; the drill followed its path on the tray
    (can_carry ?r - arm)               ; static: the arm has a cup
    (can_drill ?r - arm))              ; static: the arm has a drill

  ; Come under the tray, take it by its bottom face and raise it.
  (:durative-action lift
    :parameters (?r - arm ?t - tray ?p - pad)
    :duration (= ?duration 6)
    :condition (and (at start (free ?r)) (at start (can_carry ?r)) (at start (rests ?t ?p)))
    :effect (and (at start (not (free ?r)))
                 (at end (holding ?r ?t)) (at end (not (rests ?t ?p))) (at end (above ?t ?p))))

  ; Carry the raised tray over another stand.
  (:durative-action carry
    :parameters (?r - arm ?t - tray ?from - pad ?to - pad)
    :duration (= ?duration 22)
    :condition (and (at start (holding ?r ?t)) (at start (above ?t ?from)) (over all (holding ?r ?t)))
    :effect (and (at start (in_motion ?t))
                 (at end (not (in_motion ?t))) (at end (not (above ?t ?from))) (at end (above ?t ?to))))

  ; Touch the moving tray with the drill and follow the path on it.
  (:durative-action drill
    :parameters (?r - arm ?t - tray)
    :duration (= ?duration 19)
    :condition (and (at start (free ?r)) (at start (can_drill ?r)) (over all (in_motion ?t)))
    :effect (and (at start (not (free ?r))) (at end (free ?r)) (at end (drilled ?t))))

  ; Lower the tray on the stand under it and let it go.
  (:durative-action set_down
    :parameters (?r - arm ?t - tray ?p - pad)
    :duration (= ?duration 5)
    :condition (and (at start (holding ?r ?t)) (at start (above ?t ?p)))
    :effect (and (at end (rests ?t ?p)) (at end (free ?r)) (at end (not (holding ?r ?t)))
                 (at end (not (above ?t ?p))))))
