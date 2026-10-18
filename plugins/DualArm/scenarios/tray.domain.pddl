; One arm carries a tray of glasses; the other arm visits each glass while
; the tray moves. A temporal domain: visit needs the tray in motion over all
; its duration, so the plan holds the tray during the visits (required
; concurrency). The state is read on the shared beliefs by the observers of
; the mission (TrayMission.cpp). Durations are estimates in seconds: the
; executor waits for the skills, not for the clock.
(define (domain tray)
  (:requirements :strips :typing :durative-actions :negative-preconditions)

  (:types arm tray glass pad)

  (:predicates
    (free ?r - arm)                    ; the arm holds nothing
    (holding ?r - arm ?t - tray)       ; the cup of the arm holds the tray
    (rests ?t - tray ?p - pad)         ; the tray lies on the stand
    (above ?t - tray ?p - pad)         ; the held tray is over the stand
    (in_motion ?t - tray)              ; a plan token: true during a carry
    (on ?g - glass ?t - tray)          ; the glass stands on the tray
    (visited ?g - glass)               ; a tool passed above the glass
    (can_carry ?r - arm)               ; static: the arm has a cup
    (can_visit ?r - arm))              ; static: the arm has a pointer

  ; Take the tray and raise it.
  (:durative-action lift
    :parameters (?r - arm ?t - tray ?p - pad)
    :duration (= ?duration 6)
    :condition (and (at start (free ?r)) (at start (can_carry ?r)) (at start (rests ?t ?p)))
    :effect (and (at start (not (free ?r)))
                 (at end (holding ?r ?t)) (at end (not (rests ?t ?p))) (at end (above ?t ?p))))

  ; Carry the raised tray over another stand.
  (:durative-action carry
    :parameters (?r - arm ?t - tray ?from - pad ?to - pad)
    :duration (= ?duration 12)
    :condition (and (at start (holding ?r ?t)) (at start (above ?t ?from)) (over all (holding ?r ?t)))
    :effect (and (at start (in_motion ?t))
                 (at end (not (in_motion ?t))) (at end (not (above ?t ?from))) (at end (above ?t ?to))))

  ; Bring the tool above a glass of the moving tray, and keep it there.
  (:durative-action visit
    :parameters (?r - arm ?g - glass ?t - tray)
    :duration (= ?duration 2)
    :condition (and (at start (free ?r)) (at start (can_visit ?r))
                    (over all (in_motion ?t)) (over all (on ?g ?t)))
    :effect (and (at start (not (free ?r))) (at end (free ?r)) (at end (visited ?g))))

  ; Lower the tray on the stand under it and let it go.
  (:durative-action set_down
    :parameters (?r - arm ?t - tray ?p - pad)
    :duration (= ?duration 5)
    :condition (and (at start (holding ?r ?t)) (at start (above ?t ?p)))
    :effect (and (at end (rests ?t ?p)) (at end (free ?r)) (at end (not (holding ?r ?t)))
                 (at end (not (above ?t ?p))))))
