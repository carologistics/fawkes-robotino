;---------------------------------------------------------------------------
;  goal-maintain-production.clp - Generate production goals of RCLL
;
;  Created: Tue 09 Jan 2018 17:03:31 CET
;  Copyright  2018  Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file in the doc directory.
;---------------------------------------------------------------------------

; This program is free software; you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation; either version 2 of the License, or
; (at your option) any later version.
;
; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU Library General Public License for more details.
;
; Read the full text in the LICENSE.GPL file in the doc directory.
;

(defglobal
  ; production order priorities
  ?*PRIORITY-ENTER-FIELD* = 200
  ?*PRIORITY-FIND-MISSING-MPS* = 110
  ?*PRIORITY-DELIVER* = 100
  ?*PRIORITY-RESET-MPS* = 98
  ?*PRIORITY-CLEAR-BS* = 97
  ?*PRIORITY-PRODUCE-CX* = 95
  ?*PRIORITY-PRODUCE-C0* = 90
  ?*PRIORITY-ADD-ADDITIONAL-RING* = 85
  ?*PRIORITY-ADD-FIRST-RING* = 80
  ?*PRIORITY-CLEAR-CS* = 70
  ?*PRIORITY-CLEAR-RS* = 55
  ?*PRIORITY-PREFILL-CS* = 50
  ?*PRIORITY-PREFILL-RS-WITH-HOLDING-BASE* = 45
  ?*PRIORITY-PREFILL-RS* = 40
  ?*PRIORITY-ADD-ADDITIONAL-RING-WAITING* = 20
  ?*PRIORITY-DISCARD-UNKNOWN* = 10
  ?*PRIORITY-NOTHING-TO-DO* = -1
  ;ToDo:The proirites are copied from old agent
  ;     for the current moment. Filter out uneeded
  ;     later. For now needed for refrence.

)

; ## Maintain beacon sending
(defrule goal-reasoner-create-beacon-maintain
  (not (goal (id BEACONMAINTAIN)))
  =>
  (assert (goal (id BEACONMAINTAIN) (type MAINTAIN)))
)

(defrule goal-reasoner-create-beacon-achieve
  ?g <- (goal (id BEACONMAINTAIN) (mode SELECTED))
  (not (goal (id BEACONACHIEVE)))
  (time $?now)
  ; TODO: make interval a constant
  (goal-meta (goal-id BEACONMAINTAIN)
    (last-achieve $?last&:(timeout ?now ?last 1)))
  =>
  (assert (goal (id BEACONACHIEVE) (parent BEACONMAINTAIN)))
)


; ## Maintain wp-spawning
(defrule goal-reasoner-create-wp-spawn-maintain
 (domain-facts-loaded)
 (not (goal (id WPSPAWN-MAINTAIN)))
 =>
 (assert (goal (id WPSPAWN-MAINTAIN) (type MAINTAIN)))
)

(defrule goal-reasoner-create-wp-spawn-achieve
  ?g <- (goal (id WPSPAWN-MAINTAIN) (mode SELECTED))
  (not (goal (id WPSPAWN-ACHIEVE)))
  (time $?now)
  ; TODO: make interval a constant
  (goal-meta (goal-id WPSPAWN-MAINTAIN)
  (last-achieve $?last&:(timeout ?now ?last 1)))
  (wm-fact (key domain fact self args? r ?robot))
  (not (and
    (domain-object (name ?wp) (type workpiece))
    (wm-fact (key domain fact wp-spawned-by args? wp ?wp r ?robot))))
  =>
  (assert (goal (id WPSPAWN-ACHIEVE) (parent WPSPAWN-MAINTAIN)
                                     (params robot ?robot)))
)


; ## Maintain production
(defrule goal-reasoer-create-goal-production-maintain
  "The parent production goal. Allowes formulation of
  production goals only if proper game state selected
  and domain loaded. Other production goals are
  formulated as sub-goals of this goal"
  (domain-facts-loaded)
  (not (goal (id PRODUCTION-MAINTAIN)))
  (wm-fact (key refbox state) (type UNKNOWN) (value RUNNING))
  (wm-fact (key refbox phase) (type UNKNOWN) (value PRODUCTION))
  =>
  (assert (goal (id PRODUCTION-MAINTAIN) (type MAINTAIN)))
)

(defrule goal-reasoner-create-enter-field
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id PRODUCTION-MAINTAIN) (mode SELECTED))
  ; (not (goal (id ENTER-FIELD)))
  (not (goal-already-tried ENTER-FIELD))
  (wm-fact (key domain fact self args? r ?robot))
  (wm-fact (key domain fact robot-waiting args? r ?robot))
  (not (wm-fact (key domain fact entered-field args? r ?robot)))
  =>
  (printout t "Goal " ENTER-FIELD " formulated" crlf)
  (assert (goal (id ENTER-FIELD) (priority ?*PRIORITY-ENTER-FIELD*)
                                 (parent PRODUCTION-MAINTAIN)))
  ; This is just to make sure we formulate the goal only once.
  ; In an actual domain this would be more sophisticated.
  (assert (goal-already-tried ENTER-FIELD))
)

(deffunction random-id ()
  "Return a random task id"
  (return (random 0 1000000000))
)

(defrule goal-reasoner-evaluate-completed-subgoal-wp-spawn
  ?g <- (goal (id WPSPAWN-ACHIEVE) (parent WPSPAWN-MAINTAIN)
            (mode FINISHED) (outcome COMPLETED)
            (params robot ?robot))
  ?p <- (goal (id WPSPAWN-MAINTAIN))
  ?m <- (goal-meta (goal-id WPSPAWN-MAINTAIN))
  (time $?now)
  (wm-fact (key domain fact self args? r ?robot))
  =>
  (printout debug "Goal '" WPSPAWN-ACHIEVE "' (part of '" WPSPAWN-MAINTAIN
    "') has been completed, Evaluating" crlf)
     (bind ?wp-id (sym-cat WP (random-id)))
  (assert
    (domain-object (name ?wp-id) (type workpiece))
    (wm-fact (key domain fact wp-unused args? wp ?wp-id) (value TRUE))
    (wm-fact (key domain fact wp-cap-color args? wp ?wp-id col CAP_NONE) (value TRUE))
    (wm-fact (key domain fact wp-ring1-color args? wp ?wp-id col RING_NONE) (value TRUE))
    (wm-fact (key domain fact wp-ring2-color args? wp ?wp-id col RING_NONE) (value TRUE))
    (wm-fact (key domain fact wp-ring3-color args? wp ?wp-id col RING_NONE) (value TRUE))
    (wm-fact (key domain fact wp-base-color args? wp ?wp-id col BASE_NONE) (value TRUE))
    (wm-fact (key domain fact wp-spawned-by args? wp ?wp-id r ?robot) (value TRUE))
  )
  (modify ?g (mode EVALUATED))
  (modify ?m (last-achieve ?now))
)