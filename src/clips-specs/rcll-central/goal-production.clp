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
(defrule goal-production-create-beacon-maintain
" The parent goal for beacon signals. Allows formulation of
  goals that periodically communicate with the refbox.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (not (goal (class BEACON-MAINTAIN)))
  (or (domain-facts-loaded)
      (wm-fact (key refbox phase) (value ~SETUP&~PRE_GAME)))
  =>
  (bind ?goal (goal-tree-assert-run-endless BEACON-MAINTAIN 1))
  (modify ?goal (verbosity QUIET) (params frequency 1))
)


(defrule goal-production-create-beacon-achieve
" Send a beacon signal whenever at least one second has elapsed since it
  last one got sent.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (time $?now)
  ?g <- (goal (id ?maintain-id) (class BEACON-MAINTAIN) (mode SELECTED))
  ; TODO: make interval a constant
  =>
  (assert (goal (id (sym-cat SEND-BEACON- (gensym*))) (sub-type SIMPLE)
                (class SEND-BEACON) (parent ?maintain-id) (verbosity QUIET)))
)

(defrule goal-production-create-refill-shelf-maintain
" The parent goal to refill a shelf. Allows formulation of goals to refill
  a shelf only if the game is in the production phase and the domain is loaded.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (domain-facts-loaded)
  (not (goal (class REFILL-SHELF-MAINTAIN)))
  (not (mutex (name ?n&:(eq ?n (resource-to-mutex refill-shelf))) (state LOCKED)))
  (wm-fact (key refbox phase) (value PRODUCTION))
  =>
  (bind ?goal (goal-tree-assert-run-endless REFILL-SHELF-MAINTAIN 1))
  (modify ?goal (required-resources refill-shelf)
                (params frequency 1 retract-on-REJECTED)
                (verbosity QUIET))
)


(defrule goal-production-create-refill-shelf-achieve
  "Refill a shelf whenever it is empty."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?g <- (goal (id ?maintain-id) (class REFILL-SHELF-MAINTAIN) (mode SELECTED))
  (not (goal (class REFILL-SHELF)))
  (wm-fact (key refbox phase) (value PRODUCTION))
  (wm-fact (key game state) (value RUNNING))
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (not (wm-fact (key domain fact wp-on-shelf args? wp ?wp m ?mps spot ?spot)))
  =>
  (assert (goal (id (sym-cat REFILL-SHELF- (gensym*)))
                (class REFILL-SHELF) (sub-type SIMPLE)
                (parent ?maintain-id) (verbosity QUIET)
                (params mps ?mps)))
)


(defrule goal-production-navgraph-compute-wait-positions-finished
  "Add the waiting points to the domain once their generation is finished."
  (NavGraphWithMPSGeneratorInterface (final TRUE))
  (not (NavGraphWithMPSGeneratorInterface (final ~TRUE)))
=>
  (printout t "Navgraph generation of waiting-points finished. Getting waitpoints." crlf)
  (do-for-all-facts ((?waitzone navgraph-node)) (str-index "WAIT-" ?waitzone:name)
    (assert
      (domain-object (name (sym-cat ?waitzone:name)) (type waitpoint))
      (wm-fact (key navgraph waitzone args? name (sym-cat ?waitzone:name)) (is-list TRUE) (type INT) (values (nth$ 1 ?waitzone:pos) (nth$ 2 ?waitzone:pos)))
    )
  )
  (assert (wm-fact (key navgraph waitzone generated) (type BOOL) (value TRUE)))
)

(defrule goal-production-create-enter-field
  "Enter the field (drive outside of the starting box)."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (not (goal (class ENTER-FIELD)))
  (wm-fact (key central agent robot args? r ?robot))
  (wm-fact (key domain fact robot-waiting args? r ?robot))
  (wm-fact (key refbox state) (value RUNNING))
  (wm-fact (key refbox phase) (value PRODUCTION|EXPLORATION))
  (wm-fact (key navgraph waitzone generated) (type BOOL) (value TRUE))
  (wm-fact (key refbox team-color) (value ?team-color))
  (NavGraphGeneratorInterface (final TRUE))
  ;Check if robot has entered the field already
  (not (wm-fact (key domain fact entered-field args? r ?robot)))
  =>
  (printout t "Goal " ENTER-FIELD " formulated" crlf)
  (assert (goal (id (sym-cat ENTER-FIELD- (gensym*)))
                (class ENTER-FIELD) (sub-type SIMPLE)
                (params r ?robot team-color ?team-color)))
)

;Diasbled due to the tendency of robot 2 and 3 failing to enter
;(defrule goal-production-hack-failed-enter-field
;  "HACK: Stop trying to enter the field when it failed a few times."
  ; TODO-GM: this was after 3 tries, now its instantly
;  ?g <- (goal (id ?gid) (class ENTER-FIELD)
;               (mode FINISHED) (outcome FAILED))
;  ?pa <- (plan-action (goal-id ?gid) (state FAILED) (action-name enter-field))
;  =>
;  (printout t "Goal '" ?gid "' has failed, evaluating" crlf)
;  (modify ?pa (state EXECUTION-SUCCEEDED))
;  (modify ?g (mode DISPATCHED) (outcome UNKNOWN))
;)



(defrule goal-production-visit-first-station
  "Formulate a goal for a robot to visit his first station, taking first matched station
   creates semi-random spread of robots over the field"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ; Get a robot that hasn't visited a station yet but has entered the field
  (wm-fact (key central agent robot args? r ?robot))
  (not (started ?robot))
  ;(goal (class ENTER-FIELD) (mode FINISHED) (params r ?robot team-color ?team-color))
  (wm-fact (key domain fact entered-field args? r ?robot))
  ; Get an unvisited station and a side
  (domain-object (type mps) (name ?station))
  (domain-object (type mps-side) (name ?side))
  (test (or (eq ?side INPUT) (eq ?side OUTPUT)))
  (not (visited ?station))
  (not (goal (class VISIT-STATION) (params r ? station ?station side ?)))
  =>
  (printout t "Robot " ?robot " has first station: " ?station crlf )
  (printout t "Goal " VISIT-STATION " formulated for " ?station " and " ?robot crlf)
  (assert (goal (id (sym-cat VISIT- ?station -WITH- ?robot))
                (class VISIT-STATION) (type ACHIEVE) (sub-type SIMPLE) 
                (params r ?robot station ?station side ?side)))
  (assert (started ?robot))
)



(defrule goal-production-visit-one-station
  "Formulate goals to send robots to unvisited stations until 
   all stations have been visited"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ; Get a robot that has already visited at least one station and
  ; is not currently assigned to a goal
  (wm-fact (key central agent robot args? r ?robot))
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (goal (class VISIT-STATION) (params r ?robot station ? side ?)))
  ; Get an unvisited station and a side
  (domain-object (type mps) (name ?station))
  (domain-object (type mps-side) (name ?side))
  (test (or (eq ?side INPUT) (eq ?side OUTPUT)))
  (not (visited ?station))
  (not (goal (class VISIT-STATION) (params r ? station ?station side ?)))
  ; Get current position (station) of robot (side unrelated to station side!)
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?))
  ; Get positions of current and next station candidate on the field
  (navgraph-node
    (name ?curr-station&:(eq ?curr-station ?curr-location))
    (pos $?curr-pos))
  (navgraph-node
    (name ?next-station&:(eq ?next-station
                     (str-cat ?station (if (eq ?side INPUT) then -I else -O))))
    (pos $?next-pos))
  ; Check that no non-visited station alternative with lower distance 
  ; to current station exists
  (not (and
    (domain-object (type mps) (name ?station-alt))
    (domain-object (type mps-side) (name ?side-alt))
    (test (or (eq ?side-alt INPUT) (eq ?side-alt OUTPUT)))
    (not (visited ?station-alt))
    (not (goal (class VISIT-STATION) (params r ? station ?station-alt side ?)))
    (navgraph-node
      (name ?node-alt&:(eq ?node-alt
                       (str-cat ?station-alt (if (eq ?side-alt INPUT) then -I else -O))))
      (pos $?next-pos-alt))
    (test
      (< (distance-mf ?curr-pos ?next-pos-alt)
         (distance-mf ?curr-pos ?next-pos))
    )
  ))
  =>
  (printout t "Goal " VISIT-STATION " formulated for station " ?station " and robot " ?robot crlf)
  (printout t "Previous was " ?curr-location crlf)
  (printout t "Distance is " (distance-mf ?curr-pos ?next-pos) crlf)
  (assert (goal (id (sym-cat VISIT- ?station -WITH- ?robot))
                (class VISIT-STATION) (type ACHIEVE) (sub-type SIMPLE) 
                (params r ?robot station ?station side ?side)))
)
