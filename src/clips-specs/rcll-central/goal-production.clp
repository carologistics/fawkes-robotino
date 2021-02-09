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


?*PRIORITY-C0-GET-BASE-WAIT* = 5
?*PRIORITY-C0-BUFFER-CS* = 5
?*PRIORITY-C0-MOUNT-CAP-DELIVER* = 5
?*PRIORITY-C0-REFILL-SHELF* = 3
?*PRIORITY-C0-GO-WAIT* = 1
)

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
  (wm-fact (key domain fact robot-waiting args? r ?robot))
  (wm-fact (key refbox state) (value RUNNING))
  (wm-fact (key refbox phase) (value PRODUCTION|EXPLORATION))
  (wm-fact (key navgraph waitzone generated) (type BOOL) (value TRUE))
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  ; (NavGraphGeneratorInterface (final TRUE))
  ; (not (wm-fact (key domain fact entered-field args? r ?robot)))
  =>
  (printout t "Goal " ENTER-FIELD " formulated for " ?robot crlf)
  (goal-tree-assert-retry ENTER-FIELD-LOOP 999 (assert (goal (id (sym-cat ENTER-FIELD- (gensym*))) ;Robin Küpper fix retry
                (class ENTER-FIELD) (sub-type SIMPLE)
                (params r ?robot team-color ?team-color))))
)


(defrule goal-production-create-produce-c0
" Produce a C0 product: Get the correct base and mount the right cap on it.
  The produced workpiece stays in the output of the used cap station after
  successfully executing this goal.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (wm-fact (key domain fact order-complexity args? ord ?order com C0))
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))

 (wm-fact (key refbox team-color) (value ?team-color))


 (wm-fact (key domain fact mps-type args? m ?cs t CS))
 (wm-fact (key domain fact mps-team args? m ?cs col ?team-color))

 (wm-fact (key domain fact mps-type args? m ?bs t BS))
 (wm-fact (key domain fact mps-team args? m ?bs col ?team-color))

 (wm-fact (key domain fact mps-type args? m ?ds t DS))
 (wm-fact (key domain fact mps-team args? m ?ds col ?team-color))
  (not(wm-fact (key domain fact order-fulfilled args? ord ?order)))
  (not (goal (class PRODUCE-C0)))
  =>
  (printout t "Goal for C0 order " ?order " formulated: " ?base-color " " ?cap-color   crlf)
  (bind ?wp (sym-cat WP- (random-id)))
  (assert (goal (id (sym-cat PRODUCE-C0- (gensym*)))
                (class PRODUCE-C0)(sub-type RUN-SUBGOALS-IN-PARALLEL)
                (params order ?order bs-color ?base-color cs-color ?cap-color wp ?wp bs ?bs cs ?cs ds ?ds)
  ))
)

(defrule goal-produce-c0-get-base-wait
  "get a base for c0-production and wait at the cap-station"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c0-id) (class PRODUCE-C0) (mode SELECTED))
  (not (goal (class GET-BASE-WAIT) (parent ?produce-c0-id)))
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))

  =>
  
  (printout t "Goal GET-BASE-WAIT formulated" ?*PRIORITY-C0-BUFFER-CS* crlf)
  (assert(goal (id (sym-cat GET-BASE-WAIT- (gensym*))) (class GET-BASE-WAIT) (parent ?produce-c0-id) (sub-type SIMPLE)(priority ?*PRIORITY-C0-GET-BASE-WAIT*)(mode FORMULATED)))
)

(defrule goal-produce-c0-buffer-cs
  "get a base for c0-production and wait at the cap-station"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c0-id) (class PRODUCE-C0) (mode SELECTED))
  (not (goal (class BUFFER-CS) (parent ?produce-c0-id)))
  =>

  ;(bind ?*PRIORITY-C0-MOUNT-CAP-DELIVER* 1)
  (printout t "Goal BUFFER-CS formulated" crlf)
  (assert (goal (id (sym-cat BUFFER-CS- (gensym*))) (class BUFFER-CS) (parent ?produce-c0-id) (sub-type SIMPLE) (priority ?*PRIORITY-C0-BUFFER-CS*)(mode FORMULATED)))
)

(defrule goal-produce-c0-mount-cap-deliver
  "get a base for c0-production and wait at the cap-station"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c0-id) (class PRODUCE-C0) (mode SELECTED))
  (not (goal (class MOUNT-CAP-DELIVER) (parent ?produce-c0-id)))
  =>
  (printout t "Goal MOUNT-CAP-DELIVER formulated" crlf)
  (assert (goal (id (sym-cat MOUNT-CAP-DELIVER- (gensym*))) (class MOUNT-CAP-DELIVER) (parent ?produce-c0-id) (sub-type SIMPLE)(priority ?*PRIORITY-C0-MOUNT-CAP-DELIVER*)(mode FORMULATED)))
)

(defrule goal-produce-c0-create-refill-shelf
  "Refill a shelf whenever it is empty."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (not (wm-fact (key domain fact wp-on-shelf args? wp ?wp m ?mps spot ?spot)))
  (goal (id ?produce-c0-id) (class PRODUCE-C0) (mode SELECTED))
  (not (goal (class REFILL-SHELF) (parent ?produce-c0-id)))
  =>
  (printout t "Goal REFILL-SHELF formulated" crlf)
  (assert (goal (id (sym-cat REFILL-SHELF- (gensym*))) (class REFILL-SHELF) (parent ?produce-c0-id) (sub-type SIMPLE)(priority ?*PRIORITY-C0-REFILL-SHELF*)(mode FORMULATED)(params m ?mps )))
)


(defrule goal-production-create-go-wait
  "Drive to a waiting position and wait there."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (domain-object (type waitpoint) (name ?waitpoint&:
               (eq (str-length (str-cat ?waitpoint)) 10)))
  (not(goal(class GO-WAIT)))
  =>
  (printout t "Goal " GO-WAIT " formulated" ?waitpoint crlf)
  (assert (goal (id (sym-cat GO-WAIT- (gensym*)))
                (class GO-WAIT) (sub-type SIMPLE)(priority ?*PRIORITY-C0-GO-WAIT*)
                (params waitpoint ?waitpoint)
  ))
)