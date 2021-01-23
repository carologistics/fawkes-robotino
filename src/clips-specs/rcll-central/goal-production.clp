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

; =================== administrative goals (no robot action) ========================

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
  ;(not (mutex (name ?n&:(eq ?n (resource-to-mutex refill-shelf))) (state LOCKED)))
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

(defrule goal-production-create-handle-mps
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?pre <- (wm-fact (key mps-handling prepare ?prepare-action ?mps args? $?prepare-params))
  ?pro <- (wm-fact (key mps-handling process ?process-action ?mps args? $?process-params))
  (not (goal (class HANDLE-MPS) (params ?mps)))
  =>
  (assert (goal (id (sym-cat HANDLE-MPS-(gensym*))) (class HANDLE-MPS)
                (params ?mps) (sub-type SIMPLE)))
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

(deffunction create-wp (?order)
  (bind ?wp (sym-cat "wp"-(gensym*)))
  (assert (wm-fact (key domain fact wp-unused args? wp ?wp)))
  (assert (wm-fact (key domain fact wp-cap-color args? wp ?wp col CAP_NONE)))
  (assert (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order)))
  (assert (wm-fact (key domain fact wp-base-color args? wp ?wp col BASE_NONE)))
  (assert (wm-fact (key domain fact wp-ring1-color args? wp ?wp col RING_NONE)))
  (assert (wm-fact (key domain fact wp-ring2-color args? wp ?wp col RING_NONE)))
  (assert (wm-fact (key domain fact wp-ring3-color args? wp ?wp col RING_NONE)))
  (return ?wp)
)

; ============================= Enter-field ===============================

(defrule goal-production-create-enter-field
  "Enter the field (drive outside of the starting box)."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (not (goal (class ENTER-FIELD)))

  ;Get robot
  (wm-fact (key central agent robot args? r ?robot))
  (wm-fact (key domain fact robot-waiting args? r ?robot))

  ;Check game state
  (wm-fact (key refbox state) (value RUNNING))
  (wm-fact (key refbox phase) (value PRODUCTION|EXPLORATION))
  (wm-fact (key navgraph waitzone generated) (type BOOL) (value TRUE))
  (wm-fact (key refbox team-color) (value ?team-color))
  (NavGraphGeneratorInterface (final TRUE))

  ;Check if robot has entered the field already
  (not (wm-fact (key domain fact entered-field args? r ?robot)))
  
  ;Robot only enter in the correct order
  (not (and (eq ?robot robot2) (not (wm-fact (key domain fact entered-field args? r robot1)))))
  (not (and (eq ?robot robot3) (not (wm-fact (key domain fact entered-field args? r robot2)))))
  =>
  (printout t "Goal " ENTER-FIELD " formulated for " ?robot crlf)
  (assert (goal (id (sym-cat ENTER-FIELD- (gensym*)))
                (class ENTER-FIELD) (sub-type SIMPLE)
                (params r ?robot team-color ?team-color)))
)


; ============================= Production goals ===============================


(defrule goal-production-produce-c0
  "Create root goal of c0-production tree"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity&C0))

  ; get required base and cap color
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
  
  ; order is not already being handled
  (not (goal (class PRODUCE-C0) (params order ?order $?other-params)))
  ; more products ordered
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  (wm-fact (key domain fact quantity-delivered args? ord ?order team ?team-color)
	  (value ?qd&:(> ?qr ?qd)))

  ; get cap station
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-type args? m ?cap-station t CS))
  (wm-fact (key domain fact mps-team args? m ?cap-station col ?team-color))
  (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?cap-station spot ?spot))
  (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))

  ; get base station
  (wm-fact (key domain fact mps-type args? m ?base-station t BS))
  (wm-fact (key domain fact mps-team args? m ?base-station col ?team-color))

  ; get delivery station
  (wm-fact (key domain fact mps-type args? m ?ds t DS))
  (wm-fact (key domain fact mps-team args? m ?ds col ?team-color))

  =>
  (bind ?wp (create-wp ?order))
  (assert 
    (goal (id (sym-cat PRODUCE-C0- (gensym*)))
          (class PRODUCE-C0)
          (sub-type RUN-ALL-OF-SUBGOALS)
          (required-resources ?cap-station)
          (params order ?order
                  wp ?wp
                  cs ?cap-station
                  bs ?base-station
                  cap-color ?cap-color
                  base-color ?base-color
                  ds ?ds)
    )
  )
)

(defrule goal-production-produce-c0-create-subgoals
  "Create subgoals with parallelizable steps for c0 production"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?p <- (goal (id ?parent) (class PRODUCE-C0) (mode SELECTED)
              (params order ?order
                      wp ?wp
                      cs ?cap-station
                      bs ?base-station
                      cap-color ?cap-color
                      base-color ?base-color
                      ds ?ds))
  =>
  (assert
    (goal (id (sym-cat PRODUCE-C0-GET-BASE-AND-CAP-(gensym*)))
          (class PRODUCE-C0-GET-BASE-AND-CAP)
          (parent ?parent)
          (sub-type RUN-SUBGOALS-IN-PARALLEL)
          (priority 3.0)
    )
    (goal (id (sym-cat MOUNT-CAP-(gensym*)))
          (class MOUNT-CAP)
          (parent ?parent)
          (sub-type SIMPLE)
          (priority 2.0)
          (params cs ?cap-station cap-color ?cap-color wp ?wp)
    )
    (goal (id (sym-cat DELIVER-(gensym*)))
          (class DELIVER)
          (parent ?parent)
          (sub-type SIMPLE)
          (priority 1.0)
          (params order ?order
                  ds ?ds
                  wp ?wp)
    )
  )
  (modify ?p (mode EXPANDED))
)


(defrule goal-production-get-base-and-cap
  "Leaf goals to prepare a cap and remove the unused base and get a base running in parallel."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?p <- (goal (id ?parent) (class PRODUCE-C0-GET-BASE-AND-CAP) (mode SELECTED) (parent ?root))
  (goal (id ?root) (params order ?order
                            wp ?wp
                            cs ?cap-station
                            bs ?base-station
                            cap-color ?cap-color
                            base-color ?base-color
                            ds ?ds))
  (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?cap-station spot ?spot))
  (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
  =>
  (assert
    (goal (id (sym-cat FILL-CS-(gensym*)))
          (class FILL-CS)
          (parent ?parent)
          (sub-type SIMPLE)
          (params mps ?cap-station cc ?cc)
          (priority 2.0)
    )
    (goal (id (sym-cat GET-BASE-(gensym*)))
          (class GET-BASE)
          (parent ?parent)
          (sub-type SIMPLE)
          (params bs ?base-station
                  bs-side OUTPUT
                  bs-color ?base-color
                  target-station ?cap-station
                  wp ?wp)
          (priority 1.0)
    )
  )
  (modify ?p (mode EXPANDED))
)

(defrule assign-robot-to-production-goal
  "Select a non-busy robot for executing a production leaf goal without assigned robot"
  ?g <- (goal (id ?goal-id) (class ?class) (params $?params) (mode SELECTED))
  (test (goal-needs-fresh-robot ?class))
  (not (test (member$ robot $?params)))

  ; Get robot
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (goal (params robot ?robot $?some-params)))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?some-wp)))
  =>
  (printout t "Assigning " ?robot " to " ?goal-id crlf)
  (modify ?g (params robot ?robot $?params))
)

(defrule assign-robot-holding-wp
  ?g <- (goal (id ?goal-id) (class ?class) (params $?params) (mode SELECTED))
  (test (goal-needs-robot-holding-wp ?class))
  (not (test (member$ robot $?params)))
  (wm-fact (key domain fact holding args? r ?robot wp ?wp&:(member$ ?wp $?params)))
  =>
  (printout t "Assigning " ?robot " to " ?goal-id crlf)
  (modify ?g (params robot ?robot $?params))
)


(defrule clear-station
  "Formulate goal to remove robots from stations. This is only done unless the robot
  could execute another production goal"
  ; select non-busy robot
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (goal (params robot ?robot $?some-params)))

  ; check that robot is at input or output
  (domain-object (type mps) (name ?some-station))
  (wm-fact (key domain fact at args? r ?robot m ?some-station 
              side ?side&:(or (eq ?side INPUT) (eq ?side OUTPUT))))

  ; there is nothing else to do:
  (not (goal (class ?class&:(goal-needs-fresh-robot ?class))
            (mode SELECTED)
            (params $?params&:(not (member$ robot $?params)))))
  =>
  (assert (goal (id (sym-cat CLEAR-STATION-(gensym*)))
                (class CLEAR-STATION)
                (sub-type SIMPLE)
                (params robot ?robot)))
)
