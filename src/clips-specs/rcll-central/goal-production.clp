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

; =============== Execute actions requested by goal-reasoner ===========

; copied from rcll-agent
(defrule goal-production-create-reset-mps
" Reset an mps to restore a consistent world model after getting a workpiece
  from it failed too often.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (wm-fact (key evaluated reset-mps args? m ?mps))
  (not (goal (class RESET-MPS) (params m ?mps)))
  =>
  (printout t "Goal " RESET-MPS " formulated" crlf)
  (assert (goal (id (sym-cat RESET-MPS- (gensym*)))
                (class RESET-MPS) (sub-type SIMPLE)
                (params m ?mps)
  ))
)

(defrule goal-production-create-drop-wp
" Create goal to drop wp if requested by the goal reasoner after a production
  root goal has finally failed
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (wm-fact (key evaluated drop-wp args? r ?r wp ?wp))
  (not (goal (class DROP-WP) (params r ?r wp ?wp)))
  =>
  (printout t "GOAL " DROP-WP " formulated" crlf)
  (assert (goal (id (sym-cat DROP-WP- (gensym*)))
          (class DROP-WP) (sub-type SIMPLE)
          (params r ?r wp ?wp)
  ))
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

; ============================= Debug Options ===============================

(defglobal
  ; 1 = skip
  ?*DEBUG-SKIP-C0* = 1
  ?*DEBUG-SKIP-C1* = 1
  ?*DEBUG-SKIP-C2* = 1
  ?*DEBUG-SKIP-C3* = 0
  ?*DEBUG-SKIP-PF-CS* = 0
  ?*DEBUG-SKIP-PF-RS* = 0
)

; ============================= Production goals ===============================


(defrule goal-production-produce-c0
  "Create root goal of c0-production tree"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))

  ; check current game state
  (wm-fact (key refbox state) (value RUNNING))
  (wm-fact (key refbox phase) (value PRODUCTION))
  (domain-facts-loaded)

  ; get order of complexity C0
  (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity&C0))

  ; debugging conditions
  (test (neq ?*DEBUG-SKIP-C0* 1))
  ;(not (goal (class PRODUCE-C0)))

  ; get required base and cap color
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
  
  ; TODO: check for more products in single order
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
    (goal (id (sym-cat PRODUCE-C0-HANDLE-CS-(gensym*)))
          (class PRODUCE-C0-HANDLE-CS)
          (parent ?parent)
          (sub-type RUN-ALL-OF-SUBGOALS)
          (priority 2.0)
          (required-resources ?cap-station)
          (params cs ?cap-station
                  wp ?wp
                  bs ?base-station
                  cap-color ?cap-color
                  base-color ?base-color)
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

(defrule goal-production-produce-c0-handle-cs
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?p <- (goal (id ?parent)
              (class PRODUCE-C0-HANDLE-CS)
              (mode SELECTED)
              (params cs ?cap-station
                      wp ?wp
                      bs ?base-station
                      cap-color ?cap-color
                      base-color ?base-color))
  =>
  (assert
    (goal (id (sym-cat PRODUCE-C0-GET-BASE-AND-CAP-(gensym*)))
          (class PRODUCE-C0-GET-BASE-AND-CAP)
          (parent ?parent)
          (sub-type RUN-SUBGOALS-IN-PARALLEL)
          (priority 2.0)
          (params cs ?cap-station wp ?wp bs ?base-station
                  cap-color ?cap-color base-color ?base-color)
    )
    (goal (id (sym-cat MOUNT-CAP-(gensym*)))
          (class MOUNT-CAP)
          (parent ?parent)
          (sub-type SIMPLE)
          (priority 1.0)
          (params cs ?cap-station cap-color ?cap-color wp ?wp)
    )
  )
  (modify ?p (mode EXPANDED))
)


(defrule goal-production-get-base-and-cap
  "Leaf goals to prepare a cap and remove the unused base and get a base running in parallel."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?p <- (goal (id ?parent) (class PRODUCE-C0-GET-BASE-AND-CAP) (mode SELECTED)
              (params cs ?cap-station wp ?wp bs ?base-station
              cap-color ?cap-color base-color ?base-color))
  ; get a cap carrier
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

(defrule goal-production-produce-cx
  "Create root goal of cx-production tree"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))

  ; check current game state
  (wm-fact (key refbox state) (value RUNNING))
  (wm-fact (key refbox phase) (value PRODUCTION))
  (domain-facts-loaded)

  ; get order of complexity CX
  (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity&~C0))

  ; debugging conditions
  (test (or (neq ?*DEBUG-SKIP-C1* = 1) (neq ?complexity C1)))
  (test (or (neq ?*DEBUG-SKIP-C2* = 1) (neq ?complexity C2)))
  (test (or (neq ?*DEBUG-SKIP-C3* = 1) (neq ?complexity C3)))
  ;(not (goal (class PRODUCE-CX)))

  ; get all required colors
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))

  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring-color1))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?ring-color2))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?ring-color3))

  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
  
  ; TODO: check for more products in single order
  ; order is not already being handled
  (not (goal (class PRODUCE-CX) (params order ?order $?other-params)))
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

  ; get ring station and number of required bases
  (wm-fact (key domain fact rs-ring-spec args? m ?ring-station1 r ?ring-color1 rn ?ring-base-req1&~NA))
  (wm-fact (key domain fact mps-team args? m ?ring-station1 col ?team-color))
  (wm-fact (key domain fact rs-ring-spec args? m ?ring-station2 r ?ring-color2 rn ?ring-base-req2&~NA))
  (wm-fact (key domain fact mps-team args? m ?ring-station2 col ?team-color))
  (wm-fact (key domain fact rs-ring-spec args? m ?ring-station3 r ?ring-color3 rn ?ring-base-req3&~NA))
  (wm-fact (key domain fact mps-team args? m ?ring-station3 col ?team-color))

  ; get delivery station
  (wm-fact (key domain fact mps-type args? m ?ds t DS))
  (wm-fact (key domain fact mps-team args? m ?ds col ?team-color))

  =>
  (bind ?wp (create-wp ?order))
  (assert 
    (goal (id (sym-cat PRODUCE-CX- (gensym*)))
          (class PRODUCE-CX)
          (sub-type RUN-ALL-OF-SUBGOALS)
          (params order ?order
                  wp ?wp
                  complexity ?complexity
                  cs ?cap-station
                  bs ?base-station
                  rs1 ?ring-station1
                  rs2 ?ring-station2
                  rs3 ?ring-station3
                  cap-color ?cap-color
                  base-color ?base-color
                  ring-color1 ?ring-color1
                  ring-color2 ?ring-color2
                  ring-color3 ?ring-color3
                  ring-base-req1 ?ring-base-req1
                  ring-base-req2 ?ring-base-req2
                  ring-base-req3 ?ring-base-req3
                  ds ?ds)
    )
  )
)

(defrule goal-production-produce-cx-create-subgoals
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?p <- (goal (id ?parent) (class PRODUCE-CX) (mode SELECTED)
        (params order ?order
                wp ?wp
                complexity ?complexity
                cs ?cap-station
                bs ?base-station
                rs1 ?ring-station1
                rs2 ?ring-station2
                rs3 ?ring-station3
                cap-color ?cap-color
                base-color ?base-color
                ring-color1 ?ring-color1
                ring-color2 ?ring-color2
                ring-color3 ?ring-color3
                ring-base-req1 ?ring-base-req1
                ring-base-req2 ?ring-base-req2
                ring-base-req3 ?ring-base-req3
                ds ?ds))
  =>
  (assert
    (goal (id (sym-cat PRODUCE-CX-HANDLE-RS-(gensym*)))
          (parent ?parent)
          (class PRODUCE-CX-HANDLE-RS)
          (sub-type RUN-ALL-OF-SUBGOALS)
          (priority 4.0)
          (required-resources ?ring-station1)
          (params order ?order
                  wp ?wp
                  bs ?base-station
                  rs ?ring-station1
                  base-color ?base-color
                  ring-color ?ring-color1
                  ring-mount-index ONE
                  ring-base-req ?ring-base-req1)
    )
  )
  (if ( or (eq ?complexity C2) (eq ?complexity C3)) then
    (assert
      (goal (id (sym-cat PRODUCE-CX-HANDLE-RS-(gensym*)))
            (parent ?parent)
            (class PRODUCE-CX-HANDLE-RS)
            (sub-type RUN-ALL-OF-SUBGOALS)
            (priority 3.0)
            (required-resources ?ring-station2)
            (params order ?order
                    wp ?wp
                    bs ?base-station
                    rs ?ring-station2
                    base-color ?base-color
                    ring-color ?ring-color2
                    ring-mount-index TWO
                    ring-base-req ?ring-base-req2)
      )
    )
  )
  (if (eq ?complexity C3) then
    (assert
      (goal (id (sym-cat PRODUCE-CX-HANDLE-RS-(gensym*)))
            (parent ?parent)
            (class PRODUCE-CX-HANDLE-RS)
            (sub-type RUN-ALL-OF-SUBGOALS)
            (priority 2.0)
            (required-resources ?ring-station3)
            (params order ?order
                    wp ?wp
                    bs ?base-station
                    rs ?ring-station3
                    base-color ?base-color
                    ring-color ?ring-color3
                    ring-mount-index THREE
                    ring-base-req ?ring-base-req3)
      )
    )
  )
  (assert
    (goal (id (sym-cat PRODUCE-CX-HANDLE-CS-(gensym*)))
          (parent ?parent)
          (class PRODUCE-CX-HANDLE-CS)
          (sub-type RUN-ALL-OF-SUBGOALS)
          (priority 1.0)
          (required-resources ?cap-station)
          (params order ?order
                  wp ?wp
                  cs ?cap-station
                  cap-color ?cap-color
                  ds ?ds)
    )
    (goal (id (sym-cat DELIVER-(gensym*)))
          (parent ?parent)
          (class DELIVER)
          (sub-type SIMPLE)
          (priority 0.0)
          (params order ?order
                  ds ?ds
                  wp ?wp)
          
    )
  )
  (modify ?p (mode EXPANDED))
)


(defrule goal-production-produce-cx-handle-rs
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?p <- (goal (id ?parent) (class PRODUCE-CX-HANDLE-RS) (mode SELECTED)
              (params order ?order
                      wp ?wp
                      bs ?base-station
                      rs ?ring-station
                      base-color ?base-color
                      ring-color ?ring-color
                      ring-mount-index ?ring-mount-index
                      ring-base-req ?ring-base-req))
  =>
  (assert
    (goal (id (sym-cat FILL-BASES-IN-RS-(gensym*)))
          (parent ?parent)
          (class FILL-BASES-IN-RS)
          (sub-type RUN-SUBGOALS-IN-PARALLEL)
          (priority 3.0)
          (params bs ?base-station
                  rs ?ring-station
                  ring-base-req ?ring-base-req)
    )
  )
  (if (eq ?ring-mount-index ONE)
  then
    (assert
      (goal (id (sym-cat GET-BASE-(gensym*)))
            (parent ?parent)
            (class GET-BASE)
            (sub-type SIMPLE)
            (priority 2.0)
            (params bs ?base-station
                    bs-side OUTPUT
                    bs-color ?base-color
                    target-station ?ring-station
                    wp ?wp)
      )
    )
  else
    (assert
      (goal (id (sym-cat PICKUP-WP-(gensym*)))
            (parent ?parent)
            (class PICKUP-WP)
            (sub-type SIMPLE)
            (priority 2.0)
            (params wp ?wp)
      )
    )
  )
  (assert
    (goal (id (sym-cat CLEAR-OUTPUT-(gensym*)))
          (parent ?parent)
          (class CLEAR-OUTPUT)
          (sub-type SIMPLE)
          (priority 1.0)
          (params mps ?ring-station mps-side OUTPUT)
    )
    (goal (id (sym-cat MOUNT-RING-(gensym*)))
          (parent ?parent)
          (class MOUNT-RING)
          (sub-type SIMPLE)
          (priority 0.0)
          (params rs ?ring-station
                  ring-mount-index ?ring-mount-index
                  ring-color ?ring-color
                  ring-base-req ?ring-base-req
                  wp ?wp)
    )
  )
  (modify ?p (mode EXPANDED))
)

(defrule goal-production-produce-cx-handle-cs
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?p <- (goal (id ?parent) (class PRODUCE-CX-HANDLE-CS) (mode SELECTED)
              (params order ?order
                      wp ?wp
                      cs ?cap-station
                      cap-color ?cap-color
                      ds ?ds))
  (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?cap-station spot ?spot))
  (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
  =>
  (assert
    (goal (id (sym-cat FILL-CS-(gensym*)))
          (parent ?parent)
          (class FILL-CS)
          (sub-type SIMPLE)
          (priority 2.0)
          (params mps ?cap-station cc ?cc)
    )
    (goal (id (sym-cat PICKUP-WP-(gensym*)))
          (parent ?parent)
          (class PICKUP-WP)
          (sub-type SIMPLE)
          (priority 1.0)
          (params wp ?wp)
    )
    (goal (id (sym-cat MOUNT-CAP-(gensym*)))
          (parent ?parent)
          (class MOUNT-CAP)
          (sub-type SIMPLE)
          (priority 0.0)
          (params cs ?cap-station cap-color ?cap-color wp ?wp)
    )
  )
  (modify ?p (mode EXPANDED))
)

; TODO: check case that no additional bases are needed -> fix with if (might not be working)
; TODO: formulates before async mount ring completes -> wrong ring count (should be fixed with IDLE)
(defrule goal-production-fill-bases-in-rs
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?p <- (goal (id ?parent) (class FILL-BASES-IN-RS) (mode SELECTED)
              (params bs ?base-station
                      rs ?ring-station
                      ring-base-req ?ring-base-req))
  (wm-fact (key domain fact mps-state args? m ?ring-station s IDLE|PROCESSED))
  (wm-fact (key domain fact rs-filled-with args? m ?ring-station n ?rs-before))
  (wm-fact (key domain fact rs-sub args? minuend ?ring-base-req subtrahend ?rs-before difference ?rs-needed))
  =>
  (bind ?rs-needed-int (sym-to-int ?rs-needed))
  (printout t "Ring needs " ?ring-base-req " bases, " ?rs-before " in ring station, " ?rs-needed " to get which is " ?rs-needed-int crlf)

  ; create as many fill goals as necessary
  (if (> ?rs-needed-int 0) then
    (assert (goal (id (sym-cat FILL-BASE-IN-RS-(gensym*)))
                  (parent ?parent)
                  (class FILL-BASE-IN-RS)
                  (sub-type RUN-ALL-OF-SUBGOALS)
                  (params bs ?base-station rs ?ring-station))
    )  
  )
  (if (> ?rs-needed-int 1) then
    (assert (goal (id (sym-cat FILL-BASE-IN-RS-(gensym*)))
                  (parent ?parent)
                  (class FILL-BASE-IN-RS)
                  (sub-type RUN-ALL-OF-SUBGOALS)
                  (params bs ?base-station rs ?ring-station))
    )  
  )
  (if (> ?rs-needed-int 2) then
    (assert (goal (id (sym-cat FILL-BASE-IN-RS-(gensym*)))
                  (parent ?parent)
                  (class FILL-BASE-IN-RS)
                  (sub-type RUN-ALL-OF-SUBGOALS)
                  (params bs ?base-station rs ?ring-station))
    )
  )
  ; if no additional bases necessary, terminate goal
  (if (eq ?rs-needed-int 0) 
  then (modify ?p (mode FINISHED) (outcome COMPLETED))
  else (modify ?p (mode EXPANDED))
  )
)

(defrule goal-production-fill-base-in-rs
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?p <- (goal (id ?parent) (class FILL-BASE-IN-RS) (mode SELECTED)
              (params bs ?base-station rs ?ring-station))
  =>
  (bind ?wp (create-wp (sym-cat FILL-BASE- (gensym*))))
  (bind ?base-color (sym-cat BASE_BLACK))
  (assert
    (goal (id (sym-cat GET-BASE-(gensym*)))
          (parent ?parent)
          (class GET-BASE)
          (sub-type SIMPLE)
          (priority 1.0)
          (params bs ?base-station
                  bs-side OUTPUT
                  bs-color ?base-color
                  target-station ?ring-station
                  wp ?wp)
    )
    (goal (id (sym-cat FILL-RS-(gensym*)))
          (parent ?parent)
          (class FILL-RS)
          (sub-type SIMPLE)
          (priority 0.0)
          (params bs ?base-station 
                  rs ?ring-station 
                  wp ?wp)
    )
  )
  (modify ?p (mode EXPANDED))
)

; ============================= Passive Prefills ===============================

(defrule passive-prefill-cap-station
  "Prefill a cap station without an immediate need in preparation for future production"
  ; TODO: own salience?
  ;(declare (salience ?*SALIENCE-GOAL-FORMULATE*))

  ; debugging conditions (not working)
  ;(test (neq ?*DEBUG-SKIP-PF-CS* 1))

  ; select non-busy robot
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (goal (params robot ?robot $?some-params)))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?some-wp)))

  ; there is nothing else to do:
  (not (goal (class ?class&:(goal-needs-fresh-robot ?class))
            (mode SELECTED)
            (params $?params&:(not (member$ robot $?params)))))

  ; get a cap station
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-type args? m ?cap-station t CS))
  (wm-fact (key domain fact mps-team args? m ?cap-station col ?team-color))
  (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?cap-station spot ?spot))
  (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
  (wm-fact (key domain fact mps-state args? m ?cap-station s IDLE|PROCESSED))

  ; cap station not filled
  (not (wm-fact (key domain fact cs-buffered args? m ?cap-station col ?cap-color)))

  ; don't formulate goal if it can't be executed immediatley
  (not (goal (acquired-resources ?cap-station)))

  ; get corresponding cap carrier
  (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?cap-station spot ?spot))
  (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
  =>
  (assert
    (goal (id (sym-cat FILL-CS-(gensym*)))
          (class FILL-CS)
          (sub-type SIMPLE)
          (params robot ?robot mps ?cap-station cc ?cc)
          (required-resources ?cap-station)
          (priority 1.0)
    )
  )
)

(defrule passive-prefill-ring-station
  "Prefill a ring station without an immediate need in preparation for future production"
  ; TODO: own salience?
  ;(declare (salience ?*SALIENCE-GOAL-FORMULATE*))

  ; debugging conditions (not working)
  ;(test (neq ?*DEBUG-SKIP-PF-RS* 1))

  ; select non-busy robot
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (goal (params robot ?robot $?some-params)))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?some-wp)))

  ; there is nothing else to do:
  (not (goal (class ?class&:(goal-needs-fresh-robot ?class))
            (mode SELECTED)
            (params $?params&:(not (member$ robot $?params)))))

  ; get base station
  (wm-fact (key domain fact mps-type args? m ?base-station t BS))
  (wm-fact (key domain fact mps-team args? m ?base-station col ?team-color))

  ; get ring station
  (wm-fact (key domain fact mps-team args? m ?ring-station col ?team-color))
  (wm-fact (key domain fact mps-state args? m ?ring-station s IDLE|PROCESSED))

  ; TODO: discuss max bases vs THREE
  ; ring station not at max bases
  (wm-fact (key domain fact rs-ring-spec args? m ?ring-station r ?ring-color rn ?ring-base-req))
  (not (wm-fact (key domain fact rs-ring-spec args? m ?ring-station r ?other-ring-color 
        rn ?other-ring-base-req&:(> (sym-to-int ?other-ring-base-req) (sym-to-int ?ring-base-req)))))
  (not (wm-fact (key domain fact rs-filled-with args? m ?ring-station n ?ring-base-req)))

  ; don't formulate goal if it can't be executed immediatley
  (not (goal (acquired-resources ?ring-station)))
  =>
  ; create goals to add 1 base to the the ring station
  (assert (goal (id (sym-cat FILL-BASE-IN-RS-(gensym*)))
                  (class FILL-BASE-IN-RS)
                  (sub-type RUN-ALL-OF-SUBGOALS)
                  (required-resources ?ring-station)
                  (params bs ?base-station rs ?ring-station))
  ) 
)


; ============================= Robot selection ===============================

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