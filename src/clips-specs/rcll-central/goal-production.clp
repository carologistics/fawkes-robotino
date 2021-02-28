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
  (assert (wm-fact (key domain fact wp-cap-color args? wp ?wp col CAP_NONE) (value TRUE)))
  (assert (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order)))
  (assert (wm-fact (key domain fact wp-base-color args? wp ?wp col BASE_NONE) (value TRUE)))
  (assert (wm-fact (key domain fact wp-ring1-color args? wp ?wp col RING_NONE) (value TRUE)))
  (assert (wm-fact (key domain fact wp-ring2-color args? wp ?wp col RING_NONE) (value TRUE)))
  (assert (wm-fact (key domain fact wp-ring3-color args? wp ?wp col RING_NONE) (value TRUE)))
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
          (params robot ?r wp ?wp)
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
  ?*DEBUG-SKIP-C0* = 0
  ?*DEBUG-SKIP-C1* = 0
  ?*DEBUG-SKIP-C2* = 0
  ?*DEBUG-SKIP-C3* = 0
  ?*DEBUG-SKIP-PF-CS* = 0
  ?*DEBUG-SKIP-PF-RS* = 0
  ; currently unusable due to SS implementation:
  ?*USE-SS-INPUT* = 1
)


; ============================= Estimators/Priorities ===============================

(deffunction order-time-estimate-upper (?complexity ?competitive)
  (bind ?comp 0)
  (if (eq ?competitive TRUE) then (bind ?comp 30))
  (switch ?complexity
    (case C0 then (return (+ 150 ?comp)))
    (case C1 then (return (+ 210 ?comp)))
    (case C2 then (return (+ 330 ?comp)))
    (case C3 then (return (+ 390 ?comp)))
    (default none)
  )
)

(deffunction order-time-estimate-lower (?complexity ?competitive)
  (switch ?complexity
    (case C0 then (return 150))
    (case C1 then (return 200))
    (case C2 then (return 280))
    (case C3 then (return 350))
    (default none)
  )
)

(deffunction order-base-priority (?complexity)
  (switch ?complexity
    (case C0 then (return 0))
    (case C1 then (return 100))
    (case C2 then (return 200))
    (case C3 then (return 300))
    (default none)
  )
)


; ============================= Production goals ===============================

; ========== root goal rules ==========

(defrule goal-production-produce-cx
  "Create root goal of a cx-production tree. Includes all production steps of a product."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))

  ; check current game state
  (wm-fact (key refbox state) (value RUNNING))
  (wm-fact (key refbox phase) (value PRODUCTION))
  (domain-facts-loaded)

  ; get order of complexity CX
  (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity))

  ; debugging conditions
  (test (or (neq ?*DEBUG-SKIP-C0* = 1) (neq ?complexity C0)))
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

  ; order is not already being handled
  (not (goal (class PRODUCE-CX) (params order ?order $?other-params)))

  ; more products ordered
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  (wm-fact (key domain fact quantity-delivered args? ord ?order team ?team-color)
	  (value ?qd&:(> ?qr ?qd)))
  (not (goal (class HANDLE-MPS) (params ?ds)))

  ; is order competitive?
  (wm-fact (key order meta competitive args? ord ?order) (value ?competitive))

  ; start production for order only in a certain time frame around the deliver window
  (wm-fact (key refbox game-time) (values $?game-time))
  (wm-fact (key refbox order ?order delivery-begin) (type UINT)
	  (value ?begin&:(< ?begin (+ (nth$ 1 ?game-time) (order-time-estimate-upper ?complexity ?competitive)))))
  (wm-fact (key refbox order ?order delivery-end) (type UINT)
	  (value ?end&:(> ?end (+ (nth$ 1 ?game-time) (order-time-estimate-lower ?complexity ?competitive)))))
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
          (meta global-priority (+ (order-base-priority ?complexity) (- 700 (* (div ?end 5) 3))))
    )
  )
  (printout t "Order " ?order " formulated with priority " (+ (order-base-priority ?complexity) (- 700 (* (div ?end 5) 3))) crlf)
)

(defrule goal-production-produce-cx-create-subgoals
  "Expands CX root goal by formulationg all necessary subgoals."
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
                      ds ?ds
              )
              (meta $? global-priority ?pprio $?)
        )
  ; check that all facts have been defined
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?wp-ring1-color) (value TRUE))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?wp-ring2-color) (value TRUE))
  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?wp-ring3-color) (value TRUE))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col ?wp-cap-color) (value TRUE))
  =>
  ; C1, C2 and C3 and not recovering from failure where step already completed
  (if (and (neq ?wp-ring1-color ?ring-color1) (neq ?complexity C0)) then
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
            (meta global-priority (+ ?pprio 100))
      )
    )
  )
  ; C2 and C3 and not recovering from failure where step already completed
  (if (and (neq ?wp-ring2-color ?ring-color2) (or (eq ?complexity C2) (eq ?complexity C3))) then
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
            (meta global-priority (+ ?pprio 700))
      )
    )
  )
  ; C3 and not recovering from failure where step already completed
  (if (and (neq ?wp-ring3-color ?ring-color3) (eq ?complexity C3)) then
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
            (meta global-priority (+ ?pprio 700))
      )
    )
  )
  ; all and not recovering from failure where step already completed
  (if (neq ?wp-cap-color ?cap-color) then
    (assert
      (goal (id (sym-cat PRODUCE-CX-HANDLE-CS-(gensym*)))
            (parent ?parent)
            (class PRODUCE-CX-HANDLE-CS)
            (sub-type RUN-ALL-OF-SUBGOALS)
            (priority 1.0)
            (required-resources ?cap-station)
            (params cs ?cap-station
                    wp ?wp
                    complexity ?complexity
                    bs ?base-station
                    cap-color ?cap-color
                    base-color ?base-color)
            (meta global-priority (+ ?pprio 700))
      )
    )
  )
  ; all
  (assert
    (goal (id (sym-cat PICKUP-AND-DELIVER-(gensym*)))
          (parent ?parent)
          (class PICKUP-AND-DELIVER)
          (sub-type RUN-ALL-OF-SUBGOALS)
          (priority 0.0)
          (params order ?order
                  ds ?ds
                  wp ?wp)
          (meta global-priority (+ ?pprio 1200))
    )
  )
  (modify ?p (mode EXPANDED))
)

; ========== ring station rules ==========

(defrule goal-production-produce-cx-handle-rs
  "Root goal for all production steps requiring a ring station."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?p <- (goal (id ?parent) (class PRODUCE-CX-HANDLE-RS) (mode SELECTED)
              (params order ?order
                      wp ?wp
                      bs ?base-station
                      rs ?ring-station
                      base-color ?base-color
                      ring-color ?ring-color
                      ring-mount-index ?ring-mount-index
                      ring-base-req ?ring-base-req
              )
        )
  ; TODO: value true? 
  (wm-fact (key domain fact wp-base-color args? wp ?wp col ?wp-base-color))
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
  ; get base only needed if we are at ring index one and not recovering from a fail
  (if (and (eq ?ring-mount-index ONE) (neq ?wp-base-color ?base-color))
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

(defrule goal-production-fill-bases-in-rs
  "Fill as many bases as needed into a ring-station for a certain ring mount."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?p <- (goal (id ?parent) (class FILL-BASES-IN-RS) (mode SELECTED)
              (params bs ?base-station
                      rs ?ring-station
                      ring-base-req ?ring-base-req
              )
        )
  ; avoid wrong ring counts if the machine is still active
  (wm-fact (key domain fact mps-state args? m ?ring-station s ~BROKEN&~PROCESSING&~DOWN))

  ; get current fill status of ring station
  (wm-fact (key domain fact rs-filled-with args? m ?ring-station n ?rs-before))
  =>
  ; calculate necessary bases
  (bind ?ring-base-req-int (sym-to-int ?ring-base-req))
  (bind ?rs-before-int (sym-to-int ?rs-before))
  (bind ?rs-needed-int (- ?ring-base-req-int ?rs-before-int))
  
  ;(printout t "Ring needs " ?ring-base-req-int " bases, " ?rs-before-int " in ring station, " ?rs-needed-int " needed" crlf)

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
  (if (<= ?rs-needed-int 0) 
  then (modify ?p (mode FINISHED) (outcome COMPLETED))
  else (modify ?p (mode EXPANDED))
  )
)

(defrule goal-production-fill-base-in-rs
  "Fill a single fresh base into a ring station."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?p <- (goal (id ?parent) (class FILL-BASE-IN-RS) (mode SELECTED)
              (params bs ?base-station rs ?ring-station)
        )
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
          (params rs ?ring-station 
                  wp ?wp)
    )
  )
  (modify ?p (mode EXPANDED))
)

; ========== cap station rules ==========
(defrule goal-production-produce-cx-handle-cs
  "Root goal for all production steps requiring a cap station."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?p <- (goal (id ?parent) (class PRODUCE-CX-HANDLE-CS) (mode SELECTED)
              (params cs ?cap-station
                      wp ?wp
                      complexity ?complexity
                      bs ?base-station
                      cap-color ?cap-color
                      base-color ?base-color
              )
        )
  =>
  (assert
    (goal (id (sym-cat CLEAR-OUTPUT-(gensym*)))
          (class CLEAR-OUTPUT)
          (parent ?parent)
          (sub-type SIMPLE)
          (params mps ?cap-station mps-side OUTPUT)
          (priority 4.0)
    )
    (goal (id (sym-cat BUFFER-CS-(gensym*)))
          (class BUFFER-CS)
          (parent ?parent)
          (sub-type SIMPLE)
          (priority 3.0)
          (params mps ?cap-station)
    )
    (goal (id (sym-cat GET-BASE-AND-REMOVE-CC-(gensym*)))
          (class GET-BASE-AND-REMOVE-CC)
          (parent ?parent)
          (sub-type RUN-SUBGOALS-IN-PARALLEL)
          (priority 2.0)
          (params cs ?cap-station wp ?wp complexity ?complexity bs ?base-station
                  base-color ?base-color)
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

(defrule goal-production-get-base-and-remove-cc
  "Remove the cc after retrieving its cap and get the main base in parallel."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?p <- (goal (id ?parent) (class GET-BASE-AND-REMOVE-CC) (mode SELECTED)
              (params cs ?cap-station wp ?wp complexity ?complexity bs ?base-station
                      base-color ?base-color
              )
        )
  (wm-fact (key domain fact wp-base-color args? wp ?wp col ?wp-base-color) (value TRUE))
  =>
  ; remove cc
  (assert
    (goal (id (sym-cat CLEAR-OUTPUT-(gensym*)))
          (class CLEAR-OUTPUT)
          (parent ?parent)
          (sub-type SIMPLE)
          (params mps ?cap-station mps-side OUTPUT)
          (priority 2.0)
    )
  )
  ; get new base for C0 or pickup wp for C1-C3
  (if (eq ?complexity C0)
  then
    (if (neq ?wp-base-color ?base-color) then
      (assert
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
    )
  else
    (assert
      (goal (id (sym-cat PICKUP-WP-(gensym*)))
            (parent ?parent)
            (class PICKUP-WP)
            (sub-type SIMPLE)
            (priority 1.0)
            (params wp ?wp)
      )
    )
  )
  (modify ?p (mode EXPANDED))
)


; ========== delivery rules ==========
(defrule create-subgoals-pickup-and-deliver
 "Create subgoals to pickup and deliver a workpiece"
 (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
 ?g <-  (goal (id ?parent) (class PICKUP-AND-DELIVER) (mode SELECTED) 
            (params order ?order
                    ds ?ds
                    wp ?wp
            )
        )

  ; get delivery time for order
  (wm-fact (key refbox order ?order delivery-begin) (type UINT) (value ?begin))
  ; check time so that the delivery goals is not expanded too early
  (wm-fact (key refbox game-time) (values ?game-time&:(> (+ ?game-time 45) ?begin) $?))
 =>
 (assert
  (goal (id (sym-cat PICKUP-WP-(gensym*)))
            (parent ?parent)
            (class PICKUP-WP)
            (sub-type SIMPLE)
            (priority 1.0)
            (params wp ?wp)
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
  (modify ?g (mode EXPANDED))
)


; ============================= Passive Optimization ===============================

; ========== positioning ==========

(defrule clear-station
  "Formulate goal to remove robots from stations. This is only done unless the robot
  could execute another production goal
  "
  ; select non-busy robot
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (goal (params robot ?robot $?some-params)))

  ; check that robot is at input or output, not wait
  (domain-object (type mps) (name ?some-station))
  (wm-fact (key domain fact at args? r ?robot m ?some-station 
              side ?side&:(or (eq ?side INPUT) (eq ?side OUTPUT))))

  ; there is nothing else to do:
  (not (goal (class ?class&:(goal-needs-fresh-robot ?class))
            (mode SELECTED)
            (params $?params&:(not (member$ robot $?params)))
       )
  )
  (not (and (goal (class ?class&:(goal-needs-robot-holding-wp ?class))
            (mode SELECTED)
            (params $?paramsprefix wp ?wp $?paramssuffix))
            (wm-fact (key domain fact holding args? r ?robot wp ?wp))
       )
  )
  =>
  (assert (goal (id (sym-cat CLEAR-STATION-(gensym*)))
                (class CLEAR-STATION)
                (sub-type SIMPLE)
                (params robot ?robot)))
)

(defrule passive-go-wait-at-next-station
  "Even if a robot cannot reserve the next station he can still drive there already."
  ; Get non-busy robot holding a workpiece
  (wm-fact (key domain fact holding args? r ?robot wp ?wp))
  (not (goal (params robot ?robot $?some-params)))

  ; Check the next goal for the wp to find out to which station we need to go
  (goal (id ?root-id) (class PRODUCE-C0|PRODUCE-CX) (params $? wp ?wp $?))
  (goal (id ?id) (parent ?root-id) (mode SELECTED|COMMITTED|DISPATCHED) (required-resources ?mps))

  ; We are not already at the next station
  (not (wm-fact (key domain fact at args? r ?robot m ?pos&:(eq ?pos (wait-pos ?mps INPUT)) side WAIT)))
  =>
  (printout t "Robot " ?robot " moves to next station " ?mps " to wait for future goal " ?id " ." crlf)
  (assert (goal (id (sym-cat GO-WAIT-(gensym*))) 
                (class GO-WAIT)
                (sub-type SIMPLE)
                (params robot ?robot mps ?mps mps-side INPUT)
          )
  )
)

; ========== wp reuse ==========

(defrule goal-production-wp-disposable
  "Some wps are marked as disposable if they are no longer part of an active
  production process (ccs, failed wps). They can be used to fill ringstations.
  "
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))

  ; robot holding some wp
  (wm-fact (key domain fact holding args? r ?robot wp ?wp))

  ; wp not already disposable
  (not (disposable ?wp))

  ; either the wp is a cc without cap (cap was buffered successfully) or..
  (or (and (not (wm-fact (key domain fact wp-base-color args? wp ?wp col ?)))
           (wm-fact (key domain fact wp-cap-color args? wp ?wp col CAP_NONE)))
      ; .. the cc has a cap which will not be removed (buffering failed) or..
      (and (not (wm-fact (key domain fact wp-base-color args? wp ?wp col ?)))
           (not (goal (class BUFFER-CS|FILL-CS) (params robot ?robot $?))))
      ; .. the wp is not a cc and doesn't belong to any goal (most likely belonged
      ; to a failed goal)
      (and (wm-fact (key domain fact wp-base-color args? wp ?wp col ?))
           (not (goal (params $? ?wp $?))))
  )
  =>
  (printout t "Setting workpiece " ?wp " to disposable at robot " ?robot "."   crlf)
  (assert (disposable ?wp))
)

(defrule goal-production-reuse-cc
  "There exist disposable wps that can be used to fill ring station instead of using bases.
  Use these if a robot is hoding one and he can do nothing else.
  "
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))

  ; non-busy robot is holding a disposable wp
  (wm-fact (key domain fact holding args? r ?robot wp ?wp))
  (disposable ?wp)
  (not (goal (params robot ?robot $?)))

  ; there is nothing else to do for the robot
  (not (goal (class ?class&:(goal-needs-fresh-robot ?class))
             (mode SELECTED)
             (params $?params&:(not (member$ robot $?params)))
       )
  )

  ; get ring station
  (wm-fact (key domain fact mps-team args? m ?ring-station col ?team-color))
  (wm-fact (key domain fact mps-state args? m ?ring-station s ~BROKEN&~PROCESSING&~DOWN))

  ; ring station not at max bases
  (wm-fact (key domain fact rs-filled-with args? m ?ring-station n ?filled&~NA&:(< (sym-to-int ?filled) 3)))

  ; check time (keep robot available late game)
  (wm-fact (key refbox game-time) (values ?game-time&:(< ?game-time 720) $?))

  ; don't formulate goal if it can't be executed immediatley
  (not (goal (acquired-resources ?ring-station)))
  =>
  (assert
    (goal (id (sym-cat FILL-RS-(gensym*)))
          (class FILL-RS)
          (sub-type SIMPLE)
          (params robot ?robot
                  rs ?ring-station 
                  wp ?wp)
          (required-resources ?ring-station)
    )
  )
)

(defrule goal-production-discard-cc
"The robot is holding a disposable wp and the reuse rule did not trigger,
therefore, discard the disposable wp and free the robot.
"
  ; non-busy robot is holding disposable wp
  (wm-fact (key domain fact holding args? r ?robot wp ?wp))
  (disposable ?wp)
  (not (goal (params robot ?robot $?)))
  =>
  (assert
    (goal (id (sym-cat DROP-WP-(gensym*)))
          (class DROP-WP)
          (sub-type SIMPLE)
          (params robot ?robot wp ?wp)
    )
  )
)

; ========== storage ==========

; TODO: not working, wp-put fails at storage station (not our fault)
(defrule passive-store-wp-before-deliver
  "Get a robot holding a workpiece waiting for delivery in a furture time slot and
  store it in the storage station if possible." 
  ; storage station activated? 
  (test (neq ?*USE-SS-INPUT* 1))

  ; Get a robot
  (wm-fact (key domain fact holding args? r ?robot wp ?wp))
  (not (goal (params robot ?robot $?some-params)))

  ; wp is to be delivered
  (goal (id ?id) (class PICKUP-AND-DELIVER) (mode SELECTED) (params order ?order ds ?ds wp ?wp))

  ; get delivery time for order
  (wm-fact (key refbox order ?order delivery-begin) (type UINT) (value ?begin))

  ; check time
  (wm-fact (key refbox game-time) (values ?game-time&:(< (+ ?game-time 120) ?begin) $?))

  ; get storage station and available side
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-type args? m ?mps t SS))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (wm-fact (key domain fact mps-side-free args? m ?mps side ?mps-side&INPUT))
  =>
  (printout t "Robot stores wp at storage station for future delivery." crlf)
  (assert (goal (id (sym-cat STORE-WP-(gensym*))) 
                (class STORE-WP)
                (sub-type SIMPLE)
                (params robot ?robot wp ?wp mps ?mps mps-side ?mps-side))
  )
)

; ========== prefills ==========

(defrule passive-prefill-cap-station
  "Prefill a cap station without an immediate need in preparation for future production."
  ; TODO: own salience?
  (declare (salience -100))

  ; debugging conditions TODO:(not working)
  ;(test (neq ?*DEBUG-SKIP-PF-CS* 1))

  ; select non-busy robot
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (goal (params robot ?robot $?some-params)))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?some-wp)))

  ; there is nothing else to do:
  (not (goal (class ?class&:(goal-needs-fresh-robot ?class))
            (mode SELECTED)
            (params $?params&:(not (member$ robot $?params)))
        )
  )

  ; get a cap station
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-type args? m ?cap-station t CS))
  (wm-fact (key domain fact mps-team args? m ?cap-station col ?team-color))
  (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?cap-station spot ?spot))
  (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
  (wm-fact (key domain fact mps-state args? m ?cap-station s ~BROKEN&~PROCESSING&~DOWN&~READY-AT-OUTPUT))

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
          (params robot ?robot mps ?cap-station)
          (required-resources ?cap-station)
          (priority 1.0)
          (meta global-priority 0)
    )
  )
)

(defrule passive-prefill-ring-station
  "Prefill a ring station without an immediate need in preparation for future production"
  ; TODO: own salience?
  (declare (salience -200))

  ; debugging conditions TODO:(not working)
  ;(test (neq ?*DEBUG-SKIP-PF-RS* 1))

  ; select non-busy robot
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (goal (params robot ?robot $?some-params)))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?some-wp)))

  ; there is nothing else to do:
  (not (goal (class ?class&:(goal-needs-fresh-robot ?class))
            (mode SELECTED)
            (params $?params&:(not (member$ robot $?params)))
       )
  )

  ; get base station
  (wm-fact (key domain fact mps-type args? m ?base-station t BS))
  (wm-fact (key domain fact mps-team args? m ?base-station col ?team-color))

  ; get ring station
  (wm-fact (key domain fact mps-team args? m ?ring-station col ?team-color))
  (wm-fact (key domain fact mps-state args? m ?ring-station s ~BROKEN&~PROCESSING&~DOWN))

  ; TODO: remove?
  ; TODO: discuss max bases vs THREE (done)
  ; ring station not at max bases
  ;(wm-fact (key domain fact rs-ring-spec args? m ?ring-station r ?ring-color rn ?ring-base-req&~NA))
  ;(not (wm-fact (key domain fact rs-ring-spec args? m ?ring-station r ?other-ring-color 
  ;      rn ?other-ring-base-req&~NA&:(> (sym-to-int ?other-ring-base-req) (sym-to-int ?ring-base-req)))))
  ;(not (wm-fact (key domain fact rs-filled-with args? m ?ring-station n ?ring-base-req)))
  (wm-fact (key domain fact rs-filled-with args? m ?ring-station n ?filled&~NA&:(< (sym-to-int ?filled) 3)))

  ; check time
  (wm-fact (key refbox game-time) (values ?game-time&:(< ?game-time 720) $?))

  ; don't formulate goal if it can't be executed immediatley
  (not (goal (acquired-resources ?ring-station)))
  =>
  ; create goals to add 1 base to the the ring station
  (assert (goal (id (sym-cat FILL-BASE-IN-RS-(gensym*)))
                  (class FILL-BASE-IN-RS)
                  (sub-type RUN-ALL-OF-SUBGOALS)
                  (required-resources ?ring-station)
                  (params bs ?base-station rs ?ring-station)
                  (meta global-priority -10)
          )
  ) 
)


; ============================= Robot selection ===============================

(defrule assign-robot-to-production-goal
  "Select a non-busy robot for executing a production leaf goal without assigned robot."
  ?g <- (goal (id ?goal-id) (class ?class) (params $?params) (mode SELECTED)
              (meta $? global-priority ?gprio $?)
        )
  
  ; goal class requires a fresh robot
  (test (goal-needs-fresh-robot ?class))

  ; there is no robot already assigned
  (not (test (member$ robot $?params)))

  ; check that there isn't a goal with higher priority waiting
  (not (goal (class ?oclass&:(goal-needs-fresh-robot ?oclass)) (mode SELECTED)
            (params $?oparams&:(not (member$ robot $?oparams))) 
            (meta $? global-priority ?ogprio&:(> ?ogprio ?gprio) $?)
       )
  )

  ; Get robot that is not assigned and not holding anything
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (goal (params robot ?robot $?some-params)))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?some-wp)))
  =>
  (printout t "Assigning " ?robot " to " ?goal-id crlf)
  (modify ?g (params robot ?robot $?params))
)

(defrule assign-robot-holding-wp
  "Assign a robot that is holding a wp to the correct goal for that wp."
  ?g <- (goal (id ?goal-id) (class ?class) (mode SELECTED)
              (params $?params)
        )
  ; goal class requires some robot to be holding the wp
  (test (goal-needs-robot-holding-wp ?class))

  ; there is currently no robot assigned to the goal
  (not (test (member$ robot $?params)))

  ; get the robot holding the wp required for the goal
  (wm-fact (key domain fact holding args? r ?robot wp ?wp&:(member$ ?wp $?params)))

  ; robot is not assigned to a goal already
  (not (goal (params robot ?robot $?some-params)))
  =>
  (printout t "Assigning " ?robot " to " ?goal-id crlf)
  (modify ?g (params robot ?robot $?params))
)


; ============================= Failed goals ===============================
; In some cases when a goal fails we don't have to completely start over

(defrule restart-fill-bases
" Goals of type FILL-BASES-IN-RS can be restarted by setting to SELECTED, since the
  number of bases will be recounted
"
  (declare (salience ?*SALIENCE-GOAL-PRE-EVALUATE*))
  ?g <- (goal (id ?goal-id) (class FILL-BASES-IN-RS) (mode FINISHED) (outcome FAILED)
              (meta $?pre retries ?retries&:(< ?retries ?*GOAL-MAX-TRIES*) $?post)
        )
  =>
  (printout t "Restarting " ?goal-id " for the " (+ ?retries 1) " time " crlf)
  (modify ?g (mode SELECTED) (meta $?pre retries (+ ?retries 1) $?post)
             (outcome UNKNOWN) (error))
)

(defrule fill-base-in-rs-failed-drop-wp
" If the filling of a rs failed we just drop the wp, because most likely a wp-put action
  already failed 3 times.
"
  (declare (salience ?*SALIENCE-GOAL-PRE-EVALUATE*))
  ?g <- (goal (id ?goal-id) (class FILL-BASE-IN-RS) (mode FINISHED) (outcome FAILED))
  (goal (parent ?goal-id) (params $? wp ?wp))
  (wm-fact (key domain fact holding args? r ?robot wp ?wp))
  (not (goal (class DROP-WP) (params robot ?robot wp ?wp)))
  =>
  (printout t ?robot " dropping wp because " ?goal-id " failed" crlf)
  (assert (goal (id (sym-cat DROP-WP-(gensym*))) (class DROP-WP) (sub-type SIMPLE)
                (params robot ?robot wp ?wp)
          )
  )  
)

(defrule clean-bs-after-failed
" If a robot fails at picking up a base from the base station, we want to discard the base.
"
  (declare (salience ?*SALIENCE-HIGHEST*))
  ; get base station
  (wm-fact (key domain fact mps-type args? m ?base-station t BS))
  (wm-fact (key domain fact mps-team args? m ?base-station col ?team-color))
  (wm-fact (key domain fact wp-at args? wp ?wp m ?base-station side ?side))

  ; if no robot is getting a base, there shouldn't be a base at the output
  (not (goal (class GET-BASE) (mode EXPANDED|DISPATCHED)))
  (not (goal (class CLEAN-BS)))

  ; bases that act as wp of a production goal are handed separately
  (not (goal (class PRODUCE-C0|PRODUCE-CX) (params $? wp ?wp $?)))
  =>
  (assert (goal (id (sym-cat CLEAN-BS-(gensym*))) (class CLEAN-BS) 
                (sub-type RUN-ALL-OF-SUBGOALS) 
                (params wp ?wp)
                (meta global-priority 2000)
          )
  )
)

(defrule clean-bs-after-failed-create-subgoals
  "Create subgoals to remove a discarded base at the base station."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?p <- (goal (id ?parent) (class CLEAN-BS) (params wp ?wp)
              (mode SELECTED))
  =>
  (assert 
    (goal (id (sym-cat PICKUP-WP-(gensym*))) 
          (class PICKUP-WP)
          (sub-type SIMPLE)
          (priority 1.0)
          (parent ?parent)
          (params wp ?wp)
    )
    (goal (id (sym-cat DROP-WP-(gensym*)))
          (class DROP-WP)
          (sub-type SIMPLE)
          (priority 0.0)
          (parent ?parent)
          (params wp ?wp)
    )
  )
  (modify ?p (mode EXPANDED))
)


(defrule recover-production-restart-root
" If a production goal fails try restart it a number of times and try to reuse the wp.
"
  (declare (salience ?*SALIENCE-GOAL-SELECT*))
  ?g <- (goal (class PRODUCE-C0|PRODUCE-CX) (mode FINISHED) (outcome FAILED)
              (meta $?m1 retries ?retries&:(< ?retries ?*GOAL-MAX-TRIES*) $?m2)
              (params $? wp ?wp $?)
        )
  =>
  (modify ?g (mode SELECTED) (outcome UNKNOWN) (error) (meta $?m1 retries (+ ?retries 1) $?m2))
)

(defrule pickup-wp-of-failed-production
" If a production goal fails, pick up the wp to either discard it or recover
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?g <- (goal (class PRODUCE-C0|PRODUCE-CX) (mode FINISHED) (outcome FAILED)
              (params $? wp ?wp $?)
        )

  ; wp of failed production at some station
  (wm-fact (key domain fact wp-at args? wp ?wp m ? side ?))

  ; wp not being picked up already
  (not (goal (class PICKUP-WP) (params wp ?wp)))
  =>
  (assert (goal (id (sym-cat PICKUP-WP-(gensym*))) (class PICKUP-WP)
                (sub-type SIMPLE) (params wp ?wp)))
)

(defrule recover-production-delete-subgoals
" Delete formulated subgoals of failed production goals
  so that they can be reformulated.
"
  (declare (salience ?*SALIENCE-HIGHEST*))
  (goal (id ?parent) (class PRODUCE-C0|PRODUCE-CX) (mode FINISHED) (outcome FAILED)
        (meta $?m1 retries ?retries&:(< ?retries ?*GOAL-MAX-TRIES*) $?m2)
  )
  ; child of failed root
  ?g <- (goal (id ?id) (parent ?parent) (mode FORMULATED))
  =>
  (printout t "Retracting goal " ?id crlf)
  (retract ?g)
)

