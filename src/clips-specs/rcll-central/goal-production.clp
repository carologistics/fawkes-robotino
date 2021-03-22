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
?*PRIORITY-C0* = 60
?*PRIORITY-C1* = 70
?*PRIORITY-C2* = 80
?*PRIORITY-C3* = 90
?*PRIORITY-SUPPORTING-TASKS* = 1
?*PRIORITY-REFILL-SHELF* = 2
?*PRIORITY-GO-WAIT* = 1
; MAX-RUNNING-TASKS can be useful for debugging purposes
?*MAX-RUNNING-TASKS* = 20
;?*GET-BASE-PRIO-DIFF* = 1
;?*BUFFER-CS-PRIO-DIFF* = 2
;?*BUFFER-RS-PRIO-DIFF* = 0
;?*DELIVER-PRIO-DIFF* = 5
;?*MOUNT-RING-PRIO-DIFF* = 2 
)

;(defrule goal-production-create-beacon-maintain
;" The parent goal for beacon signals. Allows formulation of
;  goals that periodically communicate with the refbox.
;"
;  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
;  (not (goal (class BEACON-MAINTAIN)))
;  (or (domain-facts-loaded)
;      (wm-fact (key refbox phase) (value ~SETUP&~PRE_GAME)))
;  =>
;  (bind ?goal (goal-tree-assert-run-endless BEACON-MAINTAIN 1))
;  (modify ?goal (verbosity QUIET) (params frequency 1))
;)


;(defrule goal-production-create-beacon-achieve
;" Send a beacon signal whenever at least one second has elapsed since it
;  last one got sent.
;"
;  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
;  (time $?now)
;  ?g <- (goal (id ?maintain-id) (class BEACON-MAINTAIN) (mode SELECTED))
;  ; TODO: make interval a constant
;  =>
;  (assert (goal (id (sym-cat SEND-BEACON- (gensym*))) (sub-type SIMPLE)
;                (class SEND-BEACON) (parent ?maintain-id) (verbosity QUIET)))
;)

(deftemplate color-assignment
  (slot cs(type SYMBOL))
  (slot color(type SYMBOL))
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

  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-team args? m ?cs1 col ?team-color))
  (wm-fact (key domain fact mps-type args? m ?cs1 t CS))

  (wm-fact (key domain fact mps-team args? m ?cs2 col ?team-color))
  (wm-fact (key domain fact mps-type args? m ?cs2 t CS))

  (test (not(eq ?cs1 ?cs2)))

  (wm-fact (key domain fact wp-on-shelf args? wp ?wp1 m ?cs1 spot ?spot1))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp1 col ?col1))
  (wm-fact (key domain fact wp-on-shelf args? wp ?wp2 m ?cs2 spot ?spot2))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp2 col ?col2))


  =>
  (assert (color-assignment (cs ?cs1) (color ?col1)))
  (assert (color-assignment (cs ?cs2) (color ?col2)))
  (printout t "Assigned " ?cs1 " to color " ?col1 crlf)
  (printout t "Assigned " ?cs2 " to color " ?col2 crlf)
  (printout t "Formulated Refill Shelf Goal" crlf)

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
  (color-assignment (cs ?mps) (color ?col))
  =>
  (printout t "Formulated Refill Shelf Goal" crlf)
  (assert (goal (id (sym-cat REFILL-SHELF- (gensym*)))
                (class REFILL-SHELF) (sub-type SIMPLE)
                (parent ?maintain-id) (verbosity QUIET)
                (params mps ?mps color ?col)))
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
  (wm-fact (key central agent robot args? r ?robot))
  (wm-fact (key domain fact robot-waiting args? r ?robot))
  (not (goal (class ENTER-FIELD)))
  (wm-fact (key refbox state) (value RUNNING))
  (wm-fact (key refbox phase) (value PRODUCTION|EXPLORATION))
  (wm-fact (key navgraph waitzone generated) (type BOOL) (value TRUE))
  (wm-fact (key refbox team-color) (value ?team-color))
  (NavGraphGeneratorInterface (final TRUE))
  (not (wm-fact (key domain fact entered-field args? r ?robot)))
  =>
  (printout t "Goal " ENTER-FIELD " formulated" crlf)
  (assert (goal (id (sym-cat ENTER-FIELD- (gensym*)))
                (class ENTER-FIELD) (sub-type SIMPLE)
                (params r ?robot team-color ?team-color)))
)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;
;CParent Production
;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule goal-production-create-produce-cparent
(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
(wm-fact (key domain fact entered-field args? r ?some-robot))
(not (goal (class PRODUCE-CPARENT)))
=>
 (printout t "CParent formulated" crlf)
 (bind ?wp (sym-cat WP- (random-id)))
 (assert (goal (id (sym-cat PRODUCE-CPARENT- (gensym*))) (mode EXPANDED)
               (class PRODUCE-CPARENT)(sub-type RUN-SUBGOALS-ON-IDLE))
)
)

(defrule init-running-tasks
(not (running-tasks (number ?some-number)))
=>
 (printout t "running tasks initialized" crlf)
 (assert (running-tasks (number 0)))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;
;C0 Production
;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;




(defrule goal-production-create-produce-c0
" Produce a C0 product: Get the correct base and mount the right cap on it.
"
(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
(wm-fact (key domain fact order-complexity args? ord ?order com C0))
(wm-fact (key refbox order ?order delivery-begin) (value ?delivery-begin))

(wm-fact (key refbox game-time) (values $?game-time))

(not(wm-fact (key domain fact order-fulfilled args? ord ?order)))
(not (goal (class PRODUCE-C0)(params order ?order)))
(goal (id ?produce-cparent-id) (class PRODUCE-CPARENT))

?r <- (running-tasks (number ?running-tasks))
(test (< ?running-tasks ?*MAX-RUNNING-TASKS*))
 =>
 (printout t "Goal for C0 order " ?order " formulated." crlf)
 (assert (running-tasks (number (+ ?running-tasks 1))))
 (retract ?r)
 (printout t "running tasks increased to "  (+ ?running-tasks 1) crlf)
 (assert (goal (id (sym-cat PRODUCE-C0- (gensym*)))
               (class PRODUCE-C0)(sub-type RUN-SUBGOALS-IN-PARALLEL)(parent ?produce-cparent-id)(meta delivery-begin ?delivery-begin game-time (nth$ 1 ?game-time)) (priority ?*PRIORITY-C0*) (mode FORMULATED)
               (params order ?order)
 ))
)

(defrule goal-produce-c0-get-base
  "get a base for c0-production and wait at the cap-station"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c0-id) (class PRODUCE-C0) (mode SELECTED) (params order ?order))
  (not (goal (class GET-BASE) (parent ?produce-c0-id)))
  =>
  (printout t "Goal GET-BASE-WAIT formulated" crlf)
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c0-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
)

(defrule goal-produce-c0-buffer-cs
  "feed a cap into the a cap station for c0-production"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c0-id) (class PRODUCE-C0) (mode SELECTED) (params order ?order))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
  (not (goal (class BUFFER-CS) (parent ?produce-c0-id)))
  =>
  (printout t "Goal BUFFER-CS formulated" crlf)
  (assert (goal (id (sym-cat GET-CC- (gensym*))) (class GET-CC) (parent ?produce-c0-id) (sub-type SIMPLE) (mode FORMULATED) (params cs-color ?cap-color cs DUMMY)))
  (assert (goal (id (sym-cat BUFFER-CS- (gensym*))) (class BUFFER-CS) (parent ?produce-c0-id) (sub-type SIMPLE) (mode FORMULATED) (params cs-color ?cap-color cs DUMMY)))
)

(defrule goal-produce-c0-mount-cap-deliver
  "mount the cap for c0-production and and deliver the product"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c0-id) (class PRODUCE-C0) (mode SELECTED) (params order ?order))
  (not (goal (class MOUNT-CAP) (parent ?produce-c0-id)))
  (not (goal (class DELIVER) (parent ?produce-c0-id)))
  =>
  (printout t "Goal MOUNT-CAP and DELIVER formulated" crlf)
  (assert (goal (id (sym-cat DELIVER- (gensym*))) (class DELIVER) (parent ?produce-c0-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order)))
  (assert (goal (id (sym-cat MOUNT-CAP- (gensym*))) (class MOUNT-CAP) (parent ?produce-c0-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order cs DUMMY)))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;
; C1 Production
;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defrule goal-production-create-produce-c1
" Produce a C1 product: Get the correct base and mount the right ring and then a cap on it.
"
 (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
 (wm-fact (key domain fact order-complexity args? ord ?order com C1))
  (wm-fact (key refbox order ?order delivery-begin) (value ?delivery-begin))

(wm-fact (key refbox game-time) (values $?game-time))

(not(wm-fact (key domain fact order-fulfilled args? ord ?order)))
(goal (id ?produce-cparent-id) (class PRODUCE-CPARENT))

(not (goal (class PRODUCE-C1) (params order ?order)))
 ?r <- (running-tasks (number ?running-tasks))
(test (< ?running-tasks ?*MAX-RUNNING-TASKS*))
 =>
 (bind ?wp (sym-cat WP- (random-id)))
 (printout t "Goal for C1 order " ?order " formulated." crlf)
(assert (running-tasks (number (+ ?running-tasks 1))))
 (retract ?r)
 (printout t "running tasks increased to "  (+ ?running-tasks 1) crlf)
 (assert (goal (id (sym-cat PRODUCE-C1- (gensym*)))
               (class PRODUCE-C1)(sub-type RUN-SUBGOALS-IN-PARALLEL)(meta delivery-begin ?delivery-begin game-time (nth$ 1 ?game-time)) (priority ?*PRIORITY-C1*)(parent ?produce-cparent-id) (mode FORMULATED)
               (params order ?order)
 ))
)

(defrule goal-produce-c1-get-base
  "get a base for c1-production and wait at the ring-station"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c1-id) (class PRODUCE-C1) (mode SELECTED) (params order ?order))
  (not (goal (class GET-BASE) (parent ?produce-c1-id)))

  =>
  (printout t "Goal GET-BASE formulated" crlf)
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c1-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
)

(defrule goal-produce-c1-buffer-cs
  "feed a cap into the cap station for c1-production"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c1-id) (class PRODUCE-C1) (mode SELECTED) (params order ?order))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
  (not (goal (class BUFFER-CS) (parent ?produce-c1-id)))
  =>
  (printout t "Goal BUFFER-CS formulated" crlf)
  (assert (goal (id (sym-cat GET-CC- (gensym*))) (class GET-CC) (parent ?produce-c1-id) (sub-type SIMPLE) (mode FORMULATED) (params cs-color ?cap-color cs DUMMY)))
  (assert (goal (id (sym-cat BUFFER-CS- (gensym*))) (class BUFFER-CS) (parent ?produce-c1-id) (sub-type SIMPLE) (mode FORMULATED) (params cs-color ?cap-color cs DUMMY)))
)

(defrule goal-produce-c1-mount-rings
  "mount rings for a c1-production and buffer rs appropriately"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c1-id) (class PRODUCE-C1) (mode SELECTED) (params order ?order))

  (wm-fact (key refbox team-color) (value ?team-color))

  ;; get order information
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))

  ;; find fitting ring stations
  (wm-fact (key domain fact mps-type args? m ?rs1 t RS))
  (wm-fact (key domain fact mps-team args? m ?rs1 col ?team-color))
  (wm-fact (key domain fact rs-ring-spec args? m ?rs1 r ?ring1-color rn ?bases-needed1))

  (not (goal (class MOUNT-RING) (parent ?produce-c2-id)))
  =>
  (printout t "MOUNT-RING goals formulated " ?bases-needed1 crlf)
  (assert (goal (id (sym-cat MOUNT-RING- (gensym*))) (class MOUNT-RING) (parent ?produce-c1-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order rs ?rs1 ring-num ONE)))
  (if (eq ?bases-needed1 ONE) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c1-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs1)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c1-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  )
  (if (eq ?bases-needed1 TWO) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c1-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs1)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c1-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs1)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c1-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c1-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  )
  (if (eq ?bases-needed1 THREE) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c1-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs1)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c1-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs1)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c1-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs1)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c1-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c1-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c1-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  )
)

(defrule goal-produce-c1-mount-cap-deliver
  "mount the cap for c1-production and and deliver the product"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c1-id) (class PRODUCE-C1) (mode SELECTED) (params order ?order))
  (not (goal (class MOUNT-CAP) (parent ?produce-c1-id)))
  (not (goal (class DELIVER) (parent ?produce-c1-id)))
  =>
  (printout t "Goal MOUNT-CAP and DELIVER formulated" crlf)
  (assert (goal (id (sym-cat DELIVER- (gensym*))) (class DELIVER) (parent ?produce-c1-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order)))
  (assert (goal (id (sym-cat MOUNT-CAP- (gensym*))) (class MOUNT-CAP) (parent ?produce-c1-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order cs DUMMY)))
)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;
; C2 Production
;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defrule goal-production-create-produce-c2
" Produce a C2 product: Get the correct base and mount the right rings and then a cap on it.
"
 (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
 (wm-fact (key domain fact order-complexity args? ord ?order com C2))
   (wm-fact (key refbox order ?order delivery-begin) (value ?delivery-begin))

(wm-fact (key refbox game-time) (values $?game-time))

(not(wm-fact (key domain fact order-fulfilled args? ord ?order)))
(goal (id ?produce-cparent-id) (class PRODUCE-CPARENT))
(not (goal (class PRODUCE-C2) (params order ?order)))
 ?r <- (running-tasks (number ?running-tasks))
(test (< ?running-tasks ?*MAX-RUNNING-TASKS*))
 =>
 (bind ?wp (sym-cat WP- (random-id)))
 (printout t "Goal for C2 order " ?order " formulated." crlf)
 (assert (running-tasks (number (+ ?running-tasks 1))))
 (retract ?r)
 (printout t "running tasks increased to "  (+ ?running-tasks 1) crlf)
 (assert (goal (id (sym-cat PRODUCE-C2- (gensym*)))
               (class PRODUCE-C2)(sub-type RUN-SUBGOALS-IN-PARALLEL)(meta delivery-begin ?delivery-begin game-time (nth$ 1 ?game-time)) (priority ?*PRIORITY-C2*)(parent ?produce-cparent-id) (mode FORMULATED)
               (params order ?order)
 ))
)

(defrule goal-produce-c2-get-base
  "get a base for c2-production and wait at the ring-station"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c2-id) (class PRODUCE-C2) (mode SELECTED) (params order ?order))
  (not (goal (class GET-BASE) (parent ?produce-c2-id)))

  =>
  (printout t "Goal GET-BASE formulated" crlf)
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
)

(defrule goal-produce-c2-buffer-cs
  "feed a cap into the cap station for c2-production"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c2-id) (class PRODUCE-C2) (mode SELECTED) (params order ?order))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
  (not (goal (class BUFFER-CS) (parent ?produce-c2-id)))
  =>
  (printout t "Goal BUFFER-CS formulated" crlf)
  (assert (goal (id (sym-cat GET-CC- (gensym*))) (class GET-CC) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params cs-color ?cap-color cs DUMMY)))
  (assert (goal (id (sym-cat BUFFER-CS- (gensym*))) (class BUFFER-CS) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params cs-color ?cap-color cs DUMMY)))
)

(defrule goal-produce-c2-mount-rings
  "mount rings for a c2-production and buffer rs appropriately"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c2-id) (class PRODUCE-C2) (mode SELECTED) (params order ?order))

  (wm-fact (key refbox team-color) (value ?team-color))

  ;; get order information
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?ring2-color))

  ;; find fitting ring stations
  (wm-fact (key domain fact mps-type args? m ?rs1 t RS))
  (wm-fact (key domain fact mps-team args? m ?rs1 col ?team-color))
  (wm-fact (key domain fact rs-ring-spec args? m ?rs1 r ?ring1-color rn ?bases-needed1))

  (wm-fact (key domain fact mps-type args? m ?rs2 t RS))
  (wm-fact (key domain fact mps-team args? m ?rs2 col ?team-color))
  (wm-fact (key domain fact rs-ring-spec args? m ?rs2 r ?ring2-color rn ?bases-needed2))

  (not (goal (class MOUNT-RING) (parent ?produce-c2-id)))
  =>
  (printout t "MOUNT-RING goals formulated " ?bases-needed1 " " ?bases-needed2 crlf)
  (assert (goal (id (sym-cat MOUNT-RING- (gensym*))) (class MOUNT-RING) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order rs ?rs1 ring-num ONE)))
  (assert (goal (id (sym-cat MOUNT-RING- (gensym*))) (class MOUNT-RING) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order rs ?rs2 ring-num TWO)))
  (if (eq ?bases-needed1 ONE) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs1)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  )
  (if (eq ?bases-needed1 TWO) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs1)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs1)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  )
  (if (eq ?bases-needed1 THREE) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs1)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs1)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs1)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  )
  (if (eq ?bases-needed2 ONE) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs2)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  )
  (if (eq ?bases-needed2 TWO) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs2)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs2)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  )
  (if (eq ?bases-needed2 THREE) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs2)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs2)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs2)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  )
)

(defrule goal-produce-c2-mount-cap-deliver
  "mount the cap for c2-production and and deliver the product"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c2-id) (class PRODUCE-C2) (mode SELECTED) (params order ?order))
  (not (goal (class MOUNT-CAP) (parent ?produce-c2-id)))
  (not (goal (class DELIVER) (parent ?produce-c2-id)))
  =>
  (printout t "Goal MOUNT-CAP and DELIVER formulated" crlf)
  (assert (goal (id (sym-cat DELIVER- (gensym*))) (class DELIVER) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order)))
  (assert (goal (id (sym-cat MOUNT-CAP- (gensym*))) (class MOUNT-CAP) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order cs DUMMY)))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;
; C3 Production
;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defrule goal-production-create-produce-c3
" Produce a C3 product: Get the correct base and mount the right rings and then a cap on it.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (wm-fact (key domain fact order-complexity args? ord ?order com C3))
  (wm-fact (key refbox order ?order delivery-begin) (value ?delivery-begin))

 (wm-fact (key refbox game-time) (values $?game-time))

 (not(wm-fact (key domain fact order-fulfilled args? ord ?order)))
 (goal (id ?produce-cparent-id) (class PRODUCE-CPARENT))

 (not (goal (class PRODUCE-C3) (params order ?order)))
 ?r <- (running-tasks (number ?running-tasks))
(test (< ?running-tasks ?*MAX-RUNNING-TASKS*))
  =>
  (bind ?wp (sym-cat WP- (random-id)))
  (printout t "Goal for C3 order " ?order " formulated." crlf)
  (assert (running-tasks (number (+ ?running-tasks 1))))
  (retract ?r)
  (printout t "running tasks increased to "  (+ ?running-tasks 1) crlf)
  (assert (goal (id (sym-cat PRODUCE-C3- (gensym*)))
                (class PRODUCE-C3)(sub-type RUN-SUBGOALS-IN-PARALLEL)(meta delivery-begin ?delivery-begin game-time (nth$ 1 ?game-time)) (priority ?*PRIORITY-C3*)(parent ?produce-cparent-id) (mode FORMULATED)
                (params order ?order)
  ))
)

(defrule goal-produce-c3-get-base
  "get a base for c3-production and wait at the ring-station"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c3-id) (class PRODUCE-C3) (mode SELECTED) (params order ?order))
  (not (goal (class GET-BASE) (parent ?produce-c3-id)))
  =>
  (printout t "Goal GET-BASE formulated" crlf)
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
)

(defrule goal-produce-c3-buffer-cs
  "feed a cap into the cap station for c3-production"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c3-id) (class PRODUCE-C3) (mode SELECTED) (params order ?order))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
  (not (goal (class BUFFER-CS) (parent ?produce-c3-id)))
  =>
  (printout t "Goal BUFFER-CS formulated" crlf)
  (assert (goal (id (sym-cat GET-CC- (gensym*))) (class GET-CC) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params cs-color ?cap-color cs DUMMY)))
  (assert (goal (id (sym-cat BUFFER-CS- (gensym*))) (class BUFFER-CS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params cs-color ?cap-color cs DUMMY)))
)

(defrule goal-produce-c3-mount-rings
  "mount rings for a c3-production and buffer rs appropriately"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c3-id) (class PRODUCE-C3) (mode SELECTED) (params order ?order))

  (wm-fact (key refbox team-color) (value ?team-color))

  ;; get order information
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?ring2-color))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?ring3-color))

  ;; find fitting ring stations
  (wm-fact (key domain fact mps-type args? m ?rs1 t RS))
  (wm-fact (key domain fact mps-team args? m ?rs1 col ?team-color))
  (wm-fact (key domain fact rs-ring-spec args? m ?rs1 r ?ring1-color rn ?bases-needed1))

  (wm-fact (key domain fact mps-type args? m ?rs2 t RS))
  (wm-fact (key domain fact mps-team args? m ?rs2 col ?team-color))
  (wm-fact (key domain fact rs-ring-spec args? m ?rs2 r ?ring2-color rn ?bases-needed2))

  (wm-fact (key domain fact mps-type args? m ?rs3 t RS))
  (wm-fact (key domain fact mps-team args? m ?rs3 col ?team-color))
  (wm-fact (key domain fact rs-ring-spec args? m ?rs3 r ?ring3-color rn ?bases-needed3))

  (not (goal (class MOUNT-RING) (parent ?produce-c3-id)))
  =>
  (printout t "MOUNT-RING goals formulated " ?bases-needed1 " " ?bases-needed2 " " ?bases-needed3 crlf)
  (assert (goal (id (sym-cat MOUNT-RING- (gensym*))) (class MOUNT-RING) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order rs ?rs1 ring-num ONE)))
  (assert (goal (id (sym-cat MOUNT-RING- (gensym*))) (class MOUNT-RING) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order rs ?rs2 ring-num TWO)))
  (assert (goal (id (sym-cat MOUNT-RING- (gensym*))) (class MOUNT-RING) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order rs ?rs3 ring-num THREE)))
  (if (eq ?bases-needed1 ONE) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs1)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  )
  (if (eq ?bases-needed1 TWO) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs1)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs1)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  )
  (if (eq ?bases-needed1 THREE) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs1)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs1)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs1)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  )
  (if (eq ?bases-needed2 ONE) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs2)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  )
  (if (eq ?bases-needed2 TWO) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs2)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs2)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  )
  (if (eq ?bases-needed2 THREE) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs2)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs2)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs2)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  )
  (if (eq ?bases-needed3 ONE) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs3)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  )
  (if (eq ?bases-needed3 TWO) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs3)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs3)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  )
  (if (eq ?bases-needed3 THREE) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs3)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs3)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params rs ?rs3)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  (assert (goal (id (sym-cat GET-BASE- (gensym*))) (class GET-BASE) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order bs DUMMY)))
  )
)

(defrule goal-produce-c3-mount-cap-deliver
  "mount the cap for c3-production and and deliver the product"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c3-id) (class PRODUCE-C3) (mode SELECTED) (params order ?order))
  (not (goal (class MOUNT-CAP) (parent ?produce-c3-id)))
  (not (goal (class DELIVER) (parent ?produce-c3-id)))
  =>
  (printout t "Goal MOUNT-CAP and DELIVER formulated" crlf)
  (assert (goal (id (sym-cat DELIVER- (gensym*))) (class DELIVER) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order)))
  (assert (goal (id (sym-cat MOUNT-CAP- (gensym*))) (class MOUNT-CAP) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params order ?order cs DUMMY)))
)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;
; Supporting TASKS
;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;
;(defrule goal-production-create-get-base-to-fill-rs
;  "Fill the ring station with a fresh base from the base station."
;  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
;  (goal (id ?produce-cparent-id) (class PRODUCE-CPARENT) (mode SELECTED))
;  (wm-fact (key refbox team-color) (value ?team-color))
;  ;Robot CEs
;  (wm-fact (key domain fact self args? r ?robot))
;  (wm-fact (key domain fact wp-spawned-for args? wp ?spawned-wp r ?robot))
;  (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
;  ;MPS-RS CEs (a cap carrier can be used to fill a RS later)
;  (wm-fact (key domain fact mps-type args? m ?mps t RS))
;  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
;  (wm-fact (key domain fact rs-filled-with args? m ?mps n ?rs-before&ZERO|ONE|TWO))
;  ;MPS-BS CEs
;  (wm-fact (key domain fact mps-type args? m ?bs t BS))
;  (wm-fact (key domain fact mps-state args? m ?bs s ~BROKEN&~DOWN))
;  (wm-fact (key domain fact mps-team args? m ?bs col ?team-color))
;  (domain-object (name ?bs-side&:(or (eq ?bs-side INPUT) (eq ?bs-side OUTPUT))) (type mps-side))
;
;  (wm-fact (key domain fact order-base-color args? ord ?any-order col ?base-color))
;  (not (goal (class GET-BASE-TO-FILL-RS) (params robot ?robot
;                                          bs ?bs
;                                          bs-side ?bs-side
;                                          base-color ?
;                                          wp ?spawned-wp)))
;  =>
;  (printout t "Goal " GET-BASE-TO-FILL-RS " formulated" crlf)
;  (bind ?distance (node-distance (str-cat ?bs - (if (eq ?bs-side INPUT) then I else O))))
;  (assert (goal (id (sym-cat GET-BASE-TO-FILL-RS- (gensym*)))
;                (class GET-BASE-TO-FILL-RS)
;                (parent ?produce-cparent-id) (sub-type SIMPLE)
;                             (params robot ?robot
;                                     bs ?bs
;                                     bs-side ?bs-side
;                                     base-color ?base-color
;                                     wp ?spawned-wp
;                                     )
;                            (required-resources ?spawned-wp)
;  ))
;)
;
;(defrule goal-production-create-discard-wp
;  "Discard a base which is not needed."
;  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
;
;  (goal (id ?produce-cparent-id) (class PRODUCE-CPARENT) (mode SELECTED))
;  (wm-fact (key refbox team-color) (value ?team-color))
;  (wm-fact (key domain fact self args? r ?robot))
;  (wm-fact (key domain fact holding args? r ?robot wp ?wp))
;  =>
;  (printout t "Goal " DISCARD-WP " formulated" crlf)
;  (assert (goal (id (sym-cat DISCARD-WP- (gensym*)))
;                (class DISCARD-WP) (sub-type SIMPLE)
;                (parent ?produce-cparent-id)
;                (params robot ?robot
;                        wp ?wp
;                )
;                (required-resources ?wp)
;  ))
;)
;
;
;(defrule goal-produce-supporting-tasks
;" Unused robots can be assigned to general tasks such as refill shelf
;"
; (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
; (wm-fact (key domain fact entered-field args? r ?some-robot))
;  (not (goal (class GO-SUPPORTING-TASKS)))
; =>
; (printout t "Goal for supporting tasks formulated "crlf)
; (bind ?wp (sym-cat WP- (random-id)))
; (assert (goal (id (sym-cat PRODUCE-SUPPORTING-TASKS- (gensym*)))
;               (class SUPPORTING-TASKS)(sub-type RUN-SUBGOALS-IN-PARALLEL)(priority ?*PRIORITY-SUPPORTING-TASKS*)
; ))
;)
;
;
;(defrule goal-production-create-go-wait
;  "Drive to a waiting position and wait there."
;  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
;  (wm-fact (key domain fact entered-field args? r ?robot))
;  (wm-fact (key domain fact self args? r ?self))
;  (goal (id ?produce-sp-id) (class PRODUCE-SUPPORTING-TASKS) (mode SELECTED))
;
;  (not (goal (class GO-WAIT)))
;  =>
;  (printout t "Goal " GO-WAIT " formulated" crlf)
;  (assert (goal (id (sym-cat GO-WAIT- (gensym*)))
;                (parent ?produce-sp-id) 
;                (class GO-WAIT) (sub-type SIMPLE)
;                (priority  ?*PRIORITY-GO-WAIT*)
;  ))
;)
