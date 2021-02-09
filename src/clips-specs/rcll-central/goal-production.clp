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
?*PRIORITY-C0* = 6
?*PRIORITY-C1* = 7
?*PRIORITY-C2* = 8
?*PRIORITY-C3* = 9
?*PRIORITY-SUPPORTING-TASKS* = 1
?*PRIORITY-REFILL-SHELF* = 2
?*PRIORITY-GO-WAIT* = 1
?*FIRST-GOAL* = 1
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
;C0 Production
;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;




(defrule goal-production-create-produce-c0
" Produce a C0 product: Get the correct base and mount the right cap on it.
"
 (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
 (wm-fact (key domain fact order-complexity args? ord ?order com C0))
 (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
 (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))

(wm-fact (key refbox team-color) (value ?team-color))

(wm-fact (key domain fact mps-type args? m ?cs t CS))
(wm-fact (key domain fact mps-team args? m ?cs col ?team-color))
(wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?cs spot ?shelf-spot))
(wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))

(wm-fact (key domain fact mps-type args? m ?bs t BS))
(wm-fact (key domain fact mps-team args? m ?bs col ?team-color))

(wm-fact (key domain fact mps-type args? m ?ds t DS))
(wm-fact (key domain fact mps-team args? m ?ds col ?team-color))

(not(wm-fact (key domain fact order-fulfilled args? ord ?order)))
(not (goal (class PRODUCE-C0)))
(not (goal (class PRODUCE-C1)))
(not (goal (class PRODUCE-C2)))
(not (goal (class PRODUCE-C3)))
 ;params order ?order bs-color ?any-base-color cs-color ?any-cap-color wp ?any-wp bs ?any-bs cs ?any-cs ds ?any-ds
 =>
 (printout t "Goal for C0 order " ?order " formulated: " ?base-color " " ?cap-color crlf)
 (bind ?wp (sym-cat WP- (random-id)))
 (assert (goal (id (sym-cat PRODUCE-C0- (gensym*)))
               (class PRODUCE-C0)(sub-type RUN-SUBGOALS-IN-PARALLEL)(priority ?*PRIORITY-C0*)
               (params order ?order bs-color ?base-color cs-color ?cap-color wp ?wp bs ?bs cs ?cs ds ?ds)
 ))
)

(defrule goal-produce-c0-get-base-wait
  "get a base for c0-production and wait at the cap-station"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c0-id) (class PRODUCE-C0) (mode SELECTED) (params order ?order bs-color ?base-color cs-color ?cap-color wp ?wp bs ?bs cs ?cs ds ?ds))
  (not (goal (class GET-BASE-WAIT) (parent ?produce-c0-id)))
  =>
  (printout t "Goal GET-BASE-WAIT formulated" crlf)
  (assert (goal (id (sym-cat GET-BASE-WAIT- (gensym*))) (class GET-BASE-WAIT) (parent ?produce-c0-id) (sub-type SIMPLE) (mode FORMULATED) (params bs-color ?base-color wp ?wp bs ?bs wait-pos ?cs wait-side INPUT)))
)

(defrule goal-produce-c0-buffer-cs
  "feed a cap into the a cap station for c0-production"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c0-id) (class PRODUCE-C0) (mode SELECTED) (params order ?order bs-color ?base-color cs-color ?cap-color wp ?wp bs ?bs cs ?cs ds ?ds))
  (not (goal (class BUFFER-CS) (parent ?produce-c0-id)))
  =>
  (printout t "Goal BUFFER-CS formulated" crlf)
  (assert (goal (id (sym-cat BUFFER-CS- (gensym*))) (class BUFFER-CS) (parent ?produce-c0-id) (sub-type SIMPLE) (mode FORMULATED) (params cs-color ?cap-color cs ?cs)))
)

(defrule goal-produce-c0-mount-cap-deliver
  "mount the cap for c0-production and and deliver the product"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c0-id) (class PRODUCE-C0) (mode SELECTED))
  (not (goal (class MOUNT-CAP-DELIVER) (parent ?produce-c0-id)))
  =>
  (printout t "Goal MOUNT-CAP-DELIVER formulated" crlf)
  (assert (goal (id (sym-cat MOUNT-CAP-DELIVER- (gensym*))) (class MOUNT-CAP-DELIVER) (parent ?produce-c0-id) (sub-type SIMPLE) (mode FORMULATED)))
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
 (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
 (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
 (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))

(wm-fact (key refbox team-color) (value ?team-color))
; wait for machines to be initialized. Thanks to Chris for this fix.
(wm-fact (key domain fact entered-field args? r ?some-robot))

(wm-fact (key domain fact mps-type args? m ?cs t CS))
(wm-fact (key domain fact mps-team args? m ?cs col ?team-color))
(wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?cs spot ?shelf-spot))
(wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))

(wm-fact (key domain fact mps-type args? m ?bs t BS))
(wm-fact (key domain fact mps-team args? m ?bs col ?team-color))

(wm-fact (key domain fact mps-type args? m ?rs1 t RS))
(wm-fact (key domain fact mps-team args? m ?rs1 col ?team-color))
(wm-fact (key domain fact rs-ring-spec args? m ?rs1 r ?ring1-color rn ?bases-needed))

(wm-fact (key domain fact mps-type args? m ?ds t DS))
(wm-fact (key domain fact mps-team args? m ?ds col ?team-color))

(not(wm-fact (key domain fact order-fulfilled args? ord ?order)))
(not (goal (class PRODUCE-C0)))
(not (goal (class PRODUCE-C1)))
(not (goal (class PRODUCE-C2)))
(not (goal (class PRODUCE-C3)))

 ;(params order ?order bs-color ?any-base-color ring1-color ?any-ring1-color cs-color ?any-cap-color wp ?any-wp bs ?any-bs cs ?any-cs ds ?any-ds rs1 ?any-rs)
 =>
 (printout t "Goal for C1 order " ?order " formulated: " ?base-color " " ?ring1-color " " ?cap-color crlf)
 (bind ?wp (sym-cat WP- (random-id)))
 (assert (goal (id (sym-cat PRODUCE-C1- (gensym*)))
               (class PRODUCE-C1)(sub-type RUN-SUBGOALS-IN-PARALLEL)(priority ?*PRIORITY-C1*)
               (params order ?order bs-color ?base-color ring1-color ?ring1-color cs-color ?cap-color wp ?wp bs ?bs cs ?cs ds ?ds rs1 ?rs1)
 ))
)

(defrule goal-produce-c1-get-base-wait
  "get a base for c1-production and wait at the ring-station"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c1-id) (class PRODUCE-C1) (mode SELECTED) (params order ?order bs-color ?base-color ring1-color ?ring1-color cs-color ?cap-color wp ?wp bs ?bs cs ?cs ds ?ds rs1 ?rs1))
  (not (goal (class GET-BASE-WAIT) (parent ?produce-c1-id)))
  =>
  (printout t "Goal GET-BASE-WAIT formulated" crlf)
  (assert (goal (id (sym-cat GET-BASE-WAIT- (gensym*))) (class GET-BASE-WAIT) (parent ?produce-c1-id) (sub-type SIMPLE) (mode FORMULATED) (params bs-color ?base-color wp ?wp bs ?bs wait-pos ?rs1 wait-side INPUT)))
)

(defrule goal-produce-c1-buffer-cs
  "feed a cap into the cap station for c1-production"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c1-id) (class PRODUCE-C1) (mode SELECTED) (params order ?order bs-color ?base-color ring1-color ?ring1-color cs-color ?cap-color wp ?wp bs ?bs cs ?cs ds ?ds rs1 ?rs1))
  (not (goal (class BUFFER-CS) (parent ?produce-c1-id)))
  =>
  (printout t "Goal BUFFER-CS formulated" crlf)
  (assert (goal (id (sym-cat BUFFER-CS- (gensym*))) (class BUFFER-CS) (parent ?produce-c1-id) (sub-type SIMPLE) (mode FORMULATED) (params cs-color ?cap-color cs ?cs)))
)

(defrule goal-produce-c1-buffer-rs1
  "feed a base into the ring station for c1-production"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c1-id) (class PRODUCE-C1) (mode SELECTED)
        (params order ?order bs-color ?base-color ring1-color ?ring1-color cs-color ?cap-color wp ?wp bs ?bs cs ?cs ds ?ds rs1 ?rs1)
  )
  (wm-fact (key domain fact rs-ring-spec args? m ?rs1 r ?ring1-color rn ?bases-needed1))
  
  (not (goal (class BUFFER-RS) (parent ?produce-c2-id) (params bs ?any-bs rs ?any-rs1 for ONE)))
  (not (goal (class BUFFER-RS) (parent ?produce-c2-id) (params robot ?any-robot bs ?any-bs rs ?any-rs1 for ONE)))
  =>
  (printout t "Goals BUFFER-RS formulated. bases-needed: " ?bases-needed1 crlf)
  (if (eq ?bases-needed1 ONE) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c1-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs1 for ONE)))
  )
  (if (eq ?bases-needed1 TWO) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c1-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs1 for ONE)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c1-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs1 for ONE)))
  )
  (if (eq ?bases-needed1 THREE) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c1-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs1 for ONE)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c1-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs1 for ONE)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c1-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs1 for ONE)))
  )
)

(defrule goal-produce-c1-mount-ring1
  "mount ring 1 for c1-production and wait at the cap-station"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c1-id) (class PRODUCE-C1) (mode SELECTED) (params order ?order bs-color ?base-color ring1-color ?ring1-color cs-color ?cap-color wp ?wp bs ?bs cs ?cs ds ?ds rs1 ?rs1))
  (not (goal (class MOUNT-RING) (parent ?produce-c1-id)))
  =>
  (printout t "Goal MOUNT-RING1 formulated" crlf)
  (assert (goal (id (sym-cat MOUNT-RING1- (gensym*))) (class MOUNT-RING1) (parent ?produce-c1-id) (sub-type SIMPLE) (mode FORMULATED) (params ring1-color ?ring1-color wp ?wp rs ?rs1 wait-pos ?cs wait-side INPUT)))
)

(defrule goal-produce-c1-mount-cap-deliver
  "mount the cap for c1-production and and deliver the product"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c1-id) (class PRODUCE-C1) (mode SELECTED))
  (not (goal (class MOUNT-CAP-DELIVER-C1) (parent ?produce-c1-id)))
  =>
  (printout t "Goal MOUNT-CAP-DELIVER formulated" crlf)
  (assert (goal (id (sym-cat MOUNT-CAP-DELIVER- (gensym*))) (class MOUNT-CAP-DELIVER-C1) (parent ?produce-c1-id) (sub-type SIMPLE) (mode FORMULATED)))
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
 (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
 (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
 (wm-fact (key domain fact order-ring2-color args? ord ?order col ?ring2-color))
 (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))

(wm-fact (key refbox team-color) (value ?team-color))
; wait for machines to be initialized. Thanks to Chris for this fix.
(wm-fact (key domain fact entered-field args? r ?some-robot))

(wm-fact (key domain fact mps-type args? m ?cs t CS))
(wm-fact (key domain fact mps-team args? m ?cs col ?team-color))
(wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?cs spot ?shelf-spot))
(wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))

(wm-fact (key domain fact mps-type args? m ?bs t BS))
(wm-fact (key domain fact mps-team args? m ?bs col ?team-color))

(wm-fact (key domain fact mps-type args? m ?rs1 t RS))
(wm-fact (key domain fact mps-team args? m ?rs1 col ?team-color))
(wm-fact (key domain fact rs-ring-spec args? m ?rs1 r ?ring1-color rn ?bases-needed1))

(wm-fact (key domain fact mps-type args? m ?rs2 t RS))
(wm-fact (key domain fact mps-team args? m ?rs2 col ?team-color))
(wm-fact (key domain fact rs-ring-spec args? m ?rs2 r ?ring2-color rn ?bases-needed2))

(wm-fact (key domain fact mps-type args? m ?ds t DS))
(wm-fact (key domain fact mps-team args? m ?ds col ?team-color))

(not(wm-fact (key domain fact order-fulfilled args? ord ?order)))
(not (goal (class PRODUCE-C0)))
(not (goal (class PRODUCE-C1)))
(not (goal (class PRODUCE-C2)))
(not (goal (class PRODUCE-C3)))

 ;(params order ?order bs-color ?any-base-color ring1-color ?any-ring1-color ring2-color ?any-ring2-color cs-color ?any-cap-color wp ?any-wp bs ?any-bs cs ?any-cs ds ?any-ds rs1 ?any-rs1 rs2 ?any-rs2)))
 =>
 (printout t "Goal for C2 order " ?order " formulated: " ?base-color " " ?ring1-color " " ?ring2-color " " ?cap-color crlf)
 (bind ?wp (sym-cat WP- (random-id)))
 (assert (goal (id (sym-cat PRODUCE-C2- (gensym*)))
               (class PRODUCE-C2)(sub-type RUN-SUBGOALS-IN-PARALLEL)(priority ?*PRIORITY-C2*)
               (params order ?order bs-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color cs-color ?cap-color wp ?wp bs ?bs cs ?cs ds ?ds rs1 ?rs1 rs2 ?rs2)
 ))
)

(defrule goal-produce-c2-get-base-wait
  "get a base for c2-production and wait at the ring-station"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c2-id) (class PRODUCE-C2) (mode SELECTED) (params order ?order bs-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color cs-color ?cap-color wp ?wp bs ?bs cs ?cs ds ?ds rs1 ?rs1 rs2 ?rs2))
  (not (goal (class GET-BASE-WAIT) (parent ?produce-c2-id)))
  =>
  (printout t "Goal GET-BASE-WAIT formulated" crlf)
  (assert (goal (id (sym-cat GET-BASE-WAIT- (gensym*))) (class GET-BASE-WAIT) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params bs-color ?base-color wp ?wp bs ?bs wait-pos ?rs1 wait-side INPUT)))
)

(defrule goal-produce-c2-buffer-cs
  "feed a cap into the cap station for c2-production"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c2-id) (class PRODUCE-C2) (mode SELECTED) (params order ?order bs-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color cs-color ?cap-color wp ?wp bs ?bs cs ?cs ds ?ds rs1 ?rs1 rs2 ?rs2))
  (not (goal (class BUFFER-CS) (parent ?produce-c2-id)))
  =>
  (printout t "Goal BUFFER-CS formulated" crlf)
  (assert (goal (id (sym-cat BUFFER-CS- (gensym*))) (class BUFFER-CS) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params cs-color ?cap-color cs ?cs)))
)

(defrule goal-produce-c2-buffer-rs1
  "feed a base into the ring station for the first ring for c2-production"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c2-id) (class PRODUCE-C2) (mode SELECTED)
        (params order ?order bs-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color cs-color ?cap-color wp ?wp bs ?bs cs ?cs ds ?ds rs1 ?rs1 rs2 ?rs2)
  )
  (wm-fact (key domain fact rs-ring-spec args? m ?rs1 r ?ring1-color rn ?bases-needed1))
  (not (goal (class BUFFER-RS) (parent ?produce-c2-id) (params bs ?any-bs rs ?any-rs1 for ONE)))
  (not (goal (class BUFFER-RS) (parent ?produce-c2-id) (params robot ?any-robot bs ?any-bs rs ?any-rs1 for ONE)))
  =>
  (printout t "Goals BUFFER-RS formulated. bases-needed: " ?bases-needed1 crlf)
  (if (eq ?bases-needed1 ONE) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs1 for ONE)))
  )
  (if (eq ?bases-needed1 TWO) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs1 for ONE)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs1 for ONE)))
  )
  (if (eq ?bases-needed1 THREE) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs1 for ONE)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs1 for ONE)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs1 for ONE)))
  )
)

(defrule goal-produce-c2-buffer-rs2
  "feed a base into the ring station for the second ring for c2-production"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c2-id) (class PRODUCE-C2) (mode SELECTED)
        (params order ?order bs-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color cs-color ?cap-color wp ?wp bs ?bs cs ?cs ds ?ds rs1 ?rs1 rs2 ?rs2)
  )
  (wm-fact (key domain fact rs-ring-spec args? m ?rs2 r ?ring2-color rn ?bases-needed2))
  (not (goal (class BUFFER-RS) (parent ?produce-c2-id) (params bs ?any-bs rs ?any-rs2 for TWO)))
  (not (goal (class BUFFER-RS) (parent ?produce-c2-id) (params robot ?any-robot bs ?any-bs rs ?any-rs2 for TWO)))
  =>
  (printout t "Goals BUFFER-RS formulated. bases-needed: " ?bases-needed2 crlf)
  (if (eq ?bases-needed2 ONE) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs2 for TWO)))
  )
  (if (eq ?bases-needed2 TWO) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs2 for TWO)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs2 for TWO)))
  )
  (if (eq ?bases-needed2 THREE) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs2 for TWO)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs2 for TWO)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs2 for TWO)))
  )
)

(defrule goal-produce-c2-mount-ring1
  "mount ring 1 for c2-production and wait at the cap-station"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c2-id) (class PRODUCE-C2) (mode SELECTED) (params order ?order bs-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color cs-color ?cap-color wp ?wp bs ?bs cs ?cs ds ?ds rs1 ?rs1 rs2 ?rs2))
  (not (goal (class MOUNT-RING) (parent ?produce-c2-id)))
  =>
  (printout t "Goal MOUNT-RING1 formulated" crlf)
  (assert (goal (id (sym-cat MOUNT-RING1- (gensym*))) (class MOUNT-RING1) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params ring1-color ?ring1-color wp ?wp rs ?rs1 wait-pos ?rs2 wait-side INPUT)))
)

(defrule goal-produce-c2-mount-ring2
  "mount ring 2 for c2-production and wait at the cap-station"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c2-id) (class PRODUCE-C2) (mode SELECTED) (params order ?order bs-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color cs-color ?cap-color wp ?wp bs ?bs cs ?cs ds ?ds rs1 ?rs1 rs2 ?rs2))
  (not (goal (class MOUNT-RING) (parent ?produce-c2-id)))
  =>
  (printout t "Goal MOUNT-RING2 formulated" crlf)
  (assert (goal (id (sym-cat MOUNT-RING2- (gensym*))) (class MOUNT-RING2) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED) (params ring1-color ?ring1-color ring2-color ?ring2-color wp ?wp rs ?rs2 wait-pos ?cs wait-side INPUT)))
)

(defrule goal-produce-c2-mount-cap-deliver
  "mount the cap for c2-production and and deliver the product"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c2-id) (class PRODUCE-C2) (mode SELECTED))
  (not (goal (class MOUNT-CAP-DELIVER-C2) (parent ?produce-c2-id)))
  =>
  (printout t "Goal MOUNT-CAP-DELIVER formulated" crlf)
  (assert (goal (id (sym-cat MOUNT-CAP-DELIVER- (gensym*))) (class MOUNT-CAP-DELIVER-C2) (parent ?produce-c2-id) (sub-type SIMPLE) (mode FORMULATED)))
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
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?ring2-color))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?ring3-color))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))

 (wm-fact (key refbox team-color) (value ?team-color))
 ; wait for machines to be initialized. Thanks to Chris for this fix.
 (wm-fact (key domain fact entered-field args? r ?some-robot))

 (wm-fact (key domain fact mps-type args? m ?cs t CS))
 (wm-fact (key domain fact mps-team args? m ?cs col ?team-color))
 (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?cs spot ?shelf-spot))
 (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))

 (wm-fact (key domain fact mps-type args? m ?bs t BS))
 (wm-fact (key domain fact mps-team args? m ?bs col ?team-color))

 (wm-fact (key domain fact mps-type args? m ?rs1 t RS))
 (wm-fact (key domain fact mps-team args? m ?rs1 col ?team-color))
 (wm-fact (key domain fact rs-ring-spec args? m ?rs1 r ?ring1-color rn ?bases-needed1))

 (wm-fact (key domain fact mps-type args? m ?rs2 t RS))
 (wm-fact (key domain fact mps-team args? m ?rs2 col ?team-color))
 (wm-fact (key domain fact rs-ring-spec args? m ?rs2 r ?ring2-color rn ?bases-needed2))

 (wm-fact (key domain fact mps-type args? m ?rs3 t RS))
 (wm-fact (key domain fact mps-team args? m ?rs3 col ?team-color))
 (wm-fact (key domain fact rs-ring-spec args? m ?rs3 r ?ring3-color rn ?bases-needed3))

 (wm-fact (key domain fact mps-type args? m ?ds t DS))
 (wm-fact (key domain fact mps-team args? m ?ds col ?team-color))

 (not(wm-fact (key domain fact order-fulfilled args? ord ?order)))
 (not (goal (class PRODUCE-C0)))
 (not (goal (class PRODUCE-C1)))
 (not (goal (class PRODUCE-C2)))
 (not (goal (class PRODUCE-C3)))
  ;(params order ?order bs-color ?any-base-color ring1-color ?any-ring1-color ring2-color ?any-ring2-color ring3-color ?any-ring3-color cs-color ?any-cap-color wp ?any-wp bs ?any-bs cs ?any-cs ds ?any-ds rs1 ?any-rs1 rs2 ?any-rs2 rs3 ?any-rs3)))
  =>
  (printout t "Goal for C3 order " ?order " formulated: " ?base-color " " ?ring1-color " " ?ring2-color " " ?ring3-color " " ?cap-color crlf)
  (bind ?wp (sym-cat WP- (random-id)))
  (bind ?*FIRST-GOAL* 0)

  (assert (goal (id (sym-cat PRODUCE-C3- (gensym*)))
                (class PRODUCE-C3)(sub-type RUN-SUBGOALS-IN-PARALLEL)(priority ?*PRIORITY-C3*)
                (params order ?order bs-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color ring3-color ?ring3-color cs-color ?cap-color wp ?wp bs ?bs cs ?cs ds ?ds rs1 ?rs1 rs2 ?rs2 rs3 ?rs3)
  ))
)

(defrule goal-produce-c3-get-base-wait
  "get a base for c3-production and wait at the ring-station"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c3-id) (class PRODUCE-C3) (mode SELECTED) (params order ?order bs-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color ring3-color ?ring3-color cs-color ?cap-color wp ?wp bs ?bs cs ?cs ds ?ds rs1 ?rs1 rs2 ?rs2 rs3 ?rs3))
  (not (goal (class GET-BASE-WAIT) (parent ?produce-c3-id)))
  =>
  (printout t "Goal GET-BASE-WAIT formulated" crlf)
  (assert (goal (id (sym-cat GET-BASE-WAIT- (gensym*))) (class GET-BASE-WAIT) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params bs-color ?base-color wp ?wp bs ?bs wait-pos ?rs1 wait-side INPUT)))
)

(defrule goal-produce-c3-buffer-cs
  "feed a cap into the cap station for c3-production"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c3-id) (class PRODUCE-C3) (mode SELECTED) (params order ?order bs-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color ring3-color ?ring3-color cs-color ?cap-color wp ?wp bs ?bs cs ?cs ds ?ds rs1 ?rs1 rs2 ?rs2 rs3 ?rs3))
  (not (goal (class BUFFER-CS) (parent ?produce-c3-id)))
  =>
  (printout t "Goal BUFFER-CS formulated" crlf)
  (assert (goal (id (sym-cat BUFFER-CS- (gensym*))) (class BUFFER-CS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params cs-color ?cap-color cs ?cs)))
)

(defrule goal-produce-c3-buffer-rs1
  "feed a base into the ring station for the first ring for c3-production"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c3-id) (class PRODUCE-C3) (mode SELECTED)
        (params order ?order bs-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color ring3-color ?ring3-color cs-color ?cap-color wp ?wp bs ?bs cs ?cs ds ?ds rs1 ?rs1 rs2 ?rs2 rs3 ?rs3)
  )
  (wm-fact (key domain fact rs-ring-spec args? m ?rs1 r ?ring1-color rn ?bases-needed1))
  (not (goal (class BUFFER-RS) (parent ?produce-c3-id) (params bs ?any-bs rs ?any-rs1 for ONE)))
  (not (goal (class BUFFER-RS) (parent ?produce-c3-id) (params robot ?any-robot bs ?any-bs rs ?any-rs1 for ONE)))
  =>
  (printout t "Goals BUFFER-RS formulated. bases-needed: " ?bases-needed1 crlf)
  (if (eq ?bases-needed1 ONE) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs1 for ONE)))
  )
  (if (eq ?bases-needed1 TWO) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs1 for ONE)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs1 for ONE)))
  )
  (if (eq ?bases-needed1 THREE) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs1 for ONE)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs1 for ONE)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs1 for ONE)))
  )
)

(defrule goal-produce-c3-buffer-rs2
  "feed a base into the ring station for the second ring for c3-production"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c3-id) (class PRODUCE-C3) (mode SELECTED)
        (params order ?order bs-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color ring3-color ?ring3-color cs-color ?cap-color wp ?wp bs ?bs cs ?cs ds ?ds rs1 ?rs1 rs2 ?rs2 rs3 ?rs3)
  )
  (wm-fact (key domain fact rs-ring-spec args? m ?rs2 r ?ring2-color rn ?bases-needed2))
  (not (goal (class BUFFER-RS) (parent ?produce-c3-id) (params bs ?any-bs rs ?any-rs2 for TWO)))
  (not (goal (class BUFFER-RS) (parent ?produce-c3-id) (params robot ?any-robot bs ?any-bs rs ?any-rs2 for TWO)))
  =>
  (printout t "Goals BUFFER-RS formulated. bases-needed: " ?bases-needed2 crlf)
  (if (eq ?bases-needed2 ONE) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs2 for TWO)))
  )
  (if (eq ?bases-needed2 TWO) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs2 for TWO)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs2 for TWO)))
  )
  (if (eq ?bases-needed2 THREE) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs2 for TWO)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs2 for TWO)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs2 for TWO)))
  )
)

(defrule goal-produce-c3-buffer-rs3
  "feed a base into the ring station for the second ring for c3-production"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c3-id) (class PRODUCE-C3) (mode SELECTED)
        (params order ?order bs-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color ring3-color ?ring3-color cs-color ?cap-color wp ?wp bs ?bs cs ?cs ds ?ds rs1 ?rs1 rs2 ?rs2 rs3 ?rs3)
  )
  (wm-fact (key domain fact rs-ring-spec args? m ?rs3 r ?ring3-color rn ?bases-needed3))
  (not (goal (class BUFFER-RS) (parent ?produce-c3-id) (params bs ?any-bs rs ?any-rs3 for THREE)))
  (not (goal (class BUFFER-RS) (parent ?produce-c3-id) (params robot ?any-robot bs ?any-bs rs ?any-rs3 for THREE)))
  =>
  (printout t "Goals BUFFER-RS formulated. bases-needed: " ?bases-needed3 crlf)
  (if (eq ?bases-needed3 ONE) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs3 for THREE)))
  )
  (if (eq ?bases-needed3 TWO) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs3 for THREE)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs3 for THREE)))
  )
  (if (eq ?bases-needed3 THREE) then 
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs3 for THREE)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs3 for THREE)))
  (assert (goal (id (sym-cat BUFFER-RS- (gensym*))) (class BUFFER-RS) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params bs ?bs rs ?rs3 for THREE)))
  )
)

(defrule goal-produce-c3-mount-ring1
  "mount ring 1 for c3-production and wait at the cap-station"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c3-id) (class PRODUCE-C3) (mode SELECTED) (params order ?order bs-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color ring3-color ?ring3-color cs-color ?cap-color wp ?wp bs ?bs cs ?cs ds ?ds rs1 ?rs1 rs2 ?rs2 rs3 ?rs3))
  (not (goal (class MOUNT-RING) (parent ?produce-c3-id)))
  =>
  (printout t "Goal MOUNT-RING1 formulated" ?order crlf)
  (assert (goal (id (sym-cat MOUNT-RING1- (gensym*))) (class MOUNT-RING1) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params ring1-color ?ring1-color wp ?wp rs ?rs1 wait-pos ?rs2 wait-side INPUT)))
)

(defrule goal-produce-c3-mount-ring2
  "mount ring 2 for c3-production and wait at the cap-station"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c3-id) (class PRODUCE-C3) (mode SELECTED) (params order ?order bs-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color ring3-color ?ring3-color cs-color ?cap-color wp ?wp bs ?bs cs ?cs ds ?ds rs1 ?rs1 rs2 ?rs2 rs3 ?rs3))
  (not (goal (class MOUNT-RING) (parent ?produce-c3-id)))
  =>
  (printout t "Goal MOUNT-RING2 formulated" crlf)
  (assert (goal (id (sym-cat MOUNT-RING2- (gensym*))) (class MOUNT-RING2) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params ring1-color ?ring1-color ring2-color ?ring2-color wp ?wp rs ?rs2 wait-pos ?rs3 wait-side INPUT)))
)

(defrule goal-produce-c3-mount-ring3
  "mount ring 3 for c3-production and wait at the cap-station"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c3-id) (class PRODUCE-C3) (mode SELECTED) (params order ?order bs-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color ring3-color ?ring3-color cs-color ?cap-color wp ?wp bs ?bs cs ?cs ds ?ds rs1 ?rs1 rs2 ?rs2 rs3 ?rs3))
  (not (goal (class MOUNT-RING) (parent ?produce-c3-id)))
  =>
  (printout t "Goal MOUNT-RING3 formulated" crlf)
  (assert (goal (id (sym-cat MOUNT-RING3- (gensym*))) (class MOUNT-RING3) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED) (params ring1-color ?ring1-color ring2-color ?ring2-color ring3-color ?ring3-color wp ?wp rs ?rs3 wait-pos ?cs wait-side INPUT)))
)

(defrule goal-produce-c3-mount-cap-deliver
  "mount the cap for c3-production and and deliver the product"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?produce-c3-id) (class PRODUCE-C3) (mode SELECTED))
  (not (goal (class MOUNT-CAP-DELIVER-C3) (parent ?produce-c3-id)))
  =>
  (printout t "Goal MOUNT-CAP-DELIVER formulated" crlf)
  (assert (goal (id (sym-cat MOUNT-CAP-DELIVER- (gensym*))) (class MOUNT-CAP-DELIVER-C3) (parent ?produce-c3-id) (sub-type SIMPLE) (mode FORMULATED)))
)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;
; Supporting TASKS
;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defrule goal-produce-supporting-tasks
" Unused robots can be assigned to general tasks such as refill shelf
"
 (declare (salience ?*SALIENCE-GOAL-FORMULATE*))

 =>
 (printout t "Goal for supporting tasks formulated "crlf)
 (bind ?wp (sym-cat WP- (random-id)))
 (assert (goal (id (sym-cat PRODUCE-SUPPORTING-TASKS- (gensym*)))
               (class SUPPORTING-TASKS)(sub-type RUN-SUBGOALS-IN-PARALLEL)(priority ?*PRIORITY-SUPPORTING-TASKS*)
 ))
)


(defrule goal-production-create-go-wait
  "Drive to a waiting position and wait there."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (wm-fact (key domain fact entered-field args? r ?robot))
  (wm-fact (key domain fact self args? r ?self))
  (goal (id ?produce-sp-id) (class PRODUCE-SUPPORTING-TASKS) (mode SELECTED))

  (not (goal (class GO-WAIT)))
  =>
  (printout t "Goal " GO-WAIT " formulated" crlf)
  (assert (goal (id (sym-cat GO-WAIT- (gensym*)))
                (parent ?produce-sp-id) 
                (class GO-WAIT) (sub-type SIMPLE)
                (priority  ?*PRIORITY-GO-WAIT*)
  ))
)

(defrule goal-produce-create-refill-shelf
  "Refill a shelf whenever it is empty."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (not (wm-fact (key domain fact wp-on-shelf args? wp ?wp m ?mps spot ?spot)))
  (goal (id ?produce-sp-id) (class PRODUCE-SUPPORTING-TASKS) (mode SELECTED))
  (not (goal (class REFILL-SHELF) (parent ?produce-sp-id)))
  =>
  (printout t "Goal REFILL-SHELF formulated" crlf)
  (assert (goal (id (sym-cat REFILL-SHELF- (gensym*))) (class REFILL-SHELF) (parent ?produce-sp-id) (sub-type SIMPLE)(priority ?*PRIORITY-REFILL-SHELF*)(mode FORMULATED)(params m ?mps )))
)

