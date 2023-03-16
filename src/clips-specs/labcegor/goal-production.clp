;---------------------------------------------------------------------------
;  goal-production.clp - Generate production goals of RCLL
;
;  Created: Tue 09 Jan 2018 17:03:31 CET
;  Copyright  2018  Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>
;             2021  Tarik Viehmann <viehmann@kbsg.rwth-aachen.de>
;             2021  Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
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


; ----------------------- Maintenance Goals -------------------------------

(deffunction goal-meta-assert (?goal ?robot ?order-id ?ring-nr)
"Creates the goal-meta fact, assigns the goal to the robot and to its order"
  (assert (goal-meta (goal-id (fact-slot-value ?goal id))
                     (assigned-to ?robot)
                     (order-id ?order-id)
                     (ring-nr ?ring-nr)))
  (return ?goal)
)

(defrule goal-production-create-refill-shelf-maintain
" The parent goal to refill a shelf. Allows formulation of goals to refill
  a shelf only if the game is in the production phase and the domain is loaded.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (domain-facts-loaded)
  (not (goal (class REFILL-SHELF-MAINTAIN)))
  (wm-fact (key refbox phase) (value PRODUCTION))
  =>
  (bind ?goal (goal-tree-assert-run-endless REFILL-SHELF-MAINTAIN 1))
  (modify ?goal (required-resources)
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
  (bind ?goal (assert (goal (id (sym-cat REFILL-SHELF- (gensym*)))
                (class REFILL-SHELF) (sub-type SIMPLE)
                (parent ?maintain-id) (verbosity QUIET)
                (params mps ?mps) (is-executable TRUE))))
  (goal-meta-assert ?goal central nil nil)
)

; ----------------------- Robot Assignment -------------------------------

(defrule goal-production-assign-robot-to-enter-field
  (wm-fact (key central agent robot args? r ?robot))
  (not (wm-fact (key domain fact entered-field args? r ?robot)))
  (goal (id ?oid) (class ENTER-FIELD)  (sub-type SIMPLE) (mode FORMULATED) (is-executable FALSE))
  ?gm <- (goal-meta (goal-id ?oid) (assigned-to nil))
  (not (goal-meta (assigned-to ?robot)))
  =>
  (modify ?gm (assigned-to ?robot))
)


; ----------------------- Util -------------------------------


(deffunction goal-meta-assign-robot-to-goal (?goal ?robot)
"Changes an existing goal-meta fact and assign it to the given robot"
  (if (eq (fact-slot-value ?goal id) FALSE) then
    (printout t "Goal has no id! " ?goal crlf)
    (return)
  )
  (if (eq ?robot nil) then (return ))
  (if (not (do-for-fact ((?f goal-meta))
      (and (eq ?f:goal-id (fact-slot-value ?goal id))
           (or (eq ?f:restricted-to ?robot)
               (eq ?f:restricted-to nil)))
      (modify ?f (assigned-to ?robot))))
   then
    (printout t "FAILED assign robot " ?robot " to goal "
      (fact-slot-value ?goal id) crlf)
  )
)

(deffunction goal-meta-assert (?goal ?robot ?order-id ?ring-nr)
"Creates the goal-meta fact, assigns the goal to the robot and to its order"
  (assert (goal-meta (goal-id (fact-slot-value ?goal id))
                     (assigned-to ?robot)
                     (order-id ?order-id)
                     (ring-nr ?ring-nr)))
  (return ?goal)
)

(defrule goal-production-navgraph-compute-wait-positions-finished
  "Add the waiting points to the domain once their generation is finished."
  (NavGraphWithMPSGeneratorInterface (id "/navgraph-generator-mps") (final TRUE))
  (or (wm-fact (key config rcll use-static-navgraph) (type BOOL) (value TRUE))
      (forall
        (wm-fact (key central agent robot args? r ?robot))
        (NavGraphWithMPSGeneratorInterface (id ?id&:(eq ?id (remote-if-id ?robot "navgraph-generator-mps"))) (final TRUE))
      )
  )
=>
  (printout t "Navgraph generation of waiting-points finished. Getting waitpoints." crlf)
  (do-for-all-facts ((?waitzone navgraph-node)) (str-index "WAIT-" ?waitzone:name)
    (assert
      (domain-object (name (sym-cat ?waitzone:name)) (type waitpoint))
      (wm-fact (key navgraph waitzone args? name (sym-cat ?waitzone:name)) (is-list TRUE) (type INT) (values (nth$ 1 ?waitzone:pos) (nth$ 2 ?waitzone:pos)))
    )
  )
  (assert (wm-fact (key navgraph waitzone generated) (type BOOL) (value TRUE)))
  (delayed-do-for-all-facts ((?wm wm-fact)) (wm-key-prefix ?wm:key (create$ central agent robot))
    (assert (wm-fact (key central agent robot-waiting args? r (wm-key-arg ?wm:key r))))
  )
)

; ----------------------- Robot Assignment -------------------------------

(defrule goal-production-assign-robot-to-enter-field
  (wm-fact (key central agent robot args? r ?robot))
  (not (wm-fact (key domain fact entered-field args? r ?robot)))
  (goal (id ?oid) (class ENTER-FIELD)  (sub-type SIMPLE) (mode FORMULATED) (is-executable FALSE))
  ?gm <- (goal-meta (goal-id ?oid) (assigned-to nil))
  (not (goal-meta (assigned-to ?robot)))
  =>
  (modify ?gm (assigned-to ?robot))
)


(defrule goal-production-unassign-robot-from-finished-goals
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?g <- (goal (id ?id) (sub-type SIMPLE) (mode RETRACTED)
        (parent ?parent))
  (not (goal (id ?parent) (type MAINTAIN)))
  (goal-meta (goal-id ?id) (assigned-to ?robot&~nil))
  =>
  (remove-robot-assignment-from-goal-meta ?g)
)

(deffunction goal-production-assert-enter-field
  (?team-color)

  (bind ?goal (assert (goal (class ENTER-FIELD)
              (id (sym-cat ENTER-FIELD- (gensym*)))
              (sub-type SIMPLE)
              (verbosity NOISY) (is-executable FALSE)
              (params team-color ?team-color)
              (meta-template goal-meta)
  )))
  (return ?goal)
)


(defrule goal-production-create-enter-field
  "Enter the field (drive outside of the starting box)."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (wm-fact (key central agent robot args? r ?robot))
  (not (wm-fact (key domain fact entered-field args? r ?robot)))
  (not (goal (id ?some-goal-id) (class ENTER-FIELD) (mode FORMULATED|SELECTED|EXPANDED|COMMITTED)))
  (domain-facts-loaded)
  (wm-fact (key refbox team-color) (value ?team-color))
  =>
  (printout t "Goal " ENTER-FIELD " formulated" crlf)
  (goal-production-assert-enter-field ?team-color)
)

(defrule goal-production-remove-enter-field
  "Enter the field (drive outside of the starting box)."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?gf <- (goal (id ?some-goal-id) (class ENTER-FIELD) (mode RETRACTED))
  ?gm <- (goal-meta (goal-id ?some-goal-id))
  =>
  (printout t "Goal " ENTER-FIELD " removed after entering" crlf)
  (retract ?gf ?gm)
)





;;;;;;;;;;;;  CHANGES BY VISHWAS JAIN  ;;;;;;;;;;;





; (deffunction goal-production-find-a-robot (?task)
;   "Find a free robot"
;   (bind ?assn-robot nil)
;   (if (eq ?assn-robot nil) then
;     (if (not (do-for-all-facts ((?f goal-meta))
; 			                (and (neq ?f:assigned-to robot1) (eq ?task PRIMARY_TASK))
; 			                (bind ?assn-robot robot1)
;              )
;         ) then 
;         (if (not (do-for-all-facts ((?f goal-meta))
; 			                (and (neq ?f:assigned-to robot2) (eq ?task SECONDARY_TASK))
; 			                (bind ?assn-robot robot2)
;                  )
;             ) then 
;             (if (not (do-for-all-facts ((?f goal-meta))
; 			                                  (and (neq ?f:assigned-to robot3)  (eq ?task SECONDARY_TASK))
; 			                                  (bind ?assn-robot robot3)
;                      )
;                 ) then 
;                   (bind ?assn-robot nil) (printout t "assn-robot has been assigned as nil")
;             )
;         )
;     )
;   )
;   (return ?assn-robot)
; )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;   GROUP-1 COMPLEXITY-1 PAYMENT PARALLEL    ;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(deffunction goal-production-g1-c1-spawn-wp
	(?ord ?wp ?task ?num)

	(bind ?goal-id (sym-cat PRODUCT- ?ord -n ?num - (gensym*)))

  (bind ?cls (sym-cat PRODUCT- ?ord -T-spawn-wp))
  
  ; (bind ?robot (goal-production-find-a-robot ?task))

  (bind ?g (assert (goal (class ?cls)
                (id ?goal-id)
                (sub-type SIMPLE)
                (verbosity NOISY) (is-executable FALSE) 
                (params workpiece ?wp) 
                (meta-template goal-meta)
  )))
  (assert (goal-meta (goal-id ?goal-id) (assigned-to nil) (sub-task-type ?task)))
  (return ?g)
)


(deffunction goal-production-g1-c1-prepare-bs
	(?ord ?bs ?bs-clr ?task ?num)

	(bind ?goal-id (sym-cat PRODUCT- ?ord -n ?num - (gensym*)))

  (bind ?cls (sym-cat PRODUCT- ?ord -T-prep-bs))
  
  ; (bind ?robot (goal-production-find-a-robot ?task))

  (bind ?g (assert (goal (class ?cls)
                (id ?goal-id)
                (sub-type SIMPLE)
                (verbosity NOISY) (is-executable FALSE) 
                (params bs ?bs bs-side OUTPUT base-clr ?bs-clr) 
                (meta-template goal-meta)
  )))
  (assert (goal-meta (goal-id ?goal-id) (assigned-to nil) (sub-task-type ?task)))
  (return ?g)
)


(deffunction goal-production-g1-c1-prepare-rs
	(?ord ?rs ?rng-clr ?rng-req ?task ?num)

	(bind ?goal-id (sym-cat PRODUCT- ?ord -n ?num - (gensym*)))

  (bind ?cls (sym-cat PRODUCT- ?ord -T-prep-rs))
  
  ; (bind ?robot (goal-production-find-a-robot ?task))

  (do-for-fact ((?rs-status wm-fact))
			              (and (wm-key-prefix ?rs-status:key (create$ domain fact rs-filled-with))
			                   (eq (wm-key-arg ?rs-status:key m) ?rs))
			              (bind ?rng-before (wm-key-arg ?rs-status:key n))
  )

  (bind ?a (sym-to-int ?rng-before))
  (bind ?b (sym-to-int ?rng-req))
  (bind ?rng-after (int-to-sym (- ?a ?b)))


  (bind ?g (assert (goal (class ?cls)
                (id ?goal-id)
                (sub-type SIMPLE)
                (verbosity NOISY) (is-executable FALSE) 
                (params rs ?rs ring-color ?rng-clr ring-before ?rng-before ring-after ?rng-after ring-req ?rng-req) 
                (meta-template goal-meta)
  )))
  (assert (goal-meta (goal-id ?goal-id) (assigned-to nil) (sub-task-type ?task)))
  (return ?g)
)


(deffunction goal-production-g1-c1-cap-retrieve
	(?ord ?cs ?cap-clr ?task ?num)

	(bind ?goal-id (sym-cat PRODUCT- ?ord -n ?num - (gensym*)))

  (bind ?cls (sym-cat PRODUCT- ?ord -T-cap-retrv))
  
  ; (bind ?robot (goal-production-find-a-robot ?task))

  (bind ?g (assert (goal (class ?cls)
                (id ?goal-id)
                (sub-type SIMPLE)
                (verbosity NOISY) (is-executable FALSE) 
                (params cap-color ?cap-clr cap-station ?cs) 
                (meta-template goal-meta)
  )))
  (assert (goal-meta (goal-id ?goal-id) (assigned-to nil) (sub-task-type ?task)))
  (return ?g)
)


(deffunction goal-production-g1-c1-make-payment-cs
	(?ord ?cs ?rs ?task ?num)

	(bind ?goal-id (sym-cat PRODUCT- ?ord -n ?num - (gensym*)))

  (bind ?cls (sym-cat PRODUCT- ?ord -T-pay-bs))
  
  (do-for-fact ((?rs-status wm-fact))
			              (and (wm-key-prefix ?rs-status:key (create$ domain fact rs-filled-with))
			                   (eq (wm-key-arg ?rs-status:key m) ?rs))
			              (bind ?rng-before (wm-key-arg ?rs-status:key n))
  )
  (bind ?a (sym-to-int ?rng-before))
  (bind ?b (sym-to-int 1))
  (bind ?rng-after (int-to-sym (+ ?a ?b)))

  (bind ?g (assert (goal (class ?cls)
                (id ?goal-id)
                (sub-type SIMPLE)
                (verbosity NOISY) (is-executable FALSE) 
                (params cs ?cs rs ?rs ring-before ?rng-before ring-after ?rng-after) 
                (meta-template goal-meta)
  )))
  (assert (goal-meta (goal-id ?goal-id) (assigned-to nil) (sub-task-type ?task)))
  (return ?g)
)


(deffunction goal-production-g1-c1-make-payment-bs
	(?ord ?bs ?rs ?task ?num)

	(bind ?goal-id (sym-cat PRODUCT- ?ord -n ?num - (gensym*)))

  (bind ?cls (sym-cat PRODUCT- ?ord -T-pay-bs))
  
  ; (bind ?robot (goal-production-find-a-robot ?task))

  (bind ?g (assert (goal (class ?cls)
                (id ?goal-id)
                (sub-type SIMPLE)
                (verbosity NOISY) (is-executable FALSE) 
                (params bs ?bs rs ?rs) 
                (meta-template goal-meta)
  )))
  (assert (goal-meta (goal-id ?goal-id) (assigned-to nil) (sub-task-type ?task)))
  (return ?g)
)


(deffunction goal-production-g1-c1-transport-wp
	(?ord ?from ?from-side ?to ?to-side ?wp ?task ?num)

	(bind ?goal-id (sym-cat PRODUCT- ?ord -n ?num - (gensym*)))

  (bind ?cls (sym-cat PRODUCT- ?ord -T-transport))
  
  ; (bind ?robot (goal-production-find-a-robot ?task))

  (bind ?g (assert (goal (class ?cls)
                (id ?goal-id)
                (sub-type SIMPLE)
                (verbosity NOISY) (is-executable FALSE) 
                (params from ?from from-side ?from-side to ?to to-side ?to-side workpiece ?wp) 
                (meta-template goal-meta)
  )))
  (assert (goal-meta (goal-id ?goal-id) (assigned-to nil) (sub-task-type ?task)))
  (return ?g)

)


(deffunction goal-production-g1-c1-bs-dispense
	(?ord ?bs ?bs-clr ?wp ?task ?num)

	(bind ?goal-id (sym-cat PRODUCT- ?ord -n ?num - (gensym*)))

  (bind ?cls (sym-cat PRODUCT- ?ord -T-bs-disp))
  
  ; (bind ?robot (goal-production-find-a-robot ?task))

  (bind ?g (assert (goal (class ?cls)
                (id ?goal-id)
                (sub-type SIMPLE)
                (verbosity NOISY) (is-executable FALSE) 
                (params bs ?bs bs-clr ?bs-clr workpiece ?wp) 
                (meta-template goal-meta)
  )))
  (assert (goal-meta (goal-id ?goal-id) (assigned-to nil) (sub-task-type ?task)))
  (return ?g)

)


(deffunction goal-production-g1-c1-mount-ring1
	(?ord ?rs ?rng-clr ?rng-req ?wp ?task ?num)

	(bind ?goal-id (sym-cat PRODUCT- ?ord -n ?num - (gensym*)))

  (bind ?cls (sym-cat PRODUCT- ?ord -T-mount-rng))
  
  ; (bind ?robot (goal-production-find-a-robot ?task))

  (do-for-fact ((?rs-status wm-fact))
			              (and (wm-key-prefix ?rs-status:key (create$ domain fact rs-filled-with))
			                   (eq (wm-key-arg ?rs-status:key m) ?rs))
			              (bind ?rng-before (wm-key-arg ?rs-status:key n))
  )

  (bind ?a (sym-to-int ?rng-before))
  (bind ?b (sym-to-int ?rng-req))
  (bind ?rng-after (int-to-sym (- ?a ?b)))

  (bind ?g (assert (goal (class ?cls)
                (id ?goal-id)
                (sub-type SIMPLE)
                (verbosity NOISY) (is-executable FALSE) 
                (params rs ?rs ring-clr ?rng-clr ring-before ?rng-before ring-after ?rng-after ring-req ?rng-req workpiece ?wp) 
                (meta-template goal-meta)
  )))
  (assert (goal-meta (goal-id ?goal-id) (assigned-to nil) (sub-task-type ?task)))
  (return ?g)

)



(deffunction goal-production-g1-c1-cap-mount
	(?ord ?cs ?cap-clr ?wp ?task ?num)

	(bind ?goal-id (sym-cat PRODUCT- ?ord -n ?num - (gensym*)))

  (bind ?cls (sym-cat PRODUCT- ?ord -T-cap-mount))
  
  ; (bind ?robot (goal-production-find-a-robot ?task))

  (bind ?g (assert (goal (class ?cls)
                (id ?goal-id)
                (sub-type SIMPLE)
                (verbosity NOISY) (is-executable FALSE) 
                (params cs ?cs cap-clr ?cap-clr workpiece ?wp) 
                (meta-template goal-meta)
  )))
  (assert (goal-meta (goal-id ?goal-id) (assigned-to nil) (sub-task-type ?task)))
  (return ?g)

)


(deffunction goal-production-g1-c1-deliver
	(?ord ?ds ?ds-gate ?base-clr ?cap-clr ?rng-clr ?wp ?task ?num)

	(bind ?goal-id (sym-cat PRODUCT- ?ord -n ?num - (gensym*)))

  (bind ?cls (sym-cat PRODUCT- ?ord -T-deliver))
  
  ; (bind ?robot (goal-production-find-a-robot ?task))

  (bind ?g (assert (goal (class ?cls)
                (id ?goal-id)
                (sub-type SIMPLE)
                (verbosity NOISY) (is-executable FALSE) 
                (params order ?ord workpiece ?wp delivery-station ?ds ds-gate ?ds-gate base-clr ?base-clr cap-clr ?cap-clr rng-clr ?rng-clr) 
                (meta-template goal-meta)
  )))
  (assert (goal-meta (goal-id ?goal-id) (assigned-to nil) (sub-task-type ?task)))
  (return ?g)

)

;;;;;;;;;;;;;;   END OF PARALLELIZATION    ;;;;;;;;;;;;;;;;;;;;

; LIST OF FUTURE TASKS

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



; (deffunction goal-production-g1-c1-base
; 	(?rnd-id ?bs ?base-clr ?wp ?robot ?num)
; 	(bind ?goal-id (sym-cat ?rnd-id -g ?num))
;   (bind ?g (assert (goal (class ORDER1)
;                 (id ?goal-id)
;                 (sub-type SIMPLE)
;                 (verbosity NOISY) (is-executable FALSE) 
;                 (params bs ?bs bs-side OUTPUT base-color ?base-clr workpiece ?wp) 
;                 (meta-template goal-meta)
;   )))
;   (assert (goal-meta (goal-id ?goal-id) (assigned-to ?robot)))
;   (return ?g)

; )


; (deffunction goal-production-g1-c1-transport-wp
; 	(?rnd-id ?from ?from-side ?to ?to-side ?wp ?robot ?num)
; 	(bind ?goal-id (sym-cat ?rnd-id -g ?num))
;   (bind ?g (assert (goal (class ORDER1)
;                 (id ?goal-id)
;                 (sub-type SIMPLE)
;                 (verbosity NOISY) (is-executable FALSE) 
;                 (params from ?from from-side ?from-side to ?to to-side ?to-side workpiece ?wp) 
;                 (meta-template goal-meta)
;   )))
;   (assert (goal-meta (goal-id ?goal-id) (assigned-to ?robot)))
;   (return ?g)

; )


; (deffunction goal-production-g1-c1-rs
; 	(?rnd-id ?rs ?rng-clr ?wp ?robot ?num)
; 	(bind ?goal-id (sym-cat ?rnd-id -g ?num))

;   (do-for-fact ((?rs-status wm-fact))
; 			              (and (wm-key-prefix ?rs-status:key (create$ domain fact rs-ring-spec))
; 			                   (eq (wm-key-arg ?rs-status:key m) ?rs)
;                          (eq (wm-key-arg ?rs-status:key r) ?rng-clr))
; 			              (bind ?rng-pay (wm-key-arg ?rs-status:key rn))
;   )

;   (do-for-fact ((?rs-status wm-fact))
; 			              (and (wm-key-prefix ?rs-status:key (create$ domain fact rs-filled-with))
; 			                   (eq (wm-key-arg ?rs-status:key m) ?rs))
; 			              (bind ?rng-num (wm-key-arg ?rs-status:key n))
;   )

;   (bind ?a (sym-to-int ?rng-pay))
;   (bind ?b (sym-to-int ?rng-num))
;   (bind ?r-after (int-to-sym (- ?b ?a)))


;   (bind ?g (assert (goal (class ORDER1)
;                 (id ?goal-id)
;                 (sub-type SIMPLE)
;                 (verbosity NOISY) (is-executable FALSE) 
;                 (params target-rs ?rs ring-color ?rng-clr ring-before ?rng-num ring-after ?r-after ring-req ?rng-pay workpiece ?wp)
;                 (meta-template goal-meta)
;   )))
;   (assert (goal-meta (goal-id ?goal-id) (assigned-to ?robot)))
;   (return ?g)

; )



; (deffunction goal-production-g1-c1-cap-retrieve
; 	(?rnd-id ?cs ?cap-clr ?wp ?robot ?num)
; 	(bind ?goal-id (sym-cat ?rnd-id -g ?num))
;   (bind ?g (assert (goal (class ORDER1)
;                 (id ?goal-id)
;                 (sub-type SIMPLE)
;                 (verbosity NOISY) (is-executable FALSE) 
;                 (params cap-carrier ?wp cap-color ?cap-clr cap-station ?cs)
;                 (meta-template goal-meta)
;   )))
;   (assert (goal-meta (goal-id ?goal-id) (assigned-to ?robot)))
;   (return ?g)

; )


; (deffunction goal-production-g1-c1-cap-mount
; 	(?rnd-id ?cs ?cap-clr ?wp ?robot ?num)
; 	(bind ?goal-id (sym-cat ?rnd-id -g ?num))
;   (bind ?g (assert (goal (class ORDER1)
;                 (id ?goal-id)
;                 (sub-type SIMPLE)
;                 (verbosity NOISY) (is-executable FALSE) 
;                 (params order-carrier ?wp cap-color ?cap-clr cap-station ?cs)
;                 (meta-template goal-meta)
;   )))
;   (assert (goal-meta (goal-id ?goal-id) (assigned-to ?robot)))
;   (return ?g)

; )



; (deffunction goal-production-g1-c1-deliver
; 	(?rnd-id ?ds ?ds-gate ?base-clr ?cap-clr ?rng-clr ?wp ?ord ?robot ?num)
; 	(bind ?goal-id (sym-cat ?rnd-id -g ?num))
;   (bind ?g (assert (goal (class ORDER1)
;                 (id ?goal-id)
;                 (sub-type SIMPLE)
;                 (verbosity NOISY) (is-executable FALSE) 
;                 (params order ?ord workpiece ?wp delivery-station ?ds ds-gate ?ds-gate base-clr ?base-clr cap-clr ?cap-clr rng-clr ?rng-clr) 
;                 (meta-template goal-meta)
;   )))
;   (assert (goal-meta (goal-id ?goal-id) (assigned-to ?robot)))
;   (return ?g)

; )



(deffunction g1-goal-production-assert-c1
	(?base-clr ?rs ?rng-clr ?cs ?cap-clr ?ds ?ds-gate ?ord ?wp)
  
  (bind ?bs C-BS)

  (do-for-fact ((?rs-status wm-fact))
			              (and (wm-key-prefix ?rs-status:key (create$ domain fact rs-ring-spec))
			                   (eq (wm-key-arg ?rs-status:key m) ?rs)
                         (eq (wm-key-arg ?rs-status:key r) ?rng-clr))
			              (bind ?r-req (wm-key-arg ?rs-status:key rn))
  )

	(bind ?goal-tree-1
    (goal-tree-assert-central-run-parallel (sym-cat PRODUCT- ?ord -ST1) 
      (goal-production-g1-c1-spawn-wp ?ord ?wp PRIMARY_TASK 1)
      (goal-production-g1-c1-prepare-bs ?ord ?bs ?base-clr PRIMARY_TASK 2)
      (goal-production-g1-c1-bs-dispense ?ord ?bs ?base-clr ?wp PRIMARY_TASK 8)
      (goal-production-g1-c1-prepare-rs ?ord ?rs ?rng-clr ?r-req SECONDARY_TASK 3)
      (goal-production-g1-c1-cap-retrieve ?ord ?cs ?cap-clr SECONDARY_TASK 4)
      ;(goal-production-g1-c1-transport-wp ?ord ?rs OUTPUT ?cs INPUT ?wp ?robot 4)
      ;(goal-production-g1-c1-make-payment ?ord ?cs OUTPUT ?rs INPUT ?wp ?robot 4)
    )
  )

  (bind ?goal-tree-2
    (if (neq ?r-req TWO) then
      (create$
        (goal-tree-assert-central-run-parallel (sym-cat PRODUCT- ?ord -ST2) 
          (goal-production-g1-c1-make-payment-cs ?ord ?cs ?rs SECONDARY_TASK 5)
        )
      )
    else
      (create$
        (goal-tree-assert-central-run-parallel (sym-cat PRODUCT- ?ord -ST2)
          (goal-production-g1-c1-make-payment-cs ?ord ?cs ?rs SECONDARY_TASK 6)
          (goal-production-g1-c1-make-payment-bs ?ord ?bs ?rs SECONDARY_TASK 7)
        )
      )
    )
  )

  (bind ?goal-tree-3
    (goal-tree-assert-central-run-all-sequence (sym-cat PRODUCT- ?ord -PT) 
      (goal-production-g1-c1-transport-wp ?ord ?bs OUTPUT ?rs INPUT ?wp PRIMARY_TASK 9)
      (goal-production-g1-c1-mount-ring1 ?ord ?rs ?rng-clr ?r-req ?wp PRIMARY_TASK 10)
      (goal-production-g1-c1-transport-wp ?ord ?rs OUTPUT ?cs INPUT ?wp PRIMARY_TASK 11)
      (goal-production-g1-c1-cap-mount ?ord ?cs ?cap-clr ?wp PRIMARY_TASK 12)
      (goal-production-g1-c1-deliver ?ord ?ds ?ds-gate ?base-clr ?cap-clr ?rng-clr ?wp PRIMARY_TASK 13)
    )
  )

  (bind ?goal-tree 
    (goal-tree-assert-central-run-all-sequence (sym-cat PRODUCT- ?ord -C1)
      ?goal-tree-1
      ?goal-tree-2
      ?goal-tree-3
    ))

  (do-for-all-facts ((?gq goal))
			                (eq ?gq ?goal-tree)
			                (bind ?g-id ?gq:id)
  )

  (do-for-all-facts ((?gmq goal-meta))
			                (eq ?gmq:goal-id ?g-id)
			                (modify ?gmq (ring-nr ONE))
  )
  (goal-tree-update-priority-at-end ?goal-tree)
	(return ?goal-tree)
)


(defrule g1-verify-robots-in-field
  "Check if all the robots have entered the field"
  (wm-fact (key central agent robot args? r ?robot))  
  (wm-fact (key domain fact entered-field args? r ?robot)) 

  =>
  (do-for-fact ((?r-in wm-fact))
			              (and (wm-key-prefix ?r-in:key (create$ domain fact entered-field))
			                   (eq (wm-key-arg ?r-in:key r) robot1))
			              (assert (robot1-in-field))
  )

  (do-for-fact ((?r-in wm-fact))
			              (and (wm-key-prefix ?r-in:key (create$ domain fact entered-field))
			                   (eq (wm-key-arg ?r-in:key r) robot2))
			              (assert (robot2-in-field))
  )

  (do-for-fact ((?r-in wm-fact))
			              (and (wm-key-prefix ?r-in:key (create$ domain fact entered-field))
			                   (eq (wm-key-arg ?r-in:key r) robot3))
			              (assert (robot3-in-field))
  )
)


(defrule g1-goal-production-create-from-order-complexity-C1
	"Take goal from refbox"
  ;(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ; (wm-fact (key central agent robot args? r ?robot))  
  ; (wm-fact (key domain fact entered-field args? r ?robot)) 
	(robot1-in-field)
  (robot2-in-field)
  (robot3-in-field)
  (wm-fact (key domain fact order-complexity args?  ord ?ord com C1)) 
	(wm-fact (key domain fact order-ring1-color args? ord ?ord col ?rng-clr)) 
	;(wm-fact (key domain fact order-complexity args?  ord  ?ord comp ?ord-cmplx))
	(wm-fact (key domain fact order-base-color args? ord ?ord  col ?base-clr)) 
	(wm-fact (key domain fact order-cap-color args? ord ?ord col ?cap-clr)) 
	(wm-fact (key domain fact order-gate args? ord ?ord gate ?ds-gate)) 
  (not (goal-meta (goal-id ?some-goal-id) (ring-nr ONE)))
  (domain-facts-loaded) 
  (wm-fact (key refbox team-color) (value ?team-color)) 
  
	=>
	(bind ?ord-comp C1)     ; just for information
  (bind ?rnd-id (sym-cat ?ord - (gensym*) ))
  (bind ?wp (sym-cat WP - ?ord))
  ; (assert (domain-object (name ?wp) (type workpiece)))


	(if (or (eq ?rng-clr RING_BLUE) (eq ?rng-clr RING_YELLOW))
		then 
			(bind ?rs C-RS2)
		else 
			(bind ?rs C-RS1)
	)

  (if (eq ?cap-clr CAP_BLACK)
		then 
			(bind ?cs C-CS2)
		else 
			(bind ?cs C-CS1)
	)

	(bind ?goal-tree (g1-goal-production-assert-c1 ?base-clr ?rs ?rng-clr ?cs ?cap-clr C-DS ?ds-gate ?ord ?wp))
)


