;---------------------------------------------------------------------------
;  goal-reasoner.clp - Goal reasoning for RCLL domain
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

(deftemplate goal-meta
  (slot goal-id (type SYMBOL))
  (slot num-tries (type INTEGER))
  (slot max-tries (type INTEGER))
  (multislot last-achieve (type INTEGER) (cardinality 2 2) (default 0 0))
)

(defglobal
  ?*GOAL-MAX-TRIES* = 3
  ?*SALIENCE-GOAL-FORMULATE* = 500
  ?*SALIENCE-GOAL-REJECT* = 400
  ?*SALIENCE-GOAL-EXPAND* = 300
  ?*SALIENCE-GOAL-SELECT* = 200
  ; common evaluate rules should have
  ;   lower salience than case specific ones
  ?*SALIENCE-GOAL-EVALUTATE-GENERIC* = -1
  ; complexity
  ?*SALIENCE-COMPLEXITY* = 200
  ?*SALIENCE-COMPLEXITY2* = 100
)

(defrule goal-reasoner-create-complexity
  (declare (salience ?*SALIENCE-COMPLEXITY*))
	(not (goal (id COMPLEXITY2)))
	(not (goal (id COMPLEXITY)))
	(not (goal-already-tried COMPLEXITY))
	(wm-fact (key domain fact order-complexity args? ord ?order-id&O7 com C3) (value TRUE))
	(wm-fact (key config rcll robot-name) (value ?robot))
=>
  (bind ?goal-id COMPLEXITY)
	(printout t "Create goal " ?goal-id " with order " ?order-id crlf)
	(assert (goal (id ?goal-id)))
	(assert (goal-already-tried ?goal-id))
  (if (eq ?robot "R-1") then
  	(assert (wm-fact (key r-1-at ?goal-id update-request)))
]  	(assert (wm-fact (key r-2-at ?goal-id update-request)))
  	(assert (wm-fact (key r-3-at ?goal-id update-request)))
  )
)

(defrule goal-reasoner-create-complexity2
  (declare (salience ?*SALIENCE-COMPLEXITY2*))
	(not (goal (id COMPLEXITY)))
	(not (goal (id COMPLEXITY2)))
	(not (goal-already-tried COMPLEXITY2))
	(wm-fact (key domain fact order-complexity args? ord ?order-id&O1 com C0) (value TRUE))
  ; Two robots are free
  (wm-fact (key plan-action COMPLEXITY ?plan-id r-1-done))
  (wm-fact (key plan-action COMPLEXITY ?plan-id r-2-done))
	(wm-fact (key config rcll robot-name) (value ?robot))
=>
  (bind ?goal-id COMPLEXITY2)
	(printout t "Create goal " ?goal-id " with order " ?order-id crlf)
	(assert (goal (id ?goal-id)))
	(assert (goal-already-tried ?goal-id))
  (if (eq ?robot "R-1") then
  	(assert (wm-fact (key r-1-at ?goal-id update-request)))
  	(assert (wm-fact (key r-2-at ?goal-id update-request)))
  )
)

; #  Goal Selection
; We can choose one or more goals for expansion, e.g., calling
; a planner to determine the required steps.
(defrule goal-reasoner-select-goal
  (declare (salience ?*SALIENCE-GOAL-SELECT*))
  ?g <- (goal (parent nil) (id ?goal-id) (mode FORMULATED))
   =>
  (modify ?g (mode SELECTED))
  (assert (goal-meta (goal-id ?goal-id) (max-tries ?*GOAL-MAX-TRIES*)))
)

;Select subgoal only when parent goal is expanded
(defrule goal-reasoner-select-subgoal
  (declare (salience ?*SALIENCE-GOAL-SELECT*))
  ?p <- (goal (id ?parent-id) (mode EXPANDED))
  ?g <- (goal (parent ?parent-id) (mode FORMULATED)
          (id ?subgoal-id) (priority ?priority))
  ;Select the formulated subgoal with the highest priority
  (not (goal (parent ?parent-id) (mode FORMULATED)
             (priority ?h-priority&:(> ?h-priority ?priority)))
  )
  ;No other subgoal being processed
  (not (goal (parent ?parent-id)
             (mode SELECTED|EXPANDED|
                   COMMITTED|DISPATCHED|
                   FINISHED|EVALUATED))
  )
=>
  ;(printout t "Goal " ?subgoal-id " selected!" crlf)
  (modify ?g (mode SELECTED))
  (assert (goal-meta (goal-id ?subgoal-id)))
)

; # Expand a parent goal
;Expanding a parent goal means it has some formulated goals
(defrule goal-reasoner-expand-parent-goal
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  ?p <- (goal (id ?parent-id) (mode SELECTED))
  ?g <- (goal (parent ?parent-id) (mode FORMULATED))
  =>
  (modify ?p (mode EXPANDED))
)

; #  Commit to goal (we "intend" it)
; A goal might actually be expanded into multiple plans, e.g., by
; different planners. This step would allow to commit one out of these
; plans.
(defrule goal-reasoner-commit
  ?g <- (goal (mode EXPANDED))
  =>
  (modify ?g (mode COMMITTED))
)

; #  Dispatch goal (action selection and execution now kick in)
; Trigger execution of a plan. We may commit to multiple plans
; (for different goals), e.g., one per robot, or for multiple
; orders. It is then up to action selection and execution to determine
; what to do when.
(defrule goal-reasoner-dispatch
  ?g <- (goal (mode COMMITTED))
  =>
  (modify ?g (mode DISPATCHED))
)

(defrule goal-reasoner-finish-parent-goal
  ?pg <- (goal (id ?pg-id) (mode DISPATCHED))
  ?sg <- (goal (id ?sg-id) (parent ?pg-id) (mode EVALUATED) (outcome ?outcome))
  (time $?now)
  =>
  (printout debug "Goal '" ?pg-id " finised and " ?outcome "cause " ?sg-id
    "' has been Evaluated" crlf)
  ;Finish the parent goal with the same outcome
  ;Could be extended later for custom behavior
  ;in case we want to try something else to achive that goal
  (modify ?pg (mode FINISHED) (outcome ?outcome))
)

(defrule goal-reasoner-fail-parent-goal-if-subgoals-rejected
  ?pg <- (goal (id ?pg-id) (mode DISPATCHED))
  (not (goal (id ?sg-id) (parent ?pg-id) (mode ~REJECTED)))
  (goal (parent ?pg-id))
  (time $?now)
  =>
;  (printout debug "Goal '" ?pg-id " failed because all subgoald been REJECTED" crlf)
  (modify ?pg (mode FINISHED) (outcome FAILED))
)

; #  Goal Monitoring
; ## Goal Evaluation
(defrule goal-reasoner-evaluate-subgoal-common
  (declare (salience ?*SALIENCE-GOAL-EVALUTATE-GENERIC*))
  ?g <- (goal (id ?goal-id) (parent ?parent-id&~nil) (mode FINISHED) (outcome ?outcome))
  ?pg <- (goal (id ?parent-id))
  ?m <- (goal-meta (goal-id ?parent-id))
  (time $?now)
  =>
;  (printout debug "Goal '" ?goal-id "' (part of '" ?parent-id
;    "') has been completed, Evaluating" crlf)
  (modify ?g (mode EVALUATED))
  (modify ?m (last-achieve ?now))
)


(defrule goal-reasoner-evaluate-common
  (declare (salience ?*SALIENCE-GOAL-EVALUTATE-GENERIC*))
  ?g <- (goal (id ?goal-id) (parent nil) (mode FINISHED) (outcome ?outcome))
  ?gm <- (goal-meta (goal-id ?goal-id) (num-tries ?num-tries))
  =>
 ; (printout t "Goal '" ?goal-id "' has been " ?outcome ", evaluating" crlf)
  (if (eq ?outcome FAILED)
    then
    (bind ?num-tries (+ ?num-tries 1))
    (modify ?gm (num-tries ?num-tries))
    )

  (modify ?g (mode EVALUATED))

)

; # Goal Clean up
(defrule goal-reasoner-cleanup-common
  ?g <- (goal (id ?goal-id) (parent nil) (type ?goal-type)
          (mode EVALUATED) (outcome ?outcome))
  ?gm <- (goal-meta (goal-id ?goal-id) (num-tries ?num-tries) (max-tries ?max-tries))
	(wm-fact (key config rcll robot-name) (value ?robot))
  =>
 ; (printout t "Goal '" ?goal-id "' has been Evaluated, cleaning up" crlf)

  ;Flush plans of this goal
  (delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?goal-id)
    (delayed-do-for-all-facts ((?a plan-action)) (eq ?a:plan-id ?p:id)
      (retract ?a)
    )
    ; (delayed-do-for-all-facts ((?wf wm-fact)) (wm-key-prefix ?wf:key (create$ plan-action ?p:goal-id ?p:id))
    ;   (retract ?wf)
    ; )
    (retract ?p)
  )


  (delayed-do-for-all-facts ((?sg goal)) (eq ?sg:parent ?goal-id)
    (delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?sg:id)
     (delayed-do-for-all-facts ((?a plan-action)) (eq ?a:plan-id ?p:id)
      (retract ?a)
     )
     (delayed-do-for-all-facts ((?wf wm-fact))(wm-key-prefix ?wf:key (create$ plan-action ?p:goal-id ?p:id))
      (retract ?wf)
     )
     (retract ?p)
    )
  ;  (printout t "Goal '" ?sg:id "' (part of '" ?sg:parent
  ;     "') has, cleaning up" crlf)
    (retract ?sg)
  )

  (if (or (eq ?goal-type MAINTAIN)
          (and (eq ?outcome FAILED) (<= ?num-tries ?max-tries)))
    then
   ;   (printout t "Triggering re-expansion" crlf)

    (if (eq ?robot "R-1") then
        (if (eq ?goal-id COMPLEXITY) then
        	(assert (wm-fact (key r-1-at ?goal-id update-request)))
        	(assert (wm-fact (key r-2-at ?goal-id update-request)))
        	(assert (wm-fact (key r-3-at ?goal-id update-request)))
        )

        (if (eq ?goal-id COMPLEXITY2) then
        	(assert (wm-fact (key r-1-at ?goal-id update-request)))
        	(assert (wm-fact (key r-2-at ?goal-id update-request)))
        )
      )


      (modify ?g (mode SELECTED) (outcome UNKNOWN))
    else
      (retract ?g ?gm)
    )
)
