;---------------------------------------------------------------------------
;  simple.clp - CLIPS executive - single goal without parent or children
;
;  Created: Tue 05 Jan 2019 15:48:31 CET
;  Copyright  2019  Tarik Viehmann <tarik.viehmann@rwth-aachen.de>
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
; Sub-type: SIMPLE
; Perform: itself
;
; A SIMPLE goal has no children. If the goal has no parent it gets selected
; automatically. The user expands and evaluates the goal. Cleanup is automated.
; Thefore SIMPLE goals are suited to act as leaf nodes of trees or
; goals without any goal.
;
; Interactions:
; - User FORMULATES goal
; - User  SELECT goal
; - User EXPANDS goal
; - Automatic: COMMIT and DISPATCH goal
; - User EVALUATES goal
; - Automatic: Clean up any attached goals and RETRACT goal

(deffunction simple-goal-log-debug ($?verbosity)
	(bind ?v (nth$ 1 ?verbosity))
	(switch ?v
		(case NOISY then (return t))
		(case DEFAULT then (return nil))
		(case QUIET then (return nil))
	)
	(return nil)
)

(deffunction simple-goal-log-info ($?verbosity)
	(bind ?v (nth$ 1 ?verbosity))
	(switch ?v
		(case NOISY then (return warn))
		(case DEFAULT then (return t))
		(case QUIET then (return nil))
	)
	(return t)
)


(defrule simple-goal-commit
  ?g <- (goal (id ?goal-id) (sub-type SIMPLE) (mode EXPANDED) (verbosity ?v))
  (not (goal (parent ?goal-id)))
=>
  (printout (simple-goal-log-debug ?v) "Goal " ?goal-id " COMMITTED (SIMPLE)" crlf)
  (modify ?g (mode COMMITTED))
)


(defrule simple-goal-fail-because-of-subgoal
  ?g <- (goal (id ?goal-id) (sub-type SIMPLE) (verbosity ?v)
              (mode ~FINISHED&~EVALUATED&~RETRACTED))
  (goal (parent ?goal-id))
=>
  (printout (simple-goal-log-debug ?v) "Goal " ?goal-id
            " FINISHED with FAILED (SIMPLE)" crlf)
  (modify ?g (mode FINISHED) (outcome FAILED)
              (error SUB-GOAL)
              (message (str-cat "Sub-goal for SIMPLE goal '" ?goal-id "'")))
)


(defrule simple-goal-dispatch
  ?g <- (goal (id ?goal-id) (type ACHIEVE) (sub-type SIMPLE) (mode COMMITTED)
              (class ?type)  (committed-to nil) (required-resources $?req)
              (verbosity ?v)
              (acquired-resources $?acq&:(subsetp ?req ?acq)))
  (not (goal (parent ?goal-id)))
=>
  (printout (simple-goal-log-info ?v) "Goal " ?goal-id " DISPATCHED (SIMPLE)" crlf)
  (modify ?g (mode DISPATCHED))
)


(defrule simple-goal-retract
 ?g <- (goal (id ?goal-id) (type ACHIEVE) (sub-type SIMPLE) (mode EVALUATED)
             (verbosity ?v) (acquired-resources))
  (not (goal (parent ?goal-id)))
=>
  (delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?goal-id)
    (delayed-do-for-all-facts ((?a plan-action)) (eq ?a:plan-id ?p:id)
      (retract ?a)
    )
    (retract ?p)
  )
 (printout (simple-goal-log-debug ?v) "Goal " ?goal-id " RETRACTED (SIMPLE)" crlf)
 (modify ?g (mode RETRACTED))
)
