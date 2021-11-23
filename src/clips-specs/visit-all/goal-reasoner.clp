(defglobal
  ?*SALIENCE-GOAL-FORMULATE* = 500
  ?*SALIENCE-GOAL-EXECUTABLE-CHECK* = 450
  ?*SALIENCE-GOAL-REJECT* = 400
  ?*SALIENCE-GOAL-EXPAND* = 300
  ?*SALIENCE-GOAL-SELECT* = 200
  ?*SALIENCE-GOAL-EVALUATE-GENERIC* = -1
)

; #  Goal Creation
(defrule goal-reasoner-create
	(not (goal (id VISIT-BS)))
	(not (goal (id VISIT-CS1)))
	(not (goal (id VISIT-CS2)))
	(not (goal (id VISIT-RS1)))
	(not (goal (id VISIT-RS2)))
	(not (goal (id VISIT-SS)))
	(not (goal (id VISIT-DS)))
	(not (goal-already-tried))
	(wm-fact (key refbox phase) (value PRODUCTION))
	(wm-fact (key game state) (value RUNNING))
	(wm-fact (key refbox team-color) (value ?color&~nil))
	(domain-facts-loaded)
	(wm-fact (key navgraph waitzone generated) (value TRUE))
	=>
	(assert (goal (id VISIT-BS) (params target-mps C-BS exec-robot ROB1)))
	(assert (goal (id VISIT-CS1) (params target-mps C-CS1 exec-robot ROB1)))	
	(assert (goal (id VISIT-CS2) (params target-mps C-CS2 exec-robot ROB1)))
	(assert (goal (id VISIT-RS1) (params target-mps C-RS1 exec-robot ROB1)))	
	(assert (goal (id VISIT-RS2) (params target-mps C-RS2 exec-robot ROB1)))
	(assert (goal (id VISIT-SS) (params target-mps C-SS exec-robot ROB1)))	
	(assert (goal (id VISIT-DS) (params target-mps C-DS exec-robot ROB1)))
	; This is just to make sure we formulate the goal only once.
	; In an actual domain this would be more sophisticated.
	(assert(goal-already-tried))
)

; #  Goal Selection
; We can choose one or more goals for expansion, e.g., calling
; a planner to determine the required steps.
(defrule goal-reasoner-select
	?g <- (goal (id ?goal-id) (mode FORMULATED) (parent nil))
	=>
	(modify ?g (mode SELECTED))
)

; #  Commit to goal (we "intend" it)
; A goal might actually be expanded into multiple plans, e.g., by
; different planners. This step would allow to commit one out of these
; plans.
(defrule goal-reasoner-commit
	?g <- (goal (id ?goal-id) (mode EXPANDED))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(not (plan (id ?o-plan-id&:(neq ?o-plan-id ?plan-id)) (goal-id ?goal-id)))
	=>
	(modify ?g (mode COMMITTED) (committed-to ?plan-id))
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

; #  Goal Monitoring
(defrule goal-reasoner-completed
(declare (salience ?*SALIENCE-GOAL-EVALUATE-GENERIC*))
	?g <- (goal (id ?goal-id) (mode FINISHED) (outcome COMPLETED))
	=>
	(modify ?g (mode EVALUATED))
)

(defrule goal-reasoner-retract-achieve
" Retract a goal if it hold no resources anymore.
"
(declare (salience ?*SALIENCE-GOAL-EVALUATE-GENERIC*))
	?g <-(goal (id ?goal-id) (type ACHIEVE) (mode EVALUATED)
	           (acquired-resources))
	(not (goal (parent ?goal-id) (mode ?mode&~RETRACTED)))
=>
	(modify ?g (mode RETRACTED))
)

(defrule goal-reasoner-remove-retracted-goal-common
" Clean up a retracted goal
"
(declare (salience ?*SALIENCE-GOAL-EVALUATE-GENERIC*))
	?g <- (goal (id ?goal-id) (mode RETRACTED) (acquired-resources))
=>
	(delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?goal-id)
	  (delayed-do-for-all-facts ((?a plan-action)) (and (eq ?a:plan-id ?p:id) (eq ?a:goal-id ?goal-id))
	    (retract ?a)
	  )
	  (retract ?p)
	)
	(retract ?g)
)

; ----------------- RefBox Heartbeat Signal ----------------------------------

(deffunction goal-tree-assert-run-endless (?class ?frequency $?fact-addresses)
	(bind ?id (sym-cat MAINTAIN- ?class - (gensym*)))
	(bind ?goal (assert (goal (id ?id) (class ?class) (type MAINTAIN)
	                    (sub-type RUN-ENDLESS) (params frequency ?frequency)
	                    (meta last-formulated (now)))))
	(foreach ?f ?fact-addresses
	        (goal-tree-update-child ?f ?id (+ 1 (- (length$ ?fact-addresses) ?f-index))))
	(return ?goal)
)


(defrule goal-reasoner-create-beacon-maintain
" The parent goal for beacon signals. Allows formulation of
  goals that periodically communicate with the refbox.
"
	(not (goal (class BEACON-MAINTAIN)))
	(or (domain-facts-loaded)
	    (wm-fact (key refbox phase) (value ~SETUP&~PRE_GAME)))
	=>
	(bind ?goal (goal-tree-assert-run-endless BEACON-MAINTAIN 1))
	(modify ?goal (verbosity QUIET) (params frequency 1))
)


(defrule goal-reasoner-create-beacon-achieve
" Send a beacon signal whenever at least one second has elapsed since it
  last one got sent.
"
	?g <- (goal (id ?maintain-id) (class BEACON-MAINTAIN) (mode SELECTED))
	=>
	(assert (goal (id (sym-cat SEND-BEACON- (gensym*))) (sub-type SIMPLE)
	              (class SEND-BEACON) (parent ?maintain-id) (verbosity QUIET)
	              (is-executable TRUE)))
)
