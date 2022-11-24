; Defining templates and rules for goal selection with rl

(deftemplate rl-goal-selection
	(slot next-goal-id (type SYMBOL))
)


(deftemplate rl-finished-goal
	(slot goal-id (type SYMBOL));
	(slot outcome (type SYMBOL));
	(slot result (type INTEGER));
)

; The rl calls the rl plugin and passes the goal id. It never leaves the DISPATCHED
; state.
(deffunction rl-call (?goal-id ?goal ) ;$?executableGoals)
	(printout info "Call the rl plugin." ?goal-id crlf ?goal crlf)
	(bind ?m (blackboard-create-msg "RLAgentGoalSelectionInterface::goal-selection" "GSelectionMessage"))
	(blackboard-set-msg-field ?m "goal" ?goal-id)
	(printout info "Calling rl plugin for goal '" ?goal "' and " ?goal-id crlf) ;$?executableGoals crlf)
	(bind ?gen-id (blackboard-send-msg ?m))
)

(defrule rl-waiting-assert
	(not (reset-domain-facts))
	(not (rl-waiting))
	?g <- (goal (mode FORMULATED))
	(not (goal (mode SELECTED|COMMITTED|DISPATCHED)))
=>
	(printout t crlf "Assert rl waiting" ?g crlf crlf)
	(assert (rl-waiting))
)

(defrule rl-execution-goal-selection
	(rl-waiting)
	(execution-mode)
	(not (executing-rl-agent))
	?g <- (goal (id ?id))
=>
	(printout t crlf "Start rl test thread goal selection function in execution mode" crlf )
	;(rl-goal-selection-start ?id " TEST-STRING-RL ") ;calling RL Plugin via CLIPS Feature function
	(rl-call ?id ?g)
	(assert (executing-rl-agent))
)


(defrule executing-rl-agent-retract
	?g<-(goal (id ?id) (mode SELECTED|DISPATCHED|COMMITTED))
	?r <- (executing-rl-agent)
	=>
	(printout t crlf "Retracting executing-rl-agent " ?r crlf "Goal: " ?g " id: " ?id crlf crlf)
	(retract ?r)
)

(defrule start-training-rl-agent
	(rl-waiting)
	(not (execution-mode))
	(not (training-started))
    (wm-fact (key refbox phase) (value PRODUCTION))
	?g <- (goal (id ?id))
=>
	(printout t crlf "Start rl test thread loop in training mode" crlf )
	;(rl-goal-selection-start ?id " TEST-STRING-RL ") ;calling RL Plugin via CLIPS Feature function
	(rl-call ?id ?g)
	(assert (training-started))
)

(defrule flush-executability
; for all goals except the next goal for selection
  ?r <- (rl-goal-selection (next-goal-id ?a))
 =>
  (printout t crlf "flush-executability execpt for " ?a crlf )
	
  (delayed-do-for-all-facts ((?g goal))
	(and (eq ?g:is-executable TRUE) (neq ?g:class SEND-BEACON) (neq ?g:id ?a))
	(modify ?g (is-executable FALSE))
  )
)

;TODO Check for mode if mode is FORMULATED then leaf it
;TODO add function for mode EVALUATED or rejected to assert:
;(printout t "Goal '" ?goal-id "' has failed (" ?outcome "), evaluating" crlf)
;(assert (rl-finished-goal (goal-id ?goal-id) (outcome ?outcome) (result 0)))
(defrule rl-clips-goal-selection
	?r <- (rl-goal-selection (next-goal-id ?a))
	?g <- (goal (id ?a) (mode ?m))
	=>
	(printout t crlf "in RL Plugin added fact: " ?r " with next action " ?a crlf )
	(printout t crlf "goal: " ?g "with in mode: "?m crlf crlf)
	(modify ?g (mode SELECTED))
	(retract ?r)
)

(defrule no-reset-on-start
	?r<-(reset-domain-facts)
	?n<-(no-reset-on-training-start)
	=>
	(retract ?r)
	(retract ?n)
	(assert (reset-domain-finish))	
)

(defrule delete-domain-facts-loaded
  ?f <- (domain-facts-loaded)
  =>
  (retract ?f)
)

(defrule delete-goals
  (reset-domain-facts)
  ?g <- (goal)
  =>
	;(printout t crlf "delete goal " ?g crlf crlf)
  (retract ?g)  
)

(defrule delete-goal-metas
  (reset-domain-facts)
  ?m <- (goal-meta)
  =>
	;(printout t crlf "delete goal-meta " ?m crlf crlf)
  (retract ?m)  
)

(defrule delete-domain-facts
  (reset-domain-facts)
  ?f <- (domain-fact)
  =>
	;(printout t crlf "delete domain fact " ?f crlf crlf)
  (retract ?f)  
)

(defrule delete-domain-objects
  (reset-domain-facts)
  ?o <- (domain-object)
  =>
	;(printout t crlf "delete domain object " ?o crlf crlf)
  (retract ?o)  
)

(defrule delete-domain-object-types
  (reset-domain-facts)
  ?o <- (domain-object-type)
  =>
	;(printout t crlf "delete domain object type" ?o crlf crlf)
  (retract ?o)  
)

(defrule delete-domain-object-is-of-types
  (reset-domain-facts)
  ?o <- (domain-object-is-of-type)
  =>
	;(printout t crlf "delete domain object-is-of-type" ?o crlf crlf)
  (retract ?o)  
)

(defrule delete-domain-operators
  (reset-domain-facts)
  ?o <- (domain-operator)
  =>
	;(printout t crlf "delete domain-operator " ?o crlf crlf)
  (retract ?o)  
)

(defrule delete-domain-predicate
  (reset-domain-facts)
  ?o <- (domain-predicate)
  =>
	;(printout t crlf "delete domain predicate " ?o crlf crlf)
  (retract ?o)  
)

; (defrule delete-plan-action
;   (reset-domain-facts)
;   ?o <- (plan-action)
;   =>
; 	;(printout t crlf "delete plan-action " ?o crlf crlf)
;   (retract ?o)  
; )

; (defrule delete-action-timer
;   (reset-domain-facts)
;   ?o <- (action-timer)
;   =>
; 	;(printout t crlf "delete action-timer " ?o crlf crlf)
;   (retract ?o)  
; )

(defrule reset-domain
  ?r<-(reset-domain-facts)
  (not (domain-fact))
  (not (goal))
  (not (goal-meta))
  (not (domain-object))
  (not (domain-predicate))
  ;(not (plan-action))
  ;(not (action-timer))
  ?d<- (domain-loaded)
  ?fl<-(domain-facts-loaded)
  ?g <-(goals-loaded)
  ?p <- (wm-fact (key refbox phase))
  =>
  (printout t crlf "reset domain running" crlf crlf)
  (retract ?d)
  (retract ?fl)
  (retract ?r)
  (retract ?g)
  (assert (executive-init))
  (assert (reset-domain-running))
  (modify ?p (value SETUP))
)

(defrule reset-running
  ?r <-(reset-domain-running)
  (domain-loaded)
  (domain-facts-loaded)
  (goals-loaded)
  ?p <- (wm-fact (key refbox phase))
  =>
  (printout t crlf "reset domain finish" crlf crlf)
  (retract ?r)
  (modify ?p (value PRODUCTION))
  (assert (reset-domain-finish))
)



