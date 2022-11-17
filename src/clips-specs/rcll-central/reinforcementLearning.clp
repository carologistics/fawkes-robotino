; Defining templates and rules for goal selection with rl

(deftemplate rl-goal-selection
	(slot next-goal-id (type SYMBOL))
)

(deftemplate rl-waiting)

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
	?g <- (goal (id ?id))
=>
	(printout t crlf "Start rl test thread loop in training mode" crlf )
	;(rl-goal-selection-start ?id " TEST-STRING-RL ") ;calling RL Plugin via CLIPS Feature function
	(rl-call ?id ?g)
	(assert (training-started))
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

(defrule delete-domain-facts
  (reset-domain-facts)
  ?f <- (domain-fact)
  =>
	;(printout t crlf "delete domain fact " ?f crlf crlf)
  (retract ?f)  
)

(defrule delete-goals
  (reset-domain-facts)
  ?g <- (goal)
  =>
	;(printout t crlf "delete goal " ?g crlf crlf)
  (retract ?g)  
)

(defrule reset-domain
  ?r<-(reset-domain-facts)
  (not (domain-fact))
  (not (goal))
  ?d<- (domain-loaded)
  ?fl<-(domain-facts-loaded)
  ?g <-(goals-loaded)
  =>
  (printout t crlf "reset domain running" crlf crlf)
  (retract ?d)
  (retract ?fl)
  (retract ?r)
  (retract ?g)
  (assert (reset-domain-running))
)

(defrule reset-running
  ?r <-(reset-domain-running)
  (domain-loaded)
	(domain-facts-loaded)
  (goals-loaded)
  =>
  (printout t crlf "reset domain finish" crlf crlf)
  (retract ?r)
  (assert (reset-domain-finish))
)
