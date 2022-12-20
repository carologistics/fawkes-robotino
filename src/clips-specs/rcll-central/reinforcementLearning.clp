(defrule refbox-startup
  (declare (salience ?*SALIENCE-LOW*))
  (wm-fact (id "/refbox/comm/peer-id/public") (value ?peer-id) (type INT))
  (wm-fact (id "/refbox/phase")  (value PRE_GAME) )
  (wm-fact (id "/refbox/state")  (value WAIT_START) )
  =>
  (bind ?prepare-team (pb-create "llsf_msgs.SetTeamName"))
  ;(bind ?prepare-phase (pb-create "llsf_msgs.SetGamePhase"))
  (bind ?prepare-state (pb-create "llsf_msgs.SetGameState"))
  (pb-set-field ?prepare-team "team_name" Carologistics)
  (pb-set-field ?prepare-team "team_color" CYAN)
  ;(pb-set-field ?prepare-phase "phase" SETUP)
  (pb-set-field ?prepare-state "state" RUNNING)

  (pb-send ?peer-id ?prepare-team)
  (pb-send ?peer-id ?prepare-state)
  ;(pb-send ?peer-id ?prepare-phase)

  (pb-destroy ?prepare-team)
  ;(pb-destroy ?prepare-phase)
  (pb-destroy ?prepare-state)
  (printout error "Sent refbox init " crlf)
)

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

(defglobal
  ?*SALIENCE-RL-SELECTION* = ?*SALIENCE-HIGH*
)

(defrule rl-waiting-assert
  (declare (salience ?*SALIENCE-RL-SELECTION*))
	(not (reset-domain-facts))
	(not (rl-waiting))
  (wm-fact (id "/refbox/phase")  (value PRODUCTION) )
  (wm-fact (id "/refbox/state")  (value RUNNING) )
	?g <- (goal (mode FORMULATED))
	(not (goal (mode SELECTED|COMMITTED|DISPATCHED)))
=>
	(printout t crlf "Assert rl waiting" ?g crlf crlf)
	(assert (rl-waiting))
)

(defrule rl-execution-goal-selection
  (declare (salience ?*SALIENCE-RL-SELECTION*))
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
  (declare (salience ?*SALIENCE-RL-SELECTION*))
	?g<-(goal (id ?id) (mode SELECTED|DISPATCHED|COMMITTED))
	?r <- (executing-rl-agent)
	=>
	(printout t crlf "Retracting executing-rl-agent " ?r crlf "Goal: " ?g " id: " ?id crlf crlf)
	(retract ?r)
)

(defrule start-training-rl-agent
  (declare (salience ?*SALIENCE-RL-SELECTION*))
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

; (defrule flush-executability
; ; for all goals except the next goal for selection
;   ?r <- (rl-goal-selection (next-goal-id ?a))
;  =>
;   (printout t crlf "flush-executability execpt for " ?a crlf )
	
;   (delayed-do-for-all-facts ((?g goal))
; 	(and (eq ?g:is-executable TRUE) (neq ?g:class SEND-BEACON) (neq ?g:id ?a))
; 	(modify ?g (is-executable FALSE))
;   )
; )

;TODO Check for mode if mode is FORMULATED then leaf it
;TODO add function for mode EVALUATED or rejected to assert:
;(printout t "Goal '" ?goal-id "' has failed (" ?outcome "), evaluating" crlf)
;(assert (rl-finished-goal (goal-id ?goal-id) (outcome ?outcome) (result 0)))
(defrule rl-clips-goal-selection
  (declare (salience ?*SALIENCE-RL-SELECTION*))
	?r <- (rl-goal-selection (next-goal-id ?a))
	?g <- (goal (id ?a) (mode ?m&FORMULATED))
	=>
	(printout t crlf "in RL Plugin added fact: " ?r " with next action " ?a crlf )
	(printout t crlf "goal: " ?g "with in mode: "?m crlf crlf)
	(modify ?g (mode SELECTED))
	;(retract ?r)
)

;(wm-fact (key refbox game-time) (values ?game-time $?ms))
(defrule rl-selected-goal-finished
  (declare (salience ?*SALIENCE-RL-SELECTION*))
	?r <- (rl-goal-selection (next-goal-id ?goal-id))
	(goal (id ?goal-id) (mode ?mode&FINISHED|EVALUATED) (outcome ?outcome))
  ?pm <- (wm-fact (id "/refbox/points/magenta") (value ?mvalue))
  ?pc <- (wm-fact (id "/refbox/points/cyan") (value ?cvalue) )
  ?tc <- (wm-fact (id "/refbox/team-color")  (value ?team-color) )
	=>
	(printout t crlf "Goal: " ?goal-id " is " ?mode crlf )
  (bind ?result 1)
  (if (eq ?team-color CYAN)
    then (bind ?result ?cvalue))
  (if (eq ?team-color MAGENTA)
    then (bind ?result ?mvalue))
  
	(printout t crlf "Points CYAN: " ?cvalue " Points MAGENTA: " ?mvalue " Result: " ?result crlf )
  (assert (rl-finished-goal (goal-id ?goal-id) (outcome ?outcome) (result ?result)))
	(retract ?r)
)



(defrule reset-game-refbox-setup-running
  (declare (salience ?*SALIENCE-RL-SELECTION*))
  ;?r<-(reset-domain-facts)
  (reset-game (stage STAGE-3))
  (wm-fact (id "/refbox/comm/peer-id/public") (value ?peer-id) (type INT))
  (wm-fact (id "/refbox/phase")  (value SETUP) )
  (wm-fact (id "/refbox/state")  (value ?v) )
  =>
 	(printout t crlf "reset-game-refbox-setup-running - current state: " ?v crlf)
  ;(bind ?prepare-phase (pb-create "llsf_msgs.SetGamePhase"))
  (bind ?prepare-state (pb-create "llsf_msgs.SetGameState"))
  ;(pb-set-field ?prepare-phase "phase" SETUP)
  (pb-set-field ?prepare-state "state" RUNNING)

  (pb-send ?peer-id ?prepare-state)
  ;(pb-send ?peer-id ?prepare-phase)

  (pb-destroy ?prepare-state)
  (printout error "Change refbox phase SETUP - state RUNNING" crlf)
)

; Waiting till initial facts are loaded
(defrule reset-game-stage-four
  (declare (salience ?*SALIENCE-LOW*))
  ?r<-(reset-game (stage STAGE-3))
  (domain-facts-loaded)
	=>
 	(printout t crlf "reset-game-stage-four - domain-facts-loaded" crlf)
  (modify ?r (stage STAGE-4))
)


(defrule reset-game-refbox-production-running
  (declare (salience ?*SALIENCE-RL-SELECTION*))
	;?r<-(reset-domain-facts)
  ?r<-(reset-game (stage STAGE-4))
	(wm-fact (id "/refbox/comm/peer-id/public") (value ?peer-id) (type INT))
  (wm-fact (id "/refbox/phase")  (value SETUP) )
  (wm-fact (id "/refbox/state")  (value ?v) ) 

=>
	(printout t crlf "reset-game-refbox-production-running - current state: " ?v crlf)
  (bind ?prepare-phase (pb-create "llsf_msgs.SetGamePhase"))
  (bind ?prepare-state (pb-create "llsf_msgs.SetGameState"))
  (pb-set-field ?prepare-phase "phase" PRODUCTION)
  (pb-set-field ?prepare-state "state" RUNNING)

  (pb-send ?peer-id ?prepare-phase)
  (pb-send ?peer-id ?prepare-state)

  (pb-destroy ?prepare-phase)
  (pb-destroy ?prepare-state)
  (modify ?r (stage STAGE-5))
)
