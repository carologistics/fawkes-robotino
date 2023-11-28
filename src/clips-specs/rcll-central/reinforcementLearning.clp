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

(defrule refbox-start-production-after-startup
  (declare (salience ?*SALIENCE-RESET-GAME-LOW*))
  (wm-fact (key refbox game-time) (values ?sec&:(> ?sec 30) ?))
  ;?r<-(reset-domain-facts)
  (not (reset-game (stage ?stage)))
  (wm-fact (id "/refbox/comm/peer-id/public") (value ?peer-id) (type INT))
  (wm-fact (id "/refbox/phase")  (value SETUP) )
  (wm-fact (id "/refbox/state")  (value ?v) )
  (wm-fact (key central agent robot args? r ?))
  (wm-fact (key domain fact mps-state args? m ?m-name s ?m-state) (type BOOL) (value TRUE))  
  ;(goal (class ENTER-FIELD) (id ?id) )
  ;(goal-meta (goal-id ?id) (assigned-to ?robot&~nil) )
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
  (printout error "Sent refbox start production " crlf)
)

; Defining templates and rules for goal selection with rl




(deftemplate rl-finished-goal
	(slot goal-id (type SYMBOL));
	(slot outcome (type SYMBOL));
	(slot result (type INTEGER));
  (slot team-points (type INTEGER));
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
	(not (goal (mode SELECTED|COMMITTED|DISPATCHED) (type ACHIEVE)))
=>
	(printout t crlf "Assert rl waiting" ?g crlf crlf)
	(assert (rl-waiting))
)

(defrule rl-execution-goal-selection
  (declare (salience ?*SALIENCE-RL-SELECTION*))
	;(rl-waiting)
	(execution-mode)
	(not (executing-rl-agent-started))
	; robot is waiting
	(wm-fact (key central agent robot-waiting args? r ?robot))
    ?g <- (goal (id ?goal-id) (class ?class&~EXPLORATION-MOVE) (sub-type SIMPLE) (mode FORMULATED) (is-executable TRUE)
                  (parent ?pid))
    (goal-meta (goal-id ?goal-id) (assigned-to ?robot&~central&~nil))
	(wm-fact (key refbox phase) (value PRODUCTION))
	;?g <- (goal (class ENTER-FIELD| PRODUCE-ORDER) (id ?id)); CENTRAL-RUN-PARALLEL-SUPPORT-ROOT-gen205 buffer cap,
	;mount-ring CENTRAL-RUN-ALL-PRODUCE-ORDER-gen235
=>
	(printout t crlf "Start rl test thread goal selection function in execution mode" crlf )
	;(rl-goal-selection-start ?id " TEST-STRING-RL ") ;calling RL Plugin via CLIPS Feature function
	(rl-call ?goal-id ?g)
	(assert (executing-rl-agent-started))
)


(defrule executing-rl-agent-retract
  (declare (salience ?*SALIENCE-RL-SELECTION*))
	(execution-mode)
	?e <- (executing-rl-agent-started)
	(rl-goal-selection (next-goal-id ?id))
	?g<-(goal (id ?id) (mode DISPATCHED|COMMITTED)(type ACHIEVE))
	=>
	(printout t crlf "Retracting executing-rl-agent " ?e crlf "Goal: " ?g " id: " ?id crlf crlf)
	(retract ?e)
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


(deffunction remove-robot-assignment-from-goal-meta (?goal)
  (if (not (do-for-fact ((?f goal-meta))
      (eq ?f:goal-id (fact-slot-value ?goal id))
      (modify ?f (assigned-to nil))
      ))
   then
    (printout t "Cannot find a goal meta fact for the goal " ?goal crlf)
  )
)

;TODO Check for mode if mode is FORMULATED then leaf it
;TODO add function for mode EVALUATED or rejected to assert:
;(printout t "Goal '" ?goal-id "' has failed (" ?outcome "), evaluating" crlf)
;(assert (rl-finished-goal (goal-id ?goal-id) (outcome ?outcome) (result 0)))
(defrule rl-clips-goal-selection
  (declare (salience ?*SALIENCE-RL-SELECTION*))
	?r <- (rl-goal-selection (next-goal-id ?a))
	?next-goal <- (goal (id ?a) (mode ?m&FORMULATED))
  (goal-meta (goal-id ?a) (assigned-to ?robot))
	=>
	(printout t crlf "in RL Plugin added fact: " ?r " with next action " ?a crlf )
	(printout t crlf "goal: " ?next-goal "with in mode: "?m crlf crlf)
	
	;(retract ?r)
  (modify ?next-goal (mode SELECTED))

  (delayed-do-for-all-facts ((?g goal))
		(and (eq ?g:is-executable TRUE) (neq ?g:class SEND-BEACON))
		(modify ?g (is-executable FALSE))
	)
  ; if it is actually a robot, remove all other assignments and the waiting status
	(if (and (neq ?robot central) (neq ?robot nil))
		then
		(delayed-do-for-all-facts ((?g goal))
			(and (eq ?g:mode FORMULATED) (not (eq ?g:type MAINTAIN))
			     (any-factp ((?gm goal-meta))
			                (and (eq ?gm:goal-id ?g:id)
			                     (eq ?gm:assigned-to ?robot))))
			(remove-robot-assignment-from-goal-meta ?g)
		)
		(do-for-fact ((?waiting wm-fact))
			(and (wm-key-prefix ?waiting:key (create$ central agent robot-waiting))
			     (eq (wm-key-arg ?waiting:key r) ?robot))
			(retract ?waiting)
		)
	)
)

;(wm-fact (key refbox game-time) (values ?game-time $?ms))
(defrule rl-selected-goal-finished
  (declare (salience ?*SALIENCE-RL-SELECTION*))
	?r <- (rl-goal-selection (next-goal-id ?goal-id))
	(goal (id ?goal-id) (class ?goal-class) (mode ?mode&FINISHED|EVALUATED) (outcome ?outcome))
  ?pm <- (wm-fact (id "/refbox/points/MAGENTA") (value ?mvalue))
  ?pc <- (wm-fact (id "/refbox/points/CYAN") (value ?cvalue) )
  ?tc <- (wm-fact (id "/refbox/team-color")  (value ?team-color) )
  ?gm <- (goal-meta (goal-id ?goal-id) (order-id ?order-id) (points ?points))
	=>
	(printout t crlf "Goal: " ?goal-id " is " ?mode crlf )

  (if (eq DELIVER ?goal-class)
  then
    (do-for-fact ((?wm-fact wm-fact)) (eq ?wm-fact:key (create$ order meta points-max args? ord ?order-id))
      (bind ?maxpoints ?wm-fact:value)
    )
    (if (<= ?points 5)
    then
      (bind ?points (round (* 0.25 ?maxpoints)))
    else
      (bind ?points ?maxpoints)
    )
  )

  (if (eq ?outcome COMPLETED) 
  then
    (bind ?result ?points)
  else
    (bind ?result 0)
  )
  
  (if (eq MOVE-OUT-OF-WAY ?goal-class)
  then
    (bind ?result 0)
  )
  
  (bind ?team-points 0)
  (if (eq ?team-color CYAN)
    then (bind ?team-points ?cvalue))
  (if (eq ?team-color MAGENTA)
    then (bind ?team-points ?mvalue))
  
	(printout t crlf "Points CYAN: " ?cvalue " Points MAGENTA: " ?mvalue " Result: " ?result crlf )
  (assert (rl-finished-goal (goal-id ?goal-id) (outcome ?outcome) (result ?result) (team-points ?team-points)))
	(retract ?r)
)


(defrule delete-all-rl-selections
  (declare (salience ?*SALIENCE-FIRST*))
  (rl-delete-selections)
  ?r <-(rl-goal-selection (next-goal-id ?goal-id))
  =>
  (assert (rl-finished-goal (goal-id ?goal-id) (outcome RESET) (result 0) (team-points 0)))
  (retract ?r)
)

(defrule finished-deleting-rl-selections
  (declare (salience ?*SALIENCE-FIRST*))
  ?r <- (rl-delete-selections)
  (not (rl-goal-selection))
  =>
  (retract ?r)
)


