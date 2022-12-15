(defrule refbox-startup
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
	?r <- (rl-goal-selection (next-goal-id ?a))
	?g <- (goal (id ?a) (mode ?m))
	=>
	(printout t crlf "in RL Plugin added fact: " ?r " with next action " ?a crlf )
	(printout t crlf "goal: " ?g "with in mode: "?m crlf crlf)
	(modify ?g (mode SELECTED))
	(retract ?r)
)

(defrule no-reset-on-start
	;?r<-(reset-domain-facts)
  ?r <- (reset-game (stage STAGE-0))
	?n<-(no-reset-on-training-start)
	=>
	(retract ?r)
	(retract ?n)
	(assert (reset-game-finished))	
)

; (defrule delete-domain-facts-loaded
;   ?f <- (domain-facts-loaded)
;   =>
;   (retract ?f)
; )

(defrule reset-game-refbox-setup
  ;?r<-(reset-domain-facts)
  ?r <- (reset-game (stage STAGE-0))
  (wm-fact (id "/refbox/comm/peer-id/public") (value ?peer-id) (type INT))
  (wm-fact (id "/refbox/phase")  (value PRODUCTION) )
  (wm-fact (id "/refbox/state")  (value ?v) )
  ?f<-(domain-facts-loaded)
  ;WAIT_START
  =>
 	(printout t crlf "reset-game-refbox-setup - current state: " ?v crlf)
  (bind ?prepare-phase (pb-create "llsf_msgs.SetGamePhase"))
  (bind ?prepare-state (pb-create "llsf_msgs.SetGameState"))
  (pb-set-field ?prepare-phase "phase" SETUP)
  (pb-set-field ?prepare-state "state" PAUSED )
  ;WAIT_START

  (pb-send ?peer-id ?prepare-phase)
  (pb-send ?peer-id ?prepare-state)

  (pb-destroy ?prepare-phase)
  (pb-destroy ?prepare-state)

  (printout error "Change refbox phase to SETUP - state PAUSED" crlf)
)

(defrule reset-game-stage-one
  ;?r<-(reset-domain-facts)
  ?r <- (reset-game (stage STAGE-0))
  (wm-fact (id "/refbox/phase")  (value SETUP) )
  ?f<-(domain-facts-loaded)
  =>
  (retract ?f)
  (modify ?r (stage STAGE-1-2))
)

(defrule delete-produce-order-goals
  ;(reset-domain-facts)
  (reset-game (stage STAGE-1-2))
  ?g <- (goal (class PRODUCE-ORDER))
  =>
   (printout t crlf "delete PRDOCUTION-ROOT goal " ?g crlf crlf)
   (goal-reasoner-nuke-subtree ?g)
  (retract ?g)  
)

(defrule delete-instruct-root-goal
  ;(reset-domain-facts)
  (reset-game (stage STAGE-1-2))
  ?g <- (goal (class INSTRUCTION-ROOT))
  =>
   (printout t crlf "delete INSTRUCTION-ROOT goal " ?g crlf crlf)
   (goal-reasoner-nuke-subtree ?g)
  (retract ?g)  
)

(defrule delete-support-root-goal
  ;(reset-domain-facts)
  (reset-game (stage STAGE-1-2))
  ?g <- (goal (class SUPPORT-ROOT))
  =>
   (printout t crlf "delete SUPPORT-ROOT goal " ?g crlf crlf)
   (goal-reasoner-nuke-subtree ?g)
  (retract ?g)  
)

(defrule reset-game-stage-xx
	;(reset-domain-facts)
  ?r <- (reset-game (stage STAGE-1-2))
	(not (goal (class SUPPORT-ROOT)))
	=>
  (modify ?r (stage STAGE-1))
)


; (defrule domain-flush
;   (reset-game (stage STAGE-1)))
; 	(wm-fact (key cx identity))
; 	(wm-fact (key refbox phase) (value SETUP))
; 	=>
; 	(printout warn "Flushing domain!" crlf)
; 	(wm-robmem-flush)
; 	(do-for-all-facts ((?df domain-fact)) TRUE
; 	  (retract ?df)
; 	)
; 	(assert (domain-wm-flushed))
; )


; (defrule delete-order-information
;   (reset-domain-facts)
;   (reset-cx-stage-one)
;   (wm-fact (key domain fact order-complexity args? ord ?order-id comp ?complexity) (type BOOL) (value TRUE) )
;             (wm-fact (key domain fact order-base-color args? ord ?order-id col ?base) (type BOOL) (value TRUE) )
;             (wm-fact (key domain fact order-cap-color  args? ord ?order-id col ?cap) (type BOOL) (value TRUE) )
;             (wm-fact (key domain fact order-gate  args? ord ?order-id gate (sym-cat GATE- ?delivery-gate)) (type BOOL) (value TRUE) )
;             (wm-fact (key refbox order ?order-id quantity-requested) (type UINT) (value ?quantity-requested) )
;             (wm-fact (key domain fact quantity-delivered args? ord ?order-id team CYAN)
;                      (type UINT) (value 0))
;             (wm-fact (key domain fact quantity-delivered args? ord ?order-id team MAGENTA)
;                      (type UINT) (value 0))
;             (wm-fact (key refbox order ?order-id delivery-begin) (type UINT) (value ?begin) )
;             (wm-fact (key refbox order ?order-id delivery-end) (type UINT) (value ?end) )
;             )
;   =>
;    (printout t crlf "delete SUPPORT-ROOT goal " ?g crlf crlf)
;    (goal-reasoner-nuke-subtree ?g)
;   (retract ?g)  
; )

; (defrule rl-remove-goals
;   (reset-game (stage STAGE-1))
;   ?g <- (goal (id ?goal-id) (verbosity ?v)
;         (acquired-resources) (parent ?parent))
;   (not (goal (parent ?goal-id)))
;   (goal (id ?parent) (type MAINTAIN))
; =>
;   (delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?goal-id)
;     (delayed-do-for-all-facts ((?a plan-action)) (and (eq ?a:plan-id ?p:id) (eq ?a:goal-id ?goal-id))
;       (retract ?a)
;     )
;     (retract ?p)
;   )
;   (delayed-do-for-all-facts ((?f goal-meta)) (eq ?f:goal-id ?goal-id)
;     (retract ?f)
;   )
;   (retract ?g)
;   (printout (log-debug ?v) "Goal " ?goal-id " removed" crlf)
; )


; (defrule print-cx-identity
; 	?w <- (wm-fact (key cx identity))
; 	?r<-(reset-domain-facts)
; 	(not (print-cx-done))
; 	=>
;   	(printout t crlf "key cx identity: " ?w crlf)
; 	(assert (print-cx-done))
; )

; (defrule reset-game-stage-two
; 	;(reset-domain-facts)
;   ?r <- (reset-game (stage STAGE-1))
; 	(domain-wm-flushed)
; 	=>
;   (modify ?r (stage STAGE-2))
; )

(defrule reset-game-refbox-setup-running
  ;?r<-(reset-domain-facts)
  (reset-game (stage STAGE-2))
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
(defrule reset-game-stage-three
  ?r<-(reset-game (stage STAGE-2))
  (domain-facts-loaded)
	=>
 	(printout t crlf "reset-game-stage-three - domain-facts-loaded" crlf)
  (modify ?r (stage STAGE-3))
)


(defrule reset-game-refbox-production-running
	;?r<-(reset-domain-facts)
  (reset-game (stage STAGE-3))
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
)

(defrule reset-game-finished
  ?r<-(reset-game (stage STAGE-3))
  (wm-fact (id "/refbox/phase")  (value PRODUCTION) )
  (wm-fact (id "/refbox/state")  (value RUNNNING) ) 
  (goal (class ENTER-FIELD))
  =>

	(printout t crlf "reset-game-finished- current state: RUNNING" crlf)
  (retract ?r)
	(assert (reset-game-finished))	
)