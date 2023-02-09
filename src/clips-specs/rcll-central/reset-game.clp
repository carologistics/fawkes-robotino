; defined in util
; (deftemplate reset-game
; 	(slot stage (type SYMBOL));
; )


(defglobal
  ?*SALIENCE-RESET-GAME-HIGH* = ?*SALIENCE-HIGH*
  ?*SALIENCE-RESET-GAME-MIDDLE* = 800
  ?*SALIENCE-RESET-GAME-LOW* = 300
)

; move to uitl?!
(deffunction goal-reasoner-nuke-subtree (?goal)
  "Remove an entire subtree."
  (do-for-all-facts ((?child goal)) (eq ?child:parent (fact-slot-value ?goal id))
    (goal-reasoner-nuke-subtree ?child)
  )
  (retract ?goal)
)

(defrule no-reset-on-start
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
	;?r<-(reset-domain-facts)
  ?r <- (reset-game (stage STAGE-0))
	?n<-(no-reset-on-training-start)
	=>
	(retract ?r)
	(retract ?n)
	(assert (reset-game-finished))	
)

(defrule reset-game-refbox-setup
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  ;?r<-(reset-domain-facts)
  ?r <- (reset-game (stage STAGE-0))
  (wm-fact (id "/refbox/comm/peer-id/public") (value ?peer-id) (type INT))
  (wm-fact (id "/refbox/phase")  (value PRODUCTION|POST_GAME) )
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
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  ;?r<-(reset-domain-facts)
  ?r <- (reset-game (stage STAGE-0))
  (wm-fact (id "/refbox/phase")  (value SETUP) )
  ?f<-(domain-facts-loaded)
  =>
  (retract ?f)
  (modify ?r (stage STAGE-1))
)

(defrule delete-produce-order-goals
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  ;(reset-domain-facts)
  (reset-game (stage STAGE-1))
  ?g <- (goal (class PRODUCE-ORDER))
  =>
   (printout t crlf "delete PRDOCUTION-ROOT goal " ?g crlf crlf)
   (goal-reasoner-nuke-subtree ?g)
  (retract ?g)  
)

(defrule delete-instruct-root-goal
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  ;(reset-domain-facts)
  (reset-game (stage STAGE-1))
  ?g <- (goal (class INSTRUCTION-ROOT))
  =>
   (printout t crlf "delete INSTRUCTION-ROOT goal " ?g crlf crlf)
   (goal-reasoner-nuke-subtree ?g)
  (retract ?g)  
)

(defrule delete-support-root-goal
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  ;(reset-domain-facts)
  (reset-game (stage STAGE-1))
  ?g <- (goal (class SUPPORT-ROOT))
  =>
   (printout t crlf "delete SUPPORT-ROOT goal " ?g crlf crlf)
   (goal-reasoner-nuke-subtree ?g)
  (retract ?g)  
)

(defrule delete-wait-root-goal
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  ;(reset-domain-facts)
  (reset-game (stage STAGE-1))
  ?g <- (goal (class WAIT-ROOT))
  =>
   (printout t crlf "delete WAIT-ROOT goal " ?g crlf crlf)
   (goal-reasoner-nuke-subtree ?g)
  (retract ?g)  
)

(defrule delete-simple-goals
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (reset-game (stage STAGE-1))
  ?g <- (goal (id ?goal-id) (class ?goal-class&ENTER-FIELD|BUFFER-CAP|DELIVER|MOUNT-RING|MOUNT-CAP))
  ;?m <- (goal-meta (goal-id ?goal-id))
  =>
  (retract ?g)
)

(defrule delete-plan
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (reset-game (stage STAGE-1))
  ?p <- (plan (goal-id ?goal-id))
  (not (goal (id ?goal-id)))
  =>
  (retract ?p)
)

(defrule delete-plan-action
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (reset-game (stage STAGE-1))
  ?p <- (plan-action (goal-id ?goal-id))
  (not (goal (id ?goal-id)))
  =>
  (retract ?p)
)

(defrule delete-rl-goal-selection-fact
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (reset-game (stage STAGE-1))
  ?r <- (rl-goal-selection )
  =>
  (retract ?r)
)

(defrule delete-rl-waiting-fact
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (reset-game (stage STAGE-1))
  ?r <- (rl-waiting)
  =>
  (retract ?r)
)


(defrule delete-goal-meta
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (reset-game (stage STAGE-1))
  ;?g <- (goal (id ?goal-id) (class ?goal-class&ENTER-FIELD|BUFFER-CAP|DELIVER|MOUNT-RING|MOUNT-CAP))
  ?m <- (goal-meta (goal-id ?goal-id))
  (not (goal (id ?goal-id)))
  =>
  (retract ?m)
)

(defrule delete-wp-meta
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (reset-game (stage STAGE-1))
  ?nm <- (wm-fact (key wp meta next-machine args? wp ?wp))
=>
  (delayed-do-for-all-facts ((?wm-fact wm-fact))
    (wm-key-prefix ?wm-fact:key (create$ wp meta))
    (retract ?wm-fact)
  )
)

(defrule delete-order-meta
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (reset-game (stage STAGE-1))
  ?ss <- (wm-fact (key order meta step-scored args? ord ?order step ?curr-step))
=>
  (delayed-do-for-all-facts ((?wm-fact wm-fact))
    (wm-key-prefix ?wm-fact:key (create$ order meta))
    (retract ?wm-fact)
  )
)

(defrule delete-wm-fact-request
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (reset-game (stage STAGE-1))
  ;?ss <- (wm-fact (key request pay args? ord ?order m ?m ring ?ring seq ?seq prio ?prio))
  (wm-fact (key request $? args? $?) )
=>
  (delayed-do-for-all-facts ((?wm-fact wm-fact))
    (wm-key-prefix ?wm-fact:key (create$ request ))
    (retract ?wm-fact)
  )
)

(defrule delete-refbox-agent-task
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (reset-game (stage STAGE-1))
  (refbox-agent-task (robot ?robot) (task-id ?seq))
  ;(refbox-agent-task (robot ?robot) (task-id ?seq)
  ;(goal-id ?goal-id) (plan-id ?plan-id) (action-id ?action-id)
  ;)
  =>
  (delayed-do-for-all-facts ((?task refbox-agent-task)) TRUE
    (retract ?task)
  )

)

; (wm-fact (id "/goal/selection/criterion?t=root")
; (key goal selection criterion args? t root) (type SYMBOL) (is-list TRUE)
; (value nil) 
;(values ENTER-FIELD-gen188 CENTRAL-RUN-PARALLEL-SUPPORT-ROOT-gen190 CENTRAL-RUN-ALL-PRODUCE-ORDER-g
(defrule delete-goal-selection-criterion
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (reset-game (stage STAGE-1))
  ?r <-(wm-fact (key goal selection criterion $? args? $?) )
=>
  (delayed-do-for-all-facts ((?wm-fact wm-fact))
    (wm-key-prefix ?wm-fact:key (create$ goal selection criterion ))
    (retract ?wm-fact)
  )
  (retract ?r)
)


(defrule reset-game-stage-two
	(declare (salience 200 )) ;?*SALIENCE-HIGH*))
  ?r <- (reset-game (stage STAGE-1))
	(not (goal (class SUPPORT-ROOT)))
  (not (wm-fact (key request $? args? $?)))
  (not (refbox-agent-task (robot ?robot) (task-id ?seq)))
  (or (not (wm-fact (key template fact goal args? id ?id)))
    (goal (id ?id))) ; ensure wm-sync-cleanup done
  (or (not (wm-fact (key template fact goal-meta args? id ?meta-id)))
    (goal (id meta-?id))) ; ensure wm-sync-cleanup done
  (or (not (goal-meta (goal-id ?goal-id)))
   (goal (id ?goal-id)))
  =>
  (modify ?r (stage STAGE-2))
)

(defrule save-to-file-stage-2-finished
  (declare (salience ?*SALIENCE-HIGH*))
  (reset-game (stage STAGE-2))
  (domain-wm-flushed)
  (not (saved))
  =>
  (save-facts log_stage2_facts.txt )
  (assert (saved))
)

(defrule reset-game-stage-three
  (declare (salience ?*SALIENCE-RESET-GAME-MIDDLE*))
  ?r <- (reset-game (stage STAGE-2))
	(domain-wm-flushed)
  ?s <- (saved)
	=>
  (modify ?r (stage STAGE-3))
  (retract ?s)
)

(defrule save-to-file-stage-3
  (declare (salience ?*SALIENCE-LOW*))
  (reset-game (stage STAGE-3))
  (not (saved))
  =>
  (save-facts log_stage3_facts.txt )
  (assert (saved))
)


(defrule reset-game-refbox-setup-running
  (declare (salience ?*SALIENCE-RESET-GAME-MIDDLE*))
  ;?r<-(reset-domain-facts)
  (reset-game (stage STAGE-3))
  (wm-fact (id "/refbox/comm/peer-id/public") (value ?peer-id) (type INT))
  (wm-fact (id "/refbox/phase")  (value SETUP) )
  (wm-fact (id "/refbox/state")  (value ?v) )
  (saved)
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
  ?s <- (saved)
	=>
 	(printout t crlf "reset-game-stage-four - domain-facts-loaded" crlf)
  (modify ?r (stage STAGE-4))
  (retract ?s)
)


(defrule reset-game-refbox-production-running
  (declare (salience ?*SALIENCE-RESET-GAME-MIDDLE*))
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

(defrule save-to-file-stage-5
  (declare (salience ?*SALIENCE-HIGH*))
  (or (reset-game (stage STAGE-5))
      (reset-game-finished))
  (not (saved))
  =>
  (save-facts log_stage5_facts.txt )
  (assert (saved))
)

; reset finished if the first order is recieved from the refbox
(defrule reset-game-finished
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH* )) ;executability check runs before
  ?r<-(reset-game (stage STAGE-5))
  (wm-fact (id "/refbox/phase")  (value PRODUCTION) )
 ; (wm-fact (id "/refbox/state")  (value RUNNNING) ) 
  (goal (class ENTER-FIELD) (id ?id) (is-executable TRUE))
  (goal-meta (goal-id ?id) (assigned-to ~nil) )
  ;(goal (class PRODUCE-ORDER))
  =>
	(printout t crlf "reset-game-finished- current state: RUNNING" crlf)
  (retract ?r)
	(assert (reset-game-finished))	
)