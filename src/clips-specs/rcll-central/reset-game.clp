; defined in util
; (deftemplate reset-game
; 	(slot stage (type SYMBOL));
; )

; move to uitl?!
(deffunction goal-reasoner-nuke-subtree (?goal)
  "Remove an entire subtree."
  (do-for-all-facts ((?child goal)) (eq ?child:parent (fact-slot-value ?goal id))
    (goal-reasoner-nuke-subtree ?child)
  )
  (retract ?goal)
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

(defrule delete-simple-goals
  (reset-game (stage STAGE-1-2))
  ?g <- (goal (id ?goal-id) (class ?goal-class&ENTER-FIELD|BUFFER-CAP|DELIVER|MOUNT-RING|MOUNT-CAP))
  ;?m <- (goal-meta (goal-id ?goal-id))
  =>
  (retract ?g)
)

(defrule delete-plan
  (reset-game (stage STAGE-1-2))
  ?p <- (plan (goal-id ?goal-id))
  (not (goal (id ?goal-id)))
  =>
  (retract ?p)
)

(defrule delete-plan-action
  (reset-game (stage STAGE-1-2))
  ?p <- (plan (goal-id ?goal-id))
  (not (goal (id ?goal-id)))
  =>
  (retract ?p)
)

(defrule delete-goal-meta
  (reset-game (stage STAGE-1-2))
  ;?g <- (goal (id ?goal-id) (class ?goal-class&ENTER-FIELD|BUFFER-CAP|DELIVER|MOUNT-RING|MOUNT-CAP))
  ?m <- (goal-meta (goal-id ?goal-id))
  (not (goal (id ?goal-id)))
  =>
  (retract ?m)
)

(defrule delete-wp-meta
  (reset-game (stage STAGE-1-2))
  ?nm <- (wm-fact (key wp meta next-machine args? wp ?wp))
=>
  (delayed-do-for-all-facts ((?wm-fact wm-fact))
    (wm-key-prefix ?wm-fact:key (create$ wp meta))
    (retract ?wm-fact)
  )
)

(defrule delete-order-meta
  (reset-game (stage STAGE-1-2))
  ?ss <- (wm-fact (key order meta step-scored args? ord ?order step ?curr-step))
=>
  (delayed-do-for-all-facts ((?wm-fact wm-fact))
    (wm-key-prefix ?wm-fact:key (create$ order meta))
    (retract ?wm-fact)
  )
)

(defrule delete-wm-fact-request
  (reset-game (stage STAGE-1-2))
  ;?ss <- (wm-fact (key request pay args? ord ?order m ?m ring ?ring seq ?seq prio ?prio))
  (wm-fact (key request $? args? $?) )
=>
  (delayed-do-for-all-facts ((?wm-fact wm-fact))
    (wm-key-prefix ?wm-fact:key (create$ request ))
    (retract ?wm-fact)
  )
)

(defrule reset-game-stage-xx
	;(reset-domain-facts)
  ?r <- (reset-game (stage STAGE-1-2))
	(not (goal (class SUPPORT-ROOT)))
	=>
  (modify ?r (stage STAGE-1))
)

(defrule save-f-to-file
  (reset-game (stage STAGE-1))
  (domain-wm-flushed)
  (not (saved))
  =>
  (save-facts log_stage1_facts.txt )
  (assert (saved))
)

(defrule reset-game-stage-two
  ?r <- (reset-game (stage STAGE-1))
	(domain-wm-flushed)
	=>
  (modify ?r (stage STAGE-2))
)

(defrule reset-game-finished
  ?r<-(reset-game (stage STAGE-4))
  (wm-fact (id "/refbox/phase")  (value PRODUCTION) )
 ; (wm-fact (id "/refbox/state")  (value RUNNNING) ) 
  (goal (class ENTER-FIELD))
  =>

	(printout t crlf "reset-game-finished- current state: RUNNING" crlf)
  (retract ?r)
	(assert (reset-game-finished))	
)