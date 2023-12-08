; defined in util
; (deftemplate reset-game
; 	(slot stage (type SYMBOL));
; )


(defglobal
  ?*SALIENCE-RESET-GAME-HIGH* = ?*SALIENCE-HIGH*
  ?*SALIENCE-RESET-GAME-MIDDLE* = 800
  ?*SALIENCE-RESET-GAME-LOW* = 300
  ?*RESET-GAME-TIMER* = 1.0
)

; move to uitl?!
; (deffunction goal-reasoner-nuke-subtree (?goal)
;   "Remove an entire subtree."
;   (do-for-all-facts ((?child goal)) (eq ?child:parent (fact-slot-value ?goal id))
;     (goal-reasoner-nuke-subtree ?child)
;   )
;   (retract ?goal)
; )



(defrule assert-training-counter
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (not (training-counter (num ?n)))
	(no-reset-on-training-start)
	=>
  (assert (training-counter (num 0)))
)

(defrule no-reset-on-start
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
	;?r<-(reset-domain-facts)
  ?r <- (reset-game (stage STAGE-0))
	?n<-(no-reset-on-training-start)
  (training-counter (num 0))
	=>
	(retract ?r)
	(retract ?n)
	(assert (reset-game-finished))
)

(defrule reset-game-refbox-post-game
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  ?r <- (reset-game (stage STAGE-0) (stage-time ?t))
  (wm-fact (id "/refbox/comm/peer-id/public") (value ?peer-id) (type INT))
  (wm-fact (id "/refbox/phase")  (value PRODUCTION) )
  ?f<-(domain-facts-loaded)
  =>
  (bind ?prepare-phase (pb-create "llsf_msgs.SetGamePhase"))
  (pb-set-field ?prepare-phase "phase" POST_GAME)
  
  (pb-send ?peer-id ?prepare-phase)
  (pb-destroy ?prepare-phase)

  (modify ?r (stage STAGE-1) (stage-time (time)))
  (printout error "Change refbox phase to POST_GAME" crlf)
)


(defrule reset-game-assert-timer
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (time $?now)
  ?r <- (reset-game (stage STAGE-0|STAGE-1) (stage-time ?t))
  (not (timer (name reset-game-timer)))
=> 
  (assert (timer (name reset-game-timer)
          (time ?now) (seq 1)))
)

; (defrule reset-game-refbox-post-game-reached
;   ?r <- (reset-game (stage STAGE-2) (stage-time ?t))
;   ?s <- (send-to-post-game)
; =>
;   (retract ?s)
; )

(defrule reset-game-refbox-setup
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (time $?now)
  ;?r<-(reset-domain-facts)
  ?r <- (reset-game (stage STAGE-0|STAGE-1) (stage-time ?t))
  (wm-fact (id "/refbox/comm/peer-id/public") (value ?peer-id) (type INT))
  (wm-fact (id "/refbox/phase")  (value POST_GAME) )
  (wm-fact (id "/refbox/state")  (value ?v) )
  ?f<-(domain-facts-loaded)
  (not (changed-to-setup-phase))
  ?rtimer <-  (timer (name reset-game-timer)
                  (time $?reset-time&:(timeout ?now ?reset-time ?*RESET-GAME-TIMER*))
                (seq ?seq))
  
   ;(test (>= (time) (+ ?t 0.5)))
  =>
 	(printout t crlf "reset-game-refbox-setup - current state: " ?v " stage-time "?t  crlf)
  (bind ?prepare-phase (pb-create "llsf_msgs.SetGamePhase"))
  (bind ?prepare-state (pb-create "llsf_msgs.SetGameState"))
  (pb-set-field ?prepare-phase "phase" SETUP)
  (pb-set-field ?prepare-state "state" PAUSED )

  (pb-send ?peer-id ?prepare-phase)
  (pb-send ?peer-id ?prepare-state)

  (pb-destroy ?prepare-phase)
  (pb-destroy ?prepare-state)
  (assert (rl-delete-selections))
  (modify ?r (stage STAGE-1) (stage-time (time)))
  (modify ?rtimer (time ?now) (seq (+ ?seq 1)))
  (assert (changed-to-setup-phase))
  (printout error "Change refbox phase to SETUP - state PAUSED" crlf)
)

(defrule reset-game-stage-one
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  ;?r<-(reset-domain-facts)
  ?r <- (reset-game (stage STAGE-1))
  (wm-fact (id "/refbox/phase")  (value SETUP) )
  ?f<-(domain-facts-loaded)
  =>
  (retract ?f)
  (modify ?r (stage STAGE-2))
)

(defrule delete-changed-to-setup-phase
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (reset-game (stage STAGE-2))
  ?f <- (changed-to-setup-phase)
  =>
  (printout t crlf "delete changed-to-setup-phase fact " ?f crlf crlf)
  (retract ?f)
)

(defrule delete-produce-order-goals
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  ;(reset-domain-facts)
  (reset-game (stage STAGE-2))
  ?g <- (goal (class PRODUCE-ORDER))
  =>
   (printout t crlf "delete PRDOCUTION-ROOT goal " ?g crlf crlf)
   (goal-reasoner-nuke-subtree ?g)
  (retract ?g)  
)

(defrule delete-instruct-root-goal
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  ;(reset-domain-facts)
  (reset-game (stage STAGE-2))
  ?g <- (goal (class INSTRUCTION-ROOT))
  =>
   (printout t crlf "delete INSTRUCTION-ROOT goal " ?g crlf crlf)
   (goal-reasoner-nuke-subtree ?g)
  (retract ?g)  
)

(defrule delete-support-root-goal
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  ;(reset-domain-facts)
  (reset-game (stage STAGE-2))
  ?g <- (goal (class SUPPORT-ROOT))
  =>
   (printout t crlf "delete SUPPORT-ROOT goal " ?g crlf crlf)
   (goal-reasoner-nuke-subtree ?g)
  (retract ?g)  
)

(defrule delete-wait-root-goal
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  ;(reset-domain-facts)
  (reset-game (stage STAGE-2))
  ?g <- (goal (class WAIT-ROOT))
  =>
   (printout t crlf "delete WAIT-ROOT goal " ?g crlf crlf)
   (goal-reasoner-nuke-subtree ?g)
  (retract ?g)  
)

(defrule delete-simple-goals
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (reset-game (stage STAGE-2))
  ?g <- (goal (id ?goal-id) (class ?goal-class&ENTER-FIELD|BUFFER-CAP|DELIVER|MOUNT-RING|MOUNT-CAP|RESET-MPS))
  ;?m <- (goal-meta (goal-id ?goal-id))
  =>
  (retract ?g)
)

(defrule delete-plan
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (reset-game (stage STAGE-2))
  ?p <- (plan (goal-id ?goal-id))
  (not (goal (id ?goal-id)))
  =>
  (retract ?p)
)

(defrule delete-plan-action
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (reset-game (stage STAGE-2))
  ?p <- (plan-action (goal-id ?goal-id))
  (not (goal (id ?goal-id)))
  =>
  (retract ?p)
)

(defrule delete-metadata-prepare-mps
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (reset-game (stage STAGE-2))
  ?p <- (metadata-prepare-mps ?mps ?team-color ?peer-id $?instruction_info)
  =>
  (retract ?p)
)

(defrule delete-rl-goal-selection-fact
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (reset-game (stage STAGE-2))
  ?r <- (rl-goal-selection (next-goal-id ?id))
  =>
  (retract ?r)
)

(defrule delete-rl-waiting-fact
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (reset-game (stage STAGE-2))
  ?r <- (rl-waiting)
  =>
  (retract ?r)
)


(defrule delete-wm-robmem-sync-map-entry
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (reset-game (stage STAGE-2))
  ?f <- (wm-robmem-sync-map-entry (wm-fact-id ?id&:(str-prefix "/template/fact/" ?id))) 
  =>
  (retract ?f)
)

(defrule delete-grounded-pddl-formula
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (reset-game (stage STAGE-2))
  ?f <- (grounded-pddl-formula (id ?id))
  =>
  (retract ?f)
)

(defrule delete-grounded-pddl-predicate
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (reset-game (stage STAGE-2))
  ?f <- (grounded-pddl-predicate (id ?id))
  =>
  (retract ?f)
)

(defrule delete-confval-active-robots
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (reset-game (stage STAGE-2))
  ?cf <- (confval (path ?p&:(str-prefix ?*BBSYNC_PEER_CONFIG* ?p)))
  =>
  (retract ?cf)
)


(defrule delete-goal-meta
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (reset-game (stage STAGE-2))
  ;?g <- (goal (id ?goal-id) (class ?goal-class&ENTER-FIELD|BUFFER-CAP|DELIVER|MOUNT-RING|MOUNT-CAP))
  ?m <- (goal-meta (goal-id ?goal-id))
  (not (goal (id ?goal-id)))
  =>
  (retract ?m)
)

(defrule delete-wp-meta
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (reset-game (stage STAGE-2))
  ?nm <- (wm-fact (key wp meta next-machine args? wp ?wp))
=>
  (delayed-do-for-all-facts ((?wm-fact wm-fact))
    (wm-key-prefix ?wm-fact:key (create$ wp meta))
    (retract ?wm-fact)
  )
)

(defrule delete-monitoring-facts
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (reset-game (stage STAGE-2))
  (or (wm-fact (key monitoring action-retried ))
	    (wm-fact (key monitoring fail-goal))
      (wm-fact (key monitoring cleanup-wp)))
=>
  (delayed-do-for-all-facts ((?wm-fact wm-fact))
    (wm-key-prefix ?wm-fact:key (create$ monitoring action-retried))
    (retract ?wm-fact)
  )
  (delayed-do-for-all-facts ((?wm-fact wm-fact))
    (wm-key-prefix ?wm-fact:key (create$ monitoring fail-goal))
    (retract ?wm-fact)
  )
  (delayed-do-for-all-facts ((?wm-fact wm-fact))
    (wm-key-prefix ?wm-fact:key (create$ monitoring cleanup-wp))
    (retract ?wm-fact)
  )
)

(defrule delete-order-meta
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (reset-game (stage STAGE-2))
  ?ss <- (wm-fact (key order meta step-scored args? ord ?order step ?curr-step))
=>
  (delayed-do-for-all-facts ((?wm-fact wm-fact))
    (wm-key-prefix ?wm-fact:key (create$ order meta))
    (retract ?wm-fact)
  )
)

(defrule delete-wm-fact-request
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (reset-game (stage STAGE-2))
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
  (reset-game (stage STAGE-2))
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
  (reset-game (stage STAGE-2))
  ?r <-(wm-fact (key goal selection criterion $? args? $?) )
=>
  (delayed-do-for-all-facts ((?wm-fact wm-fact))
    (wm-key-prefix ?wm-fact:key (create$ goal selection criterion ))
    (retract ?wm-fact)
  )
  (retract ?r)
)

(defrule delete-order-quantity
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (reset-game (stage STAGE-2))
  (or ?r <-(wm-fact (key domain fact quantity-delivered $? args? $?) )
    ?r <-(wm-fact (key domain fact quantity-requested $? args? $?) )
  )
=>
  (retract ?r)
)



(defrule delete-saved-flag
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (reset-game (stage STAGE-2))
  ?r <-(saved)
=>
  (retract ?r)
)



(defrule reset-game-stage-two
	(declare (salience 200 )) ;?*SALIENCE-HIGH*))
  ?r <- (reset-game (stage STAGE-2))
	(not (goal (class SUPPORT-ROOT)))
  (not (wm-fact (key request $? args? $?)))
  (not (refbox-agent-task (robot ?robot) (task-id ?seq)))
  (or (not (wm-fact (key template fact goal args? id ?id)))
    (goal (id ?id))) ; ensure wm-sync-cleanup done
  (or (not (wm-fact (key template fact goal-meta args? id ?meta-id)))
    (goal (id meta-?id))) ; ensure wm-sync-cleanup done
  (or (not (goal-meta (goal-id ?goal-id)))
   (goal (id ?goal-id)))
  (not (wm-fact (key goal selection criterion $? args? $?) ))
  =>
  (modify ?r (stage STAGE-3))
)

; all facts for reset are deleted
(defrule save-to-file-STAGE-3-finished
  (declare (salience ?*SALIENCE-HIGH*))
  (reset-game (stage STAGE-3))
  (domain-wm-flushed)
  (not (saved))
  (training-counter (num ?c))
  =>
  (bind ?name (sym-cat log_stage3_facts_split_ ?c .txt))
  (save-facts  ?name)
  (printout t crlf "========== Split here >8 ==========" crlf)
  (assert (saved))
)

(defrule reset-game-stage-three
  (declare (salience ?*SALIENCE-RESET-GAME-MIDDLE*))
  ?r <- (reset-game (stage STAGE-3))
	(domain-wm-flushed)
  ?s <- (saved)
	=>
  (modify ?r (stage STAGE-4)) ; now initalizing the domain is started
  (retract ?s)
)

(defrule save-to-file-STAGE-4
  (declare (salience ?*SALIENCE-LOW*))
  (reset-game (stage STAGE-4))
  (not (saved))
  (training-counter (num ?c))
  =>
  (bind ?name (sym-cat log_stage4_facts_split_ ?c .txt))
  (save-facts  ?name)
  (assert (saved))
)


(defrule reset-game-refbox-setup-running
  (declare (salience ?*SALIENCE-RESET-GAME-MIDDLE*))
  ;?r<-(reset-domain-facts)
  (reset-game (stage STAGE-4))
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
  ?r<-(reset-game (stage STAGE-4))
  (domain-facts-loaded)
  ?s <- (saved)
	=>
 	(printout t crlf "reset-game-stage-four - domain-facts-loaded" crlf)
  (modify ?r (stage STAGE-5))
  (retract ?s)
)



(defrule reset-game-refbox-production-running
  (declare (salience ?*SALIENCE-RESET-GAME-MIDDLE*))
	;?r<-(reset-domain-facts)
  ?r<-(reset-game (stage STAGE-5))
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
  (modify ?r (stage STAGE-6))
)

(defrule save-to-file-STAGE-6
  (declare (salience ?*SALIENCE-HIGH*))
  (or (reset-game (stage STAGE-6))
      (reset-game-finished))
  (not (saved))
  ?t<-(training-counter (num ?c))
  =>
  (bind ?name (sym-cat log_stage6_facts_split_ ?c .txt))
  (save-facts  ?name)
  (modify ?t (num (+ ?c 1)))
  (assert (saved))
)

; reset finished if the first order is recieved from the refbox
(defrule reset-game-finished
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH* )) ;executability check runs before
  ?r<-(reset-game (stage STAGE-6))
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
