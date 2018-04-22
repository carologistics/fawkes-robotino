;A timeout for an action 
(deftemplate pending-timer
  (slot plan-id (type SYMBOL))
  (slot action-id(type NUMBER))
  (multislot timeout-time)
  (multislot start-time)
  (slot status)
)

(deftemplate gripped-wp-at
  (slot wp)
  (slot r)
  (multislot pos)
  (multislot rot)
)

(defglobal
  ?*COMMON-TIMEOUT-DURATION* = 30
  ?*MPS-DOWN-TIMEOUT-DURATION* = 120
)

(defrule gripper-init
        (executive-init)
        (ff-feature-loaded blackboard)
        (not (gripper-blackboard-init))
	=>
        (blackboard-open-reading "AX12GripperInterface" "Gripper AX12")
	(assert (gripper-blackboard-init))
)



;React to broken mps
(defrule broken-mps-reject-goals
  (declare (salience 1))
  (wm-fact (key monitoring mps-reset) (type UNKNOWN) (value ?mps))
  ?g <- (goal (id ?goal-id) (mode FORMULATED|SELECTED|EXPANDED) (params $? ?mps $?))
  (plan (id ?plan-id) (goal-id ?goal-id))
  =>
  (modify ?g (mode REJECTED))
)

(defrule reset-prepare-action-on-downed
  (declare (salience 1))
  (wm-fact (key domain fact mps-state args? m ?mps s DOWN))
  ?pa <- (plan-action (id ?id) (goal-id ?goal-id)
        (plan-id ?plan-id)
        (action-name prepare-cs|prepare-rs|prepare-ds|prepare-bs)
        (status RUNNING)
	(param-values $? ?mps $?))
  ?ta <- (timer (name prepare-mps-abort-timer))
  ?ts <- (timer (name prepare-mps-send-timer))
  =>
  (modify ?pa (status FORMULATED))
  (retract ?ta ?ts)
)


(defrule broken-mps-add-flam
  (declare (salience 1))
  (wm-fact (key domain fact mps-state args? m ?mps s BROKEN))
  (not (wm-fact (key monitoring mps-reset) (type UNKNOWN) (value ?mps)))
  => 
  (assert (wm-fact (key monitoring mps-reset) (type UNKNOWN) (value ?mps)))
)

(defrule broken-mps-remove-facts
  (declare (salience 1))
  ?flag <- (wm-fact (key monitoring mps-reset) (type UNKNOWN) (value ?mps))
  (wm-fact (key domain fact mps-state args? m ?mps s ?s&~BROKEN))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED))
  (not (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
	(status ?status& : (and (neq ?status FINAL) (neq ?status FORMULATED) (neq ?status FAILED)))
	(param-values $? ?mps $?)))	
  (wm-fact (key domain fact mps-type args? m ?mps t ?type))
   =>
  (printout error "MPS " ?mps " was broken, cleaning up facts" crlf)
  (do-for-all-facts ((?wf wm-fact)) (and (neq (member$ ?mps (wm-key-args ?wf:key)) FALSE) 
					 (or
						(neq (member$ wp-at (wm-key-path ?wf:key)) FALSE)
						(neq (member$ bs-prepared-color (wm-key-path ?wf:key)) FALSE)
						(neq (member$ ds-prepared-gate (wm-key-path ?wf:key)) FALSE)
					 	(neq (member$ bs-prepared-side (wm-key-path ?wf:key)) FALSE)
						(neq (member$ cs-prepared-for (wm-key-path ?wf:key)) FALSE)
						(neq (member$ cs-buffered (wm-key-path ?wf:key)) FALSE)
						(neq (member$ cs-can-perform (wm-key-path ?wf:key)) FALSE)
						(neq (member$ rs-filled-with (wm-key-path ?wf:key)) FALSE) 
						(neq (member$ rs-prepared-color (wm-key-path ?wf:key)) FALSE)
					 )
		  		    )			
		(retract ?wf)
		(printout t "CLEANED: " ?wf:key crlf)
  )
  (switch ?type
	(case CS then 
		(assert (wm-fact (key domain fact cs-can-perform args? m ?mps op RETRIEVE_CAP)))
	)
	(case RS then
		(assert (wm-fact (key domain fact rs-filled-with args? m ?mps n ZERO)))
	)
  )	
  (retract ?flag)
)


(defrule create-action-timeout
  (declare (salience 1))
  (plan-action (plan-id ?plan-id) (goal-id ?goal-id) 
  	(id ?id) (status ?status& : (and (neq ?status FORMULATED) (neq ?status RUNNING) (neq ?status FAILED) (neq ?status FINAL)))
  	(action-name ?action-name)
	(param-values $?param-values))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED))
  (test (neq ?goal-id BEACONACHIEVE))

  (not (pending-timer (plan-id ?plan-id) (action-id ?id) (status ?status)))
  (time $?now)	
  ;Maybe check for a DOWNED mps here?
  =>
  (bind ?sec (+ (nth$ 1 ?now) ?*COMMON-TIMEOUT-DURATION*))
  (bind $?timeout (create$ ?sec (nth$ 2 ?now)))
  (assert (pending-timer (plan-id ?plan-id) (action-id ?id) (timeout-time ?timeout) (status ?status) (start-time ?now)))
)

(defrule detect-timeout
  ?p <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
  	(id ?id) (status ?status)
	(action-name ?action-name)
	(param-values $?param-values))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED))
  ?pt <- (pending-timer (plan-id ?plan-id) (status ?status) (action-id ?id) (timeout-time $?timeout))
  (time $?now)
  (test (and (> (nth$ 1 ?now) (nth$ 1 ?timeout)) (> (nth$ 2 ?now) (nth$ 2 ?timeout))))
  =>
  (printout error "Action was too long pending: " ?action-name crlf)
  (modify ?p (status FAILED))
  (retract ?pt)
)

(defrule remove-timer
  (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
	(id ?id) (status ?status)
	(action-name ?action-name)
	(param-values $?param-values))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED))
  ?pt <- (pending-timer (plan-id ?plan-id) (action-id ?id) (status ?st& : (neq ?st ?status)) (timeout-time $?timeout))
  =>
  (retract ?pt)
)

(defrule enhance-timer-on-downed-mps
  (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
	(id ?id) (status PENDING)
	(action-name ?action-name)
	(param-values $? ?mps $?))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED))
  
  (wm-fact (key domain fact mps-state args? m ?mps s DOWN))
  ?pt <- (pending-timer (plan-id ?plan-id) (action-id ?id) (start-time $?starttime) (timeout-time $?timeout))
  (test (< (nth$ 1 ?timeout) (+ (nth$ 1 ?starttime) ?*MPS-DOWN-TIMEOUT-DURATION* ?*COMMON-TIMEOUT-DURATION*)))
  =>
  (printout t "Detected that " ?mps " is down while " ?action-name " is waiting for it. Enhance timeout-timer" crlf)
  (bind ?timeout-longer (create$ (+ (nth$ 1 ?timeout) ?*MPS-DOWN-TIMEOUT-DURATION*) (nth$ 2 ?timeout)))
  (modify ?pt (timeout-time ?timeout-longer))
)

(defrule cleanup-after-wp-put
  (declare (salience 1))
  (plan-action (id ?id) (goal-id ?goal-id)
	(plan-id ?plan-id)
	(action-name ?an& : (or (eq ?an wp-put) (eq ?an wp-put-slide-cc)))
	(param-values ?r ?wp ?mps $?)
	(status FAILED))
  (plan (id ?plan-id) (goal-id ?goal-id))
  ?g <- (goal (id ?goal-id) (mode FINISHED) (outcome FAILED))	
  ?hold <- (wm-fact (key domain fact holding args? r ?r wp ?wp))
  (AX12GripperInterface (holds_puck ?holds))
  =>
  (if (eq ?holds TRUE)
      then
      (retract ?hold)
      (assert (domain-fact (name can-hold) (param-values ?r)))
  )
  (printout t "Goal " ?goal-id " failed because of " ?an " and is evaluated" crlf)
  (modify ?g (mode EVALUATED))
)

(defrule cleanup-mps-output
  (declare (salience 1))
  (wm-fact (key domain fact mps-state args? m ?mps s ?s& : (and (neq ?s READY-AT-OUTPUT) (neq ?s DOWN))))
  ?wpat <- (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side OUTPUT))
  =>
  ;TODO: Send Maintenance message
  (printout "Cleaned up wp-at fact because the mps-state did not match" crlf)
  (retract ?wpat)
)

(defrule cleanup-get-shelf-failed
  (declare (salience 1))
   (plan-action (id ?id) (goal-id ?goal-id)
	(plan-id ?plan-id)
	(action-name wp-get-shelf)
	(param-values ?r ?wp ?mps ?spot)
	(status FAILED))
  (plan (id ?plan-id) (goal-id ?goal-id))
  ?g <- (goal (id ?goal-id) (mode FINISHED) (outcome FAILED))	
  ?hold <- (wm-fact (key domain fact wp-on-shelf args? wp ?wp m ?mps spot ?spot))
  =>
  (printout t "Goal " ?goal-id " has been failed because of wp-get-shelf and is evaluated")
  (retract ?hold)
  (modify ?g (mode EVALUATED))
  (assert (domain-fact (name can-hold) (param-values ?r)))
)

(defrule common-failed-evaluation
  ?g <- (goal (id ?goal-id) (mode FINISHED) (outcome FAILED))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (plan-action (action-name ?an) (goal-id ?goal-id) (status FAILED))
  =>
  (printout t "Goal " ?goal-id " has been failed because of " ?an " and is evaluated" crlf)
  (modify ?g (mode EVALUATED))
)

