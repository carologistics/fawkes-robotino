;A timeout for an action 
(deftemplate pending-timer
  (slot plan-id (type SYMBOL))
  (slot action-id(type NUMBER))
  (multislot timeout-time)
  (multislot start-time)
  (slot status)
)

(defglobal
  ?*COMMON-TIMEOUT-DURATION* = 30
  ?*MPS-DOWN-TIMEOUT-DURATION* = 120
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

(defrule broken-mps-add-flag
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

;DONT DO THAT
(defrule cleanup-facts-wp-get-shelf
  (declare (salience 1))
  (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
	(status FAILED)
	(action-name wp-get-shelf)
	(param-values ?r ?cc ?m ?side))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED))
  ?shelf <- (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?m spot ?side))
  ?wp-cap <- (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?col))
  =>
  (retract ?shelf)
  (retract ?wp-cap)
) 

;DONT DO THAT
(defrule cleanup-facts-wp-get-remove-wp-facts
  (declare (salience 1))
  (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
	(status FAILED)
	(action-name wp-get)
	(param-values ?r ?cc ?m ?side))
  (wm-fact (key domain fact mps-state args? m ?m s IDLE))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED))
  ?wm <- (wm-fact (key domain fact ?fact args? $? wp ?cc $?))
  =>
  (retract ?wm)
)

;DONT DO THAT
(defrule cleanup-facts-wp-put-remove-wp-facts
  (declare (salience 1))
  (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
	(status FAILED)
	(action-name wp-put)
	(param-values ?r ?cc ?m))
  (wm-fact (key domain fact mps-state args? m ?m s PREPARED))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED))
  ?wf <- (wm-fact (key domain fact ?fact args? $? wp ?cc $?))
  =>
  (retract ?wf)
  (assert (domain-fact (name can-hold) (param-values ?r))) 
)

