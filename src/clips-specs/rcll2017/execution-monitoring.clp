;A timeout for an action 
(deftemplate pending-timer
  (slot plan-id (type SYMBOL))
  (slot action-id(type NUMBER))
  (multislot timeout-time)
)

(defglobal
  ?*COMMON-TIMEOUT-DURATION* = 20
  ?*MPS-DOWN-TIMEOUT-DURATION* = 120
)

(defrule create-action-timeout
  (declare (salience 1))
  (plan-action (plan-id ?plan-id) (goal-id ?goal-id) 
  	(id ?id) (status PENDING)
  	(action-name ?action-name)
	(param-values $?param-values))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED))
  (test (neq ?goal-id BEACONACHIEVE))

  (not (pending-timer (plan-id ?plan-id) (action-id ?id)))
  (time $?now)	
  ;Maybe check for a DOWNED mps here?
  =>
  (bind ?sec (+ (nth$ 1 ?now) ?*COMMON-TIMEOUT-DURATION*))
  (bind $?timeout (create$ ?sec (nth$ 2 ?now)))
  (assert (pending-timer (plan-id ?plan-id) (action-id ?id) (timeout-time ?timeout)))
)

(defrule detect-timeout
  ?p <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
  	(id ?id) (status PENDING)
	(action-name ?action-name)
	(param-values $?param-values))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED))
  ?pt <- (pending-timer (plan-id ?plan-id) (action-id ?id) (timeout-time $?timeout))
  (time $?now)
  (test (and (> (nth$ 1 ?now) (nth$ 1 ?timeout)) (> (nth$ 2 ?now) (nth$ 2 ?timeout))))
  =>
  (printout error "Action was too long pending: " ?action-name crlf)
  (modify ?p (status FAILED))
  (retract ?pt)
)

(defrule remove-timer
  (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
	(id ?id) (status ?status&~PENDING)
	(action-name ?action-name)
	(param-values $?param-values))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED))
  ?pt <- (pending-timer (plan-id ?plan-id) (action-id ?id) (timeout-time $?timeout))
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
  ?pt <- (pending-timer (plan-id ?plan-id) (action-id ?id) (timeout-time $?timeout))
  =>
  (printout t "Detected that " ?mps " is down while " ?action-name " is waiting for it. Enhance timeout-timer" crlf)
  (bind ?timeout-longer (create$ (+ (nth$ 1 ?timeout) ?*MPS-DOWN-TIMEOUT-DURATION*) (nth$ 2 ?timeout)))
  (modify ?pt (timeout-time ?timeout-longer))
)

