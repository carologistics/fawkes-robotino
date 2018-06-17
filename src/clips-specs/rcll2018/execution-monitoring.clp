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
  ?*HOLDING-MONITORING* = 60
)

;Execution Monitoring MPS state
(defrule execution-monitoring-incosistent-yet-exepected-mps-state-ready-at-output
  (declare (salience 1))
  (domain-pending-sensed-fact
    (goal-id ?goal-id)
    (action-id ?action-id)
    (name mps-state)
    (param-values ?mps READY-AT-OUTPUT)
    (type POSITIVE))
  (wm-fact (key domain fact mps-state args? m ?mps s READY-AT-OUTPUT))
  (not (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side OUTPUT)))
  =>
  ;TODO: Send Maintenance message
  (printout warn "Monitoring: MPS state READY-AT-OUTPUT but no WP at output, Yet action " ?action-id " in Goal " ?goal-id
    "expected it!" crlf)
  (bind ?wp-gen  (sym-cat WP- (gensym)))
  (assert (domain-object (name (sym-cat WP- (gensym))) (type workpiece))
          (wm-fact (key domain fact wp-at args? wp ?wp-gen m ?mps side OUTPUT))
          (wm-fact (key domain fact wp-usable args? wp ?wp-gen))
          )
  ;TODO..check if it exists somewhere that will pervent execusion
  (printout warn "A WP has been Generated at the OUTPUT side" crlf)
)

;(defrule execution-monitoring-incosistent-yet-exepected-mps-state-idle
;  (declare (salience 1))
;  (domain-pending-sensed-fact
;    (goal-id ?goal-id)
;    (action-id ?action-id)
;    (name mps-state)
;    (param-values ?mps IDLE)
;    (type POSITIVE))
;  (wm-fact (key domain fact mps-state args? m ?mps IDLE))
;  ?wpat <- (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side OUTPUT))
;  =>
;  ;TODO: Send Maintenance message
;  (printout warn "Monitoring: MPS state IDLE but WP exists at output, Yet action " ?action-id " in Goal " ?goal-id
;    "expected it!!" crlf)
;  (assert (wm-fact (key monitoring cleanup-wp args? wp ?wp)))
;  (printout warn "The WP has been retracted!!" crlf)
;)


(defrule reset-prepare-action-on-downed
  (declare (salience 1))
  (wm-fact (key domain fact mps-state args? m ?mps s DOWN))
  ?pa <- (plan-action (id ?id) (goal-id ?goal-id)
        (plan-id ?plan-id)
        (action-name prepare-cs|prepare-rs|prepare-ds|prepare-bs)
        (status RUNNING)
        (param-values $? ?mps $?)
        (executable TRUE))
  ?ta <- (timer (name prepare-mps-abort-timer))
  ?ts <- (timer (name prepare-mps-send-timer))
  =>
  (modify ?pa (status FORMULATED) (executable FALSE))
  (retract ?ta ?ts)
)



;React to broken mps
(defrule broken-mps-fail-goal
;TODO: Only Do When Goal Is Production Goal (when first put and then prepare)
  (declare (salience 1))
  ?fg <-(wm-fact (key monitoring fail-goal) (type UNKNOWN) (value ?goal-id))
  ?g <- (goal (id ?goal-id) (mode DISPATCHED))
  (not (plan-action (plan-id ?plan-id) (goal-id ?goal-id) (status ~FORMULATED&~PENDING&~FINAL&~FAILED)))
  =>
  (printout t "Fail goal " ?goal-id " because it is unsatisfiable" crlf)
  (retract ?fg)
  (modify ?g (mode FINISHED) (outcome FAILED))
)

(defrule broken-mps-add-fail-goal-flag
  (declare (salience 1))
  (wm-fact (key domain fact mps-state args? m ?mps s BROKEN))
  ?g <- (goal (id ?goal-id) (mode DISPATCHED))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (plan-action (id ?id) (plan-id ?plan-id) (goal-id ?goal-id)
     (status FORMULATED|PENDING)
     (param-values $? ?mps $?)
     (action-name ?an))
  ;Is this enough for wp-put-slide-cc??
  (domain-atomic-precondition (operator ?an) (predicate mps-state) (param-values ?mps ?state))
  (not (wm-fact (key monitoring fail-goal) (value ?goal-id)))
  =>
  (assert (wm-fact (key monitoring fail-goal) (type UNKNOWN) (value ?goal-id)))
)

; (defrule broken-mps-add-reset-flag
;   (declare (salience 1))
;   (wm-fact (key domain fact mps-state args? m ?mps s BROKEN))
;   (not (wm-fact (key monitoring mps-flush-fact) (type UNKNOWN) (value ?mps)))
;   =>
;   (assert (wm-fact (key monitoring mps-flush-facts) (type UNKNOWN) (value ?mps)))
; )

(defrule broken-mps-remove-facts
  (declare (salience 1))
  (wm-fact (key domain fact mps-state args? m ?mps s BROKEN))
  (wm-fact (key domain fact mps-type args? m ?mps t ?type))
  ; ?flag <- (wm-fact (key monitoring mps-flush-facts) (type UNKNOWN) (value ?mps))
  ; (plan (id ?plan-id) (goal-id ?goal-id))
  ; (goal (id ?goal-id) (mode DISPATCHED))
  ; (not (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
  ;   status ~FORMULATED&~FAILED~FINAL&)))
  ; (param-values $? ?mps $?)))
   =>
  (printout t "MPS " ?mps " was broken, cleaning up facts" crlf)
  (do-for-all-facts ((?wf wm-fact)) (and (neq (member$ ?mps (wm-key-args ?wf:key)) FALSE)
           (or
            (wm-key-prefix ?wf:key (create$ domain fact bs-prepared-color))
            (wm-key-prefix ?wf:key (create$ domain fact ds-prepared-gate))
            (wm-key-prefix ?wf:key (create$ domain fact bs-prepared-side))
            (wm-key-prefix ?wf:key (create$ domain fact cs-prepared-for))
            (wm-key-prefix ?wf:key (create$ domain fact cs-buffered))
            (wm-key-prefix ?wf:key (create$ domain fact cs-can-perform))
            (wm-key-prefix ?wf:key (create$ domain fact rs-filled-with))
            (wm-key-prefix ?wf:key (create$ domain fact rs-prepared-color))
	    (wm-key-prefix ?wf:key (create$ evaluated fact wp-for-order))
           )
              )
    (retract ?wf)
    (printout t "Exec-Monotoring: Broken Machine " ?wf:key crlf " domain facts flushed!"  crlf)
  )
  (switch ?type
    (case CS then
      (assert (wm-fact (key domain fact cs-can-perform args? m ?mps op RETRIEVE_CAP) (value TRUE)))
    )
    (case RS then
      (assert (wm-fact (key domain fact rs-filled-with args? m ?mps n ZERO) (value TRUE)))
    )
  )
  (do-for-all-facts ((?wf wm-fact)) (and (neq (member$ ?mps (wm-key-args ?wf:key)) FALSE)
	   (wm-key-prefix ?wf:key (create$ domain fact wp-at))
           )
           (assert (wm-fact (key monitoring cleanup-wp args? wp (wm-key-arg ?wf:key wp))))
  )
  ; (retract ?flag)
)

(defrule broken-mps-reject-goals
  (declare (salience 1))
  ; (wm-fact (key monitoring mps-flush-facts) (type UNKNOWN) (value ?mps))
  (wm-fact (key domain fact mps-state args? m ?mps s BROKEN))
  ?g <- (goal (id ?goal-id) (mode FORMULATED|SELECTED|EXPANDED) (params $? ?mps $?))
  (plan (id ?plan-id) (goal-id ?goal-id))
  =>
  (modify ?g (mode FINISHED) (outcome REJECTED))
)


(defrule create-action-timeout
  (declare (salience 1))
  (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
      (id ?id)
      (status ?status&~FORMULATED&~RUNNING&~FAILED&~FINAL)
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
  (wm-fact (key game state) (value RUNNING))
  ?pt <- (pending-timer (plan-id ?plan-id) (status ?status) (action-id ?id) (timeout-time $?timeout))
  (time $?now)
  (test (and (> (nth$ 1 ?now) (nth$ 1 ?timeout)) (> (nth$ 2 ?now) (nth$ 2 ?timeout))))
  =>
  (printout t "Action "  ?action-name " timedout after " ?status  crlf)
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

(defrule enhance-timer-on-mps-nonfinal-states
  (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
	(id ?id) (status PENDING)
	(action-name ?action-name)
	(param-values $? ?mps $?))
  (domain-atomic-precondition (operator ?an) (predicate mps-state) (param-values ?mps ?state))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED))
  (wm-fact (key domain fact mps-state args? m ?mps s ?s&~IDLE&~READY-AT-OUTPUT))
  ?pt <- (pending-timer (plan-id ?plan-id) (action-id ?id) (start-time $?starttime) (timeout-time $?timeout))
  (test (< (nth$ 1 ?timeout) (+ (nth$ 1 ?starttime) ?*MPS-DOWN-TIMEOUT-DURATION* ?*COMMON-TIMEOUT-DURATION*)))
  =>
  (printout t "Detected that " ?mps " is " ?s " while " ?action-name " is waiting for it. Enhance timeout-timer" crlf)
  (bind ?timeout-longer (create$ (+ (nth$ 1 ?timeout) ?*MPS-DOWN-TIMEOUT-DURATION*) (nth$ 2 ?timeout)))
  (modify ?pt (timeout-time ?timeout-longer))
)

(defrule cleanup-after-wp-put
  (declare (salience 1))
  (plan-action (id ?id) (goal-id ?goal-id)
	(plan-id ?plan-id)
	(action-name ?an&:(or (eq ?an wp-put) (eq ?an wp-put-slide-cc)))
	(param-values ?r ?wp ?mps $?)
	(status FAILED))
  (plan (id ?plan-id) (goal-id ?goal-id))
  ?g <- (goal (id ?goal-id) (mode FINISHED) (outcome FAILED))
  ?hold <- (wm-fact (key domain fact holding args? r ?r wp ?wp))
  (AX12GripperInterface (holds_puck ?holds))
  =>
  (if (eq ?holds FALSE)
      then
      (retract ?hold)
      (assert (wm-fact (key monitoring cleanup-wp args? wp ?wp)))
      (assert (domain-fact (name can-hold) (param-values ?r)))
  )
  (printout t "Goal " ?goal-id " failed because of " ?an " and is evaluated" crlf)
  (modify ?g (mode EVALUATED))
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
  ?wp-s<- (wm-fact (key domain fact wp-on-shelf args? wp ?wp m ?mps spot ?spot))
  =>
  (printout t "Goal " ?goal-id " has been failed because of wp-get-shelf and is evaluated" crlf)
  (assert (wm-fact (key monitoring cleanup-wp args? wp ?wp)))
  (modify ?g (mode EVALUATED))
)

;(defrule monitoring-holding
;  (time $?now)
;  ?hold <- (wm-fact (key domain fact holding args? r ?r wp ?wp))
;  (AX12GripperInterface (holds_puck FALSE) (time ?s&:(> ?s (+ (nth$ 1 ?now) ?*HOLDING-MONITORING*)) ?ms&:(> ?ms (nth$ 2 ?now))))
;  (goal (id ?goal-id) (mode DISPATCHED))
;  (plan (id ?plan-id) (goal-id ?goal-id))
;  (plan-action (id ?id) (plan-id ?plan-id) (goal-id ?goal-id)
;       (status FORMULATED)
;       (action-name ?an)
;       (param-values $? ?wp $?))
;  (not (wm-fact (key monitoring fail-goal) (value ?goal-id)))
;  =>
;  (printout t "Exec-Monitoring: ensory information contradicts domain" crlf)
;  (assert (wm-fact (key monitoring fail-goal) (value ?goal-id)))
;  (retract ?hold)
;  (assert (wm-fact (key domain fact can-hold args? r ?r) (value TRUE)))
;)

(defrule execution-monitoring-start-retry-action-wp-get
  (declare (salience 1))
  (goal (id ?goal-id) (mode DISPATCHED))
  (plan (id ?plan-id) (goal-id ?goal-id))
  ?pa <- (plan-action
              (action-name ?an&wp-get)
              (plan-id ?plan-id)
              (goal-id ?goal-id)
              (status FAILED)
              (param-values $? ?wp $? ?mps $?))
  (domain-obj-is-of-type ?mps mps)
  (domain-obj-is-of-type ?wp workpiece)
  (wm-fact (key domain fact self args? r ?r))
  (not (wm-fact (key monitoring action-retried args? r ?r a ?an m ?mps wp ?wp)))
  =>
  (if (< 1 ?*MAX-RETRIES-PICK*) then
    (modify ?pa (status PENDING))
    (assert
      (wm-fact (key monitoring action-retried args? r ?r a ?an m ?mps wp ?wp) (value 1))
    )
  )
)

(defrule execution-monitoring-finish-retry-action-wp-get
  (declare (salience 1))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED))
  ?pa <- (plan-action
              (action-name ?an&wp-get)
              (plan-id ?plan-id)
              (goal-id ?goal-id)
              (status FAILED)
              (param-values $? ?wp $? ?mps $?))
  (domain-obj-is-of-type ?mps mps)
  (domain-obj-is-of-type ?wp workpiece)
  (wm-fact (key domain fact self args? r ?r))
  ?wm <- (wm-fact (key monitoring action-retried args? r ?r a ?an m ?mps wp ?wp)
          (value ?tries&:(< ?tries ?*MAX-RETRIES-PICK*)))
  =>
  (bind ?tries (+ 1 ?tries))
  (modify ?pa (status PENDING))
  (modify ?wm (value ?tries))
)

(defrule execution-monitoring-start-retry-action-wp-put-slide-cc
  (declare (salience 1))
  (goal (id ?goal-id) (mode DISPATCHED))
  (plan (id ?plan-id) (goal-id ?goal-id))
  ?pa <- (plan-action
            (action-name ?an&wp-put-slide-cc)
              (plan-id ?plan-id)
              (goal-id ?goal-id)
              (status FAILED)
              (param-values $? ?wp $? ?mps $?))
  (domain-obj-is-of-type ?mps mps)
  (domain-obj-is-of-type ?wp workpiece)
  (wm-fact (key domain fact self args? r ?r))
  (not (wm-fact (key monitoring action-retried args? r ?r a ?an m ?mps wp ?wp)))
  =>
  (if (< 1 ?*MAX-RETRIES-PUT-SLIDE*) then
    (modify ?pa (status PENDING))
    (assert
      (wm-fact (key monitoring action-retried args? r ?r a ?an m ?mps wp ?wp) (value 1))
    )
  )
)

(defrule execution-monitoring-finish-retry-action-wp-put-slide-cc
  (declare (salience 1))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED))
  ?pa <- (plan-action
              (action-name ?an&wp-put-slide-cc)
              (plan-id ?plan-id)
              (goal-id ?goal-id)
              (status FAILED)
              (param-values $? ?wp $? ?mps $?))
  (domain-obj-is-of-type ?mps mps)
  (domain-obj-is-of-type ?wp workpiece)
  (wm-fact (key domain fact self args? r ?r))
  ?wm <- (wm-fact (key monitoring action-retried args? r ?r a ?an m ?mps wp ?wp)
          (value ?tries&:(< ?tries ?*MAX-RETRIES-PUT-SLIDE*)))
  =>
  (bind ?tries (+ 1 ?tries))
  (modify ?pa (status PENDING))
  (modify ?wm (value ?tries))
)

(defrule execution-monitoring-cleanup-wp-facts
  ?cleanup <- (wm-fact (key monitoring cleanup-wp args? wp ?wp))
  =>
  (do-for-all-facts ((?wf wm-fact)) (and (neq (member$ ?wp (wm-key-args ?wf:key)) FALSE)
           (or
            (wm-key-prefix ?wf:key (create$ domain fact wp-at))
            (wm-key-prefix ?wf:key (create$ domain fact wp-usable))
	    (wm-key-prefix ?wf:key (create$ domain fact wp-on-shelf))
            (wm-key-prefix ?wf:key (create$ monitoring cleanup-wp))
           )
    )
    (retract ?wf)
    (printout t "WP-fact " ?wf:key crlf " domain fact flushed!"  crlf)
  )
  (assert (wm-fact (key wp-unused args? wp ?wp)))
  (retract ?cleanup)
)

(defrule execution-monitoring-bs-switch-sides
  ?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) 
	(id ?id) 
	(action-name location-lock) 
	(status RUNNING) 
	(param-values ?bs ?side))
  (wm-fact (key domain fact mps-type args? m ?bs t BS))
  (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (action-name bs-dispense))
  ?li <- (lock-info (name ?name) (goal-id ?goal-id) (plan-id ?plan-id) (action-id ?id) (status WAITING))
  (test (eq ?name (sym-cat ?bs - ?side)))
  =>
  (retract ?li)
  (modify ?pa (status PENDING))
  (delayed-do-for-all-facts ((?p plan-action)) (and (eq ?p:goal-id ?goal-id) (eq ?p:plan-id ?plan-id) (neq FALSE (member$ ?bs ?p:param-values)) (neq FALSE (member$ ?side ?p:param-values)))
 	(printout t "Execution monitoring: Adapting " ?p:action-name crlf)
	(bind $?modified ?p:param-values)
	(if (eq ?side INPUT) then
		(bind ?modified (replace$ ?modified (+ 1 (member$ ?bs ?p:param-values)) (+ 1 (member$ ?bs ?p:param-values)) OUTPUT))
	else 
		(bind ?modified (replace$ ?modified (+ 1 (member$ ?bs ?p:param-values)) (+ 1 (member$ ?bs ?p:param-values)) INPUT))
	)
	(modify ?p (param-values ?modified))
  )
)

(defrule execution-monitoring-last-failed-goal
  (goal (id ?goal-id) (parent ?parent) (class ?goal-class) (type ACHIEVE) (mode EVALUATED) (outcome FAILED) (params $?params))
  (not (wm-fact (key monitoring fact last-failed args? parent ?parent class ?goal-class id ?goal-id2) (values $?params)))
  =>
  (assert (wm-fact (key monitoring fact last-failed args? parent ?parent class ?goal-class id ?goal-id) (values ?params)))
)

(defrule execution-monitoring-decrease-last-failed-priority
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?g <- (goal (id ?goal-id) (parent ?parent) (class ?goal-class) (type ACHIEVE) (mode FORMULATED) (params $?params) (priority ?p&:(neq ?p (+ 1 ?*PRIORITY-GO-WAIT*)))) 
  (wm-fact (key monitoring fact last-failed args? parent ?parent class ?goal-class id ?goal-id2) (values $?params))
  =>
  (printout warn "Dont try " ?goal-class " with " ?params " twice -> Lower priority" crlf)
  (modify ?g (priority (+ 1 ?*PRIORITY-GO-WAIT*)))
) 

(defrule execution-monitoring-remove-last-failed
  (declare (salience ?*SALIENCE-GOAL-PRE-EVALUATE*))
  (goal (id ?goal-id) (parent ?parent) (class ?goal-class) (type ACHIEVE) (mode FINISHED) (outcome COMPLETED))
  ?wm <- (wm-fact (key monitoring fact last-failed args? parent ?parent class ?goal-class2 id ?goal-id2))
  =>
  (do-for-all-facts ((?wm wm-fact)) (and (wm-key-prefix ?wm:key (create$ monitoring fact last-failed))
					 (neq (member$ ?parent (wm-key-args ?wm:key)) FALSE)
				    )
	(retract ?wm)
  )
  (printout warn "Completed a goal, flush last failed" crlf)
)

