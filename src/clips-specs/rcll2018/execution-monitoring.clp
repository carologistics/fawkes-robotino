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
(defrule execution-monitoring-unexpected-mps-state-ready-at-output-start
  (declare (salience 1))
  (time $?now)
  (wm-fact (key domain fact mps-state args? m ?mps s READY-AT-OUTPUT))
  (not (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side OUTPUT|INPUT)))
  (not (timer (name ?name&:(eq ?name (sym-cat READY-AT-OUTPUT ?mps)))))
  (wm-fact (key config rcll reset-mps-unexpected-state) (value TRUE))
  =>
  ;TODO: Send Maintenance message
  (assert (timer (name (sym-cat READY-AT-OUTPUT ?mps)) (time ?now) (seq 1)))
  (printout warn "Monitoring: Unexpected READY-AT-OUTPUT and no WP at output!..starting timer!" crlf)
)

(defrule execution-monitoring-unexpected-mps-state-ready-at-output-abort
  (declare (salience 1))
  (wm-fact (key domain fact mps-type args? m ?mps t ?type))
  (or (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side ?side))
      (wm-fact (key domain fact mps-state args? m ?mps s ~READY-AT-OUTPUT))
  )
  ?t <- (timer (name ?name&:(eq ?name (sym-cat READY-AT-OUTPUT ?mps))))
   =>
  (printout warn "Monitoring: Unexpected READY-AT-OUTPUT recovered!..aborting timer! " crlf)
  (retract ?t)
)

(defrule execution-monitoring-unexpected-mps-state-ready-at-output-end
  (declare (salience 1))
  (time $?now)
  (wm-fact (key domain fact mps-state args? m ?mps s READY-AT-OUTPUT))
  (not (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side ?any-side)))
  (wm-fact (key config rcll reset-mps-unexpected-state-timeout) (value ?reset-mps-unexpected-state-timeout))
  ?t <- (timer (name ?name&:(eq ?name (sym-cat READY-AT-OUTPUT ?mps)))
   (time $?time&:(timeout ?now ?time ?reset-mps-unexpected-state-timeout)))
  (wm-fact (key config rcll reset-mps-unexpected) (value TRUE))
  (mutex (name SPAWNING-MASTER) (state LOCKED) (locked-by ?locked-by))
  (wm-fact (key domain fact self args? r ?self&:(eq ?self (sym-cat ?locked-by))))
   =>
  ;(bind ?wp-gen  (sym-cat WP- (gensym)))
  ;(assert (domain-object (name (sym-cat WP- (gensym))) (type workpiece))
  ;        (wm-fact (key domain fact wp-at args? wp ?wp-gen m ?mps side OUTPUT))
  ;        (wm-fact (key domain fact wp-usable args? wp ?wp-gen))
  ;        )
  (retract ?t)
  (assert (wm-fact (key monitoring reset-mps args? m ?mps) (type BOOL) (value TRUE)))
)

;reason of execlusion: one robot's absence of knowledg about if that wp has a mutex.
;                      does not impley that no one other robot has that mutex.
;                      (ie, it could easly be the case that R-1 has the WP mutex, yet
;                           R-2 doesn not try to get it, hence does not know about it,
;                           ..R-2 resets the machine)
;(defrule execution-monitoring-reset-mps-on-unexpected-input
;  (declare (salience 1))
;  (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side INPUT))
;  (not (mutex (name ?name&:(eq ?name (sym-cat resource- ?wp))) (state LOCKED)))
;  =>
;  ;TODO: Send Maintenance message
;  (assert (wm-fact (key monitoring reset-mps args? m ?mps) (type BOOL) (value TRUE)))
;  (printout warn "Monitoring: Unexpected input and no WP at output!" crlf)
;)


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

(defrule execution-monitoring-issue-reset-mps
  ?t <- (wm-fact (key monitoring action-retried args? r ?self a wp-get m ?mps wp ?wp)
                (value ?tried&:(> ?tried ?*MAX-RETRIES-PICK*)))
  =>
  (assert (wm-fact (key monitoring reset-mps args? m ?mps) (type BOOL) (value TRUE)))
)

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
  (plan-action
             (action-name ?an)
             (id ?id)
             (plan-id ?plan-id)
             (goal-id ?goal-id)
             (param-names $?param-names)
             (param-values $?param-values))
  (test (eq ?wp (plan-action-arg wp ?param-names ?param-values)) )
  (test (eq ?mps (plan-action-arg m ?param-names ?param-values)) )
  ;(domain-obj-is-of-type ?mps mps)
  ;(domain-obj-is-of-type ?wp workpiece)
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
              (id ?id)
              (plan-id ?plan-id)
              (goal-id ?goal-id)
              (status FAILED)
              (param-values $? ?wp $? ?mps $?))
  (plan-action
             (action-name ?an)
             (id ?id)
             (plan-id ?plan-id)
             (goal-id ?goal-id)
             (param-names $?param-names)
             (param-values $?param-values))
  (test (eq ?wp (plan-action-arg wp ?param-names ?param-values)) )
  (test (eq ?mps (plan-action-arg m ?param-names ?param-values)) )
  ;(domain-obj-is-of-type ?mps mps)
  ;(domain-obj-is-of-type ?wp workpiece)
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
              (id ?id)
              (plan-id ?plan-id)
              (goal-id ?goal-id)
              (status FAILED)
              (param-values $? ?wp $? ?mps $?))
  (plan-action
            (action-name ?an)
            (id ?id)
            (plan-id ?plan-id)
            (goal-id ?goal-id)
            (param-names $?param-names)
            (param-values $?param-values))
  (test (eq ?wp (plan-action-arg wp ?param-names ?param-values)) )
  (test (eq ?mps (plan-action-arg m ?param-names ?param-values)) )
  ;(domain-obj-is-of-type ?mps mps)
  ;(domain-obj-is-of-type ?wp workpiece)
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
  (plan-action
             (action-name ?an)
             (id ?id)
             (plan-id ?plan-id)
             (goal-id ?goal-id)
             (param-names $?param-names)
             (param-values $?param-values))
  (test (eq ?wp (plan-action-arg wp ?param-names ?param-values)) )
  (test (eq ?mps (plan-action-arg m ?param-names ?param-values)) )
  ;(domain-obj-is-of-type ?mps mps)
  ;(domain-obj-is-of-type ?wp workpiece)
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
