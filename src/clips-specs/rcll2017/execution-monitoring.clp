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

(defrule execution-monitoring-incosistent-yet-exepected-mps-state-idle
  (declare (salience 1))
  (domain-pending-sensed-fact
    (goal-id ?goal-id)
    (action-id ?action-id)
    (name mps-state)
    (param-values ?mps IDLE)
    (type POSITIVE))
  (wm-fact (key domain fact mps-state args? m ?mps IDLE))
  ?wpat <- (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side OUTPUT))
  =>
  ;TODO: Send Maintenance message
  (printout warn "Monitoring: MPS state IDLE but WP exists at output, Yet action " ?action-id " in Goal " ?goal-id
    "expected it!!" crlf)
  (retract ?wpat)
  (printout warn "The WP has been retracted!!" crlf)
)


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
            (wm-key-prefix ?wf:key (create$ domain fact wp-at))
            (wm-key-prefix ?wf:key (create$ domain fact bs-prepared-color))
            (wm-key-prefix ?wf:key (create$ domain fact ds-prepared-gate))
            (wm-key-prefix ?wf:key (create$ domain fact bs-prepared-side))
            (wm-key-prefix ?wf:key (create$ domain fact cs-prepared-for))
            (wm-key-prefix ?wf:key (create$ domain fact cs-buffered))
            (wm-key-prefix ?wf:key (create$ domain fact cs-can-perform))
            (wm-key-prefix ?wf:key (create$ domain fact rs-filled-with))
            (wm-key-prefix ?wf:key (create$ domain fact rs-prepared-color))
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
  ; (retract ?flag)
)

(defrule broken-mps-reject-goals
  (declare (salience 1))
  ; (wm-fact (key monitoring mps-flush-facts) (type UNKNOWN) (value ?mps))
  (wm-fact (key domain fact mps-state args? m ?mps s BROKEN))
  ?g <- (goal (id ?goal-id) (mode FORMULATED|SELECTED|EXPANDED) (params $? ?mps $?))
  (plan (id ?plan-id) (goal-id ?goal-id))
  =>
  (modify ?g (mode REJECTED))
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
  (wm-fact (key domain fact mps-state args? m ?mps s ~IDLE&~READY-AT-OUTPUT))
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
  (retract ?wp-s)
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
