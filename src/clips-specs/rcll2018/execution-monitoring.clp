;---------------------------------------------------------------------------
;  exploration.clp - Robotino agent for exploration phase
;
;  Copyright  2018 Mostafa Gooma, Tarik Viehmann, Daniel Habering
;
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------



;A timeout for an action
(deftemplate action-timer
  (slot plan-id (type SYMBOL))
  (slot action-id(type NUMBER))
  (slot timeout-duration)
  (multislot start-time)
  (slot status)
)

(defglobal
  ?*MONITORING-SALIENCE* = 1
  ?*COMMON-TIMEOUT-DURATION* = 30
  ?*MPS-DOWN-TIMEOUT-DURATION* = 50
  ?*HOLDING-MONITORING* = 60
)

;
; =============================== Timeouts ===============================
;

(defrule execution-monitoring-create-action-timeout
" For every state of an action that should not take long (pending, waiting for sensed effects),
  create a timeout to prohibit getting stuck at these volatile states
"
  (declare (salience ?*MONITORING-SALIENCE*))
  (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
      (id ?id)
      (state ?status&~FORMULATED&~RUNNING&~FAILED&~FINAL)
      (action-name ?action-name)
      (param-values $?param-values))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED))
  ; TODO: Only Production goals
  (test (neq ?goal-id BEACONACHIEVE))
  (not (action-timer (plan-id ?plan-id) (action-id ?id) (status ?status)))
  (wm-fact (key refbox game-time) (values $?now))
  =>
  (assert (action-timer (plan-id ?plan-id)
              (action-id ?id)
              (timeout-duration ?*COMMON-TIMEOUT-DURATION*)
              (status ?status)
              (start-time ?now)))
)


(defrule execution-monitoring-detect-timeout
" If an action was longer than its timeout-duration in a volatile state like pending or pending-sensed-effect
  reason that this action got stuck and set it to failed
"
  ?p <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
	         (id ?id) (state ?status)
	         (action-name ?action-name)
	         (param-values $?param-values))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED))
  (wm-fact (key game state) (value RUNNING))
  (wm-fact (key refbox game-time) (values $?now))
  ?pt <- (action-timer (plan-id ?plan-id) (status ?status)
            (action-id ?id)
            (start-time $?st)
            (timeout-duration ?timeout&:(timeout ?now ?st ?timeout)))
  =>
  (printout t "Action "  ?action-name " timedout after " ?status  crlf)
  (modify ?p (state FAILED))
  (retract ?pt)
)


(defrule execution-monitoring-remove-timer
" If an action is in a different state than when creating a timer,
  then we can safely remove the timer, since it got not stuck in the previous state
"
  (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
	   (id ?id) (state ?status)
	   (action-name ?action-name)
	   (param-values $?param-values))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED))
  ?pt <- (action-timer (plan-id ?plan-id) (action-id ?id) (status ?st& : (neq ?st ?status)))
  =>
  (retract ?pt)
)


(defrule execution-monitoring-enhance-timer-on-mps-nonfinal-states
" If an action is pending for a certain mps-state and the mps is currently in a non final state (processing, down, ...),
  enhance the timeout to give the mps enough time to reach final state
"
  (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
	   (id ?id) (state PENDING)
	   (action-name ?action-name)
	   (param-values $? ?mps $?))
  (domain-atomic-precondition (operator ?an) (predicate comp-state) (param-values ?mps ?state))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED))
  (wm-fact (key domain fact comp-state args? comp ?mps state ?s&~IDLE&~READY-AT-OUTPUT))
  ?pt <- (action-timer (plan-id ?plan-id)
            (action-id ?id)
            (start-time $?starttime)
            (timeout-duration ?timeout&:(neq ?timeout ?*MPS-DOWN-TIMEOUT-DURATION*)))
  =>
  (printout t "Detected that " ?mps " is " ?s " while " ?action-name " is waiting for it. Enhance timeout-timer" crlf)
  (modify ?pt (timeout-duration ?*MPS-DOWN-TIMEOUT-DURATION*))
)

; =========================== Broken MPS ===============================

(defrule execution-monitoring-broken-mps-fail-pending-action
" If an action is pending and depends on a broken mps, we can instantly set it to failed,
  since the precondition will never be satisfied.
"
  (declare (salience ?*MONITORING-SALIENCE*))
  (wm-fact (key domain fact comp-state args? comp ?mps state BROKEN))
  (goal (id ?goal-id) (mode DISPATCHED))
  ?pa <- (plan-action (id ?action-id)
                      (action-name ?an)
                      (goal-id ?goal-id)
                      (param-values $? ?mps $?)
                      (state PENDING))
  (domain-atomic-precondition (operator ?an) (grounded-with ?action-id) (is-satisfied FALSE))
  =>
  (modify ?pa (state EXECUTION-FAILED))
)



(defrule execution-monitoring-broken-mps-remove-facts
" If a mps is broken, all stored parts and stati are lost.
  Therefore, remove all corresponding facts and mark every workpiece at the machine for cleanup, since its lost
  Afterwards reset the facts of the mps to the initial state, depending on the type of the mps
"
  (declare (salience ?*MONITORING-SALIENCE*))
  (wm-fact (key domain fact comp-state args? comp ?mps state BROKEN))
  (wm-fact (key domain fact mps-type args? m ?mps t ?type))
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
                                            (and (wm-key-prefix ?wf:key (create$ mps-handling prepare))
                                                 (member$ ?mps (wm-key-path ?wf:key)))
                                            (and (wm-key-prefix ?wf:key (create$ mps-handling process))
                                                 (member$ ?mps (wm-key-path ?wf:key)))
                                         )
                                     )
    (retract ?wf)
    (printout t "Exec-Monotoring: Broken Machine " ?wf:key crlf " domain facts flushed!"  crlf)
  )

  (switch ?type
    (case CS then
      (assert (wm-fact (key domain fact cs-can-perform args? m ?mps op RETRIEVE_CAP) (type BOOL) (value TRUE)))
    )
    (case RS then
      (assert (wm-fact (key domain fact rs-filled-with args? m ?mps n ZERO) (type BOOL) (value TRUE)))
    )
  )
  (do-for-all-facts ((?wf wm-fact)) (and (neq (member$ ?mps (wm-key-args ?wf:key)) FALSE)
                                              (wm-key-prefix ?wf:key (create$ domain fact wp-at)))
           (assert (wm-fact (key monitoring cleanup-wp args? wp (wm-key-arg ?wf:key wp))))
  )
  (if (not (any-factp ((?wf wm-fact)) (and (wm-key-prefix ?wf:key (create$ domain fact mps-side-free))
                                           (eq (wm-key-arg ?wf:key m) ?mps)
                                           (eq (wm-key-arg ?wf:key side) INPUT))))
      then
    (assert (wm-fact (key domain fact mps-side-free args? m ?mps side INPUT)))
  )
  (if (not (any-factp ((?wf wm-fact)) (and (wm-key-prefix ?wf:key (create$ domain fact mps-side-free))
                                           (eq (wm-key-arg ?wf:key m) ?mps)
                                           (eq (wm-key-arg ?wf:key side) OUTPUT))))
      then
    (assert (wm-fact (key domain fact mps-side-free args? m ?mps side OUTPUT)))
  )
)

(defrule execution-monitoring-broken-mps-reject-goals
" Every goal that was formulated, that depends on a currently broken goal,
  was formulated with outdated facts about the mps. Therfore, reject it.
"
  (declare (salience ?*MONITORING-SALIENCE*))
  (wm-fact (key domain fact comp-state args? comp ?mps state BROKEN))
  ?g <- (goal (id ?goal-id) (mode FORMULATED|SELECTED|EXPANDED) (params $? ?mps $?))
  (plan (id ?plan-id) (goal-id ?goal-id))
  =>
  (modify ?g (mode FINISHED) (outcome REJECTED))
)

(defrule execution-monitoring-cleanup-wp-facts
"If a workpiece is lost, it is marked for cleanup.
 In this rule all corresponding fact of a marked workpiece are removed
"
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