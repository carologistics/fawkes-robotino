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
  ; The waiting timeout duration needs to be smaller than the common one above!
  ?*WAITING-TIMEOUT-DURATION* = 25
  ?*MPS-DOWN-TIMEOUT-DURATION* = 120
  ?*HOLDING-MONITORING* = 60
)

(defrule execution-monitoring-stop-dependency-waits
" If a move action is stuck because another robot waits on that position,
  that robot should stop waiting there. "
	(declare (salience ?*MONITORING-SALIENCE*))
	(plan-action (plan-id ?waiting-plan) (goal-id ?waiting-goal)
	    (action-name move)
	    (state PENDING) (executable FALSE)
	    (param-values $?r ?from ?from-side ?to ?to-side))
	(plan (id ?waiting-plan) (goal-id ?waiting-goal))
	(goal (id ?waiting-goal) (mode DISPATCHED))
	?pa <- (plan-action (plan-id ?blocking-plan) (goal-id ?blocking-goal)
	    (action-name wait-for-wp|wait-for-free-side)
	    (state RUNNING))
	(plan (id ?blocking-plan) (goal-id ?blocking-goal))
	(goal (id ?blocking-goal) (mode DISPATCHED))
	(goal-meta (goal-id ?blocking-goal) (assigned-to ?blocking-robot))
	(wm-fact (key domain fact at args? r ?blocking-robot m ?to side ?to-side))
	=>
	(modify ?pa (state FAILED))
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
	(test (neq ?goal-id BEACONACHIEVE))
	(not (action-timer (plan-id ?plan-id) (action-id ?id) (status ?status)))
	(wm-fact (key refbox game-time) (values $?now))
	=>
	(bind ?timeout-duration ?*COMMON-TIMEOUT-DURATION*)
	(if (eq ?status WAITING)
	 then
		(bind ?timeout-duration ?*WAITING-TIMEOUT-DURATION*)
	)
	(assert (action-timer (plan-id ?plan-id)
	            (action-id ?id)
	            (timeout-duration ?timeout-duration)
	            (status ?status)
	            (start-time ?now)))
)

(defrule execution-monitoring-retry-stuck-on-waiting
" If an action remains in state WAITING for too long, the skill call over the
  remote blackboard probably got lost. Hence, retry by setting the action
  back to pending.
  If the timeout is too small, it may cause skill calls to arrive twice,
  which aborts the call arriving first. When network delays are frequent, this
  may cause loops of aborted skills.
"
	?p <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
	         (id ?id) (state WAITING)
	         (action-name ?action-name)
	         (param-values $?param-values))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(goal (id ?goal-id) (mode DISPATCHED))
	(wm-fact (key game state) (value RUNNING))
	(wm-fact (key refbox game-time) (values $?now))
	?pt <- (action-timer (plan-id ?plan-id) (status WAITING)
	                     (action-id ?id)
	                     (start-time $?st)
	                     (timeout-duration ?timeout&:(timeout ?now ?st ?timeout)))
	?exec-info <- (skill-action-execinfo (goal-id ?goal-id) (plan-id ?plan-id) (action-id ?id))
	=>
	(printout t "Action "  ?action-name " stuck on WAITING, retry"  crlf)
	(modify ?p (state PENDING))
	(retract ?exec-info)
	(retract ?pt)
)

(defrule execution-monitoring-detect-timeout
" If an action was longer than its timeout-duration in a volatile state like pending or pending-sensed-effect
  reason that this action got stuck. Print a notification, set state to failed and retract the timer.
"
  ?p <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
	         (id ?id) (state ?status)
	         (action-name ?action-name)
	         (param-values $?param-values)
           (precondition ?grounding-id))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED))
  (wm-fact (key game state) (value RUNNING))
  (wm-fact (key refbox game-time) (values $?now))
  ?pt <- (action-timer (plan-id ?plan-id) (status ?status)
            (action-id ?id)
            (start-time $?st)
            (timeout-duration ?timeout&:(timeout ?now ?st ?timeout)))
  =>
  (printout t "Action "  ?action-name " timed out after " ?status  crlf)
  (modify ?p (state FAILED) (error-msg "Unsatisfied precondition"))
  (retract ?pt)
)

(defrule execution-monitoring-detect-timeout-cause
" If an action FAILED and has an unsatisfied precondition error, match all unsatisifed predicates
  and print the cause. This rule might execute multiple times per timed out formula.
"
  ?p <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
	         (id ?id) (state FAILED)
	         (action-name ?action-name)
	         (param-values $?param-values)
           (precondition ?grounding-id) (error-msg "Unsatisfied precondition"))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (pddl-formula (id ?formula-id) (type atom))
  (grounded-pddl-formula (formula-id ?formula-id) (grounding ?grounding-id) (is-satisfied ?sat))
  (pddl-predicate (part-of ?formula-id) (predicate ?predicate))
  (or
    (test (and ?sat (domain-is-formula-negative ?formula-id)))
    (test (and (not ?sat) (not (domain-is-formula-negative ?formula-id))))
  )
  =>
  (if ?sat
    then
      (printout error  "Action "  ?action-name " precondition (not " ?predicate ") -- which might be nested -- is not satisfied." crlf)
    else
      (printout error "Action "  ?action-name " precondition " ?predicate " -- which might be nested -- is not satisfied." crlf)
  )
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

(defrule execution-monitoring-reset-timer-on-move-when-unapproachable
" Resetting the timeout-timer if a move action is pending while waiting for its target mps-side to be approachable.
"
  (declare (salience ?*MONITORING-SALIENCE*))
  (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
	   (id ?id) (state PENDING)
	   (action-name move)
	   (param-values ?r ?from ?from-side ?to ?to-side $?)
	   )
  (wm-fact (key domain fact at args? r ?r m ?from side ?from-side))
  (not (wm-fact (key domain fact mps-side-approachable args? m ?to side ?to-side)))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED))
  (wm-fact (key refbox game-time) (values $?now))
  ?pt <- (action-timer (plan-id ?plan-id)
            (action-id ?id)
            (start-time $?st)
            (timeout-duration ?timeout&:(timeout ?now ?st ?timeout))
         )
  =>
  (printout t "Detected that " ?goal-id " is waiting with move until "
               ?to " " ?to-side " is approachable. Reset timeout-timer" crlf)
  (modify ?pt (start-time ?now))
)

;
;======================================Retries=========================================
;

(deffunction should-retry (?an ?error)
	(if (or (eq ?error "Conveyor Align Failed") (eq ?error "Drive To Machine Point Failed")) then
	  (return TRUE)
	)
	(if (eq ?error "Unsatisfied precondition") then (return FALSE))
	(if (and (or (eq ?an wp-put) (eq ?an wp-put-slide-cc))
	         (any-factp ((?if RobotinoSensorInterface))
	                    (and (not (nth$ 1 ?if:digital_in)) (nth$ 2 ?if:digital_in)))) then
	  (return TRUE)
	)
	(if (or (eq ?an move) (eq ?an go-wait)) then
	  (return TRUE)
	)
	(return FALSE)
)

(defrule execution-monitoring-start-retry-action
" For some actions it can be feasible to retry them in case of a failure, e.g. if
  the align failed. If an action failed and the error-msg inquires, that we should
  retry, add a action-retried counter and set the action to FORMULATED
"
	(declare (salience ?*MONITORING-SALIENCE*))
	(goal (id ?goal-id) (mode DISPATCHED))
	(plan (id ?plan-id) (goal-id ?goal-id))
	?pa <- (plan-action
	            (id ?id)
	            (action-name ?an)
	            (plan-id ?plan-id)
	            (goal-id ?goal-id)
	            (state FAILED)
	            (error-msg ?error)
	            (param-values $?param-values))
	(test (eq TRUE (should-retry ?an ?error)))
	(wm-fact (key central agent robot args? r ?r))
	(not (wm-fact (key monitoring action-retried args? r ?r a ?an id ?id2&:(eq ?id2 (sym-cat ?id)) m ? g ?goal-id)))
	?sae <- (skill-action-execinfo (goal-id ?goal-id) (plan-id ?plan-id) (skill-name ?an))
	=>
	(bind ?mps nil)
	(do-for-fact ((?do domain-object)) (and (member$ ?do:name ?param-values) (eq ?do:type mps))
	  (bind ?mps ?do:name)
	)
	(assert
	  (wm-fact (key monitoring action-retried args? r ?r a ?an id (sym-cat ?id) m ?mps g ?goal-id) (value 0))
	)
	(modify ?pa (state FORMULATED) (error-msg ""))
	(retract ?sae)
	(printout error "Start retrying" crlf)
)

(defrule execution-monitoring-retry-action
" If the retry of an action is failed again, increment the counter and restart the action
"
	(declare (salience ?*MONITORING-SALIENCE*))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(goal (id ?goal-id) (mode DISPATCHED))
	?pa <- (plan-action
	            (id ?id)
	            (action-name ?an)
	            (plan-id ?plan-id)
	            (goal-id ?goal-id)
	            (state FAILED)
	            (error-msg ?error)
	            (param-values $?param-values))
	(wm-fact (key central agent robot args? r ?r))
	(test (eq TRUE (should-retry ?an ?error)))
	?wm <- (wm-fact (key monitoring action-retried args? r ?r a ?an id ?id2&:(eq ?id2 (sym-cat ?id)) m ? g ?goal-id)
	        (value ?tries&:(< ?tries 3)));?*MAX-RETRIES-PICK*
	?sae <- (skill-action-execinfo (goal-id ?goal-id) (plan-id ?plan-id) (skill-name ?an))
	=>
	(bind ?tries (+ 1 ?tries))
	(modify ?pa (state FORMULATED) (error-msg ""))
	(printout error "Restarted: " ?tries crlf)
	(retract ?sae)
	(modify ?wm (value ?tries))
)

; ----------------------- HANDLE BROKEN MPS -----------------------------------

(defrule execution-monitoring-broken-mps-fail-pending-action
" If an action is pending and depends on a broken mps, we can instantly set it to failed,
  since the precondition will never be satisfied.
"
	(declare (salience ?*MONITORING-SALIENCE*))
	(wm-fact (key domain fact mps-state args? m ?mps s BROKEN))
	(goal (id ?goal-id) (mode DISPATCHED))
	?pa <- (plan-action (id ?action-id)
	                    (action-name ?an)
	                    (goal-id ?goal-id)
	                    (param-values $? ?mps $?)
	                    (state PENDING)
	                    (precondition ?grounding-id)
	)
	(grounded-pddl-predicate (grounding ?grounding-id) (is-satisfied FALSE))
	=>
	(modify ?pa (state EXECUTION-FAILED))
)

(defrule execution-monitoring-broken-mps-add-fail-goal-flag
" If an action makes use of a mps that is broken (and has the mps-state as precondition),
  mark the corresponding goal to be failed.
  Reason: A broken mps looses all stored parts, therefore making the current goal unable to achieve
"
	(declare (salience ?*MONITORING-SALIENCE*))
	(wm-fact (key domain fact mps-state args? m ?mps s BROKEN))
	?g <- (goal (id ?goal-id) (mode DISPATCHED))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(plan-action (id ?id) (plan-id ?plan-id) (goal-id ?goal-id)
	   (state FORMULATED|PENDING)
	   (param-values $? ?mps $?)
	   (action-name ?an&~move&~go-wait&~wait))
	(not (wm-fact (key monitoring fail-goal args? g ?goal-id)))
	=>
	(assert (wm-fact (key monitoring fail-goal args? g ?goal-id)))
)


(defrule execution-monitoring-broken-mps-fail-goal
" If the current dispatched goal is marked to be failed and no action is running,
  set the goal to finished and failed
"
	(declare (salience ?*MONITORING-SALIENCE*))
	?fg <-(wm-fact (key monitoring fail-goal args? g ?goal-id) )
	?g <- (goal (id ?goal-id) (mode DISPATCHED))
	(not (plan-action (plan-id ?plan-id) (goal-id ?goal-id) (state ~FORMULATED&~PENDING&~FINAL&~FAILED)))
	=>
	(printout t "Fail goal " ?goal-id " because it operates operates on a broken mps." crlf)
	(retract ?fg)
	(modify ?g (mode FINISHED) (outcome FAILED))
)

(defrule execution-monitoring-broken-mps-remove-facts
" If a mps is broken, all stored parts and stati are lost.
  Therefore, remove all corresponding facts and mark every workpiece at the machine for cleanup, since its lost
  Afterwards reset the facts of the mps to the initial state, depending on the type of the mps
"
	(declare (salience ?*MONITORING-SALIENCE*))
	(wm-fact (key domain fact mps-state args? m ?mps s BROKEN))
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
)

(defrule execution-monitoring-cleanup-wp-facts
"If a workpiece is lost, it is marked for cleanup.
 In this rule all corresponding fact of a marked workpiece are removed
"
	(declare (salience ?*MONITORING-SALIENCE*))
	?cleanup <- (wm-fact (key monitoring cleanup-wp args? wp ?wp))
	=>
	(do-for-all-facts ((?wf wm-fact)) (and (neq (member$ ?wp (wm-key-args ?wf:key)) FALSE)
	                                       (or
	                                         (wm-key-prefix ?wf:key (create$ domain fact wp-usable))
	                                         (wm-key-prefix ?wf:key (create$ monitoring cleanup-wp))
	                                       )
	                                  )
	  (retract ?wf)
	  (printout t "WP-fact " ?wf:key crlf " domain fact flushed!"  crlf)
	)
	(do-for-all-facts ((?wf wm-fact)) (and (wm-key-prefix ?wf:key (create$ domain fact wp-at))
	                  (eq ?wp (wm-key-arg ?wf:key wp)))
	  (assert (wm-fact (key domain fact mps-side-free args? m (wm-key-arg ?wf:key m) side (wm-key-arg ?wf:key side)) (type BOOL) (value TRUE)))
	  (retract ?wf)
	  (printout t "WP-fact " ?wf:key crlf " domain fact flushed!" crlf)
	)
	(do-for-all-facts ((?wf wm-fact)) (and (wm-key-prefix ?wf:key (create$ domain fact wp-on-shelf))
	                                       (eq ?wp (wm-key-arg ?wf:key wp)))
	  (assert (wm-fact (key domain fact spot-free args? m (wm-key-arg ?wf:key m) spot (wm-key-arg ?wf:key spot)) (type BOOL) (value TRUE)))
	  (retract ?wf)
	  (printout t "WP-fact " ?wf:key crlf " domain fact flushed!" crlf)
	)
	(assert (wm-fact (key wp-unused args? wp ?wp)))
	(retract ?cleanup)
)

; ----------------------- HANDLE SAME SIDE MOVE -------------------------------

(defrule execution-monitoring-handle-same-side-move
" Allows move to current location by setting mps mps-side to be approachable.
"
  (declare (salience ?*MONITORING-SALIENCE*))
  (plan-action
	   (state PENDING)
	   (action-name move)
	   (param-values $? ?mps ?mps-side ?mps ?mps-side $?)
  )
  (not (wm-fact (key domain fact mps-side-approachable args? m ?mps side ?mps-side)))
  =>
  (printout t "Move with equal start and target location "
               ?mps " " ?mps-side " is detected. Set side approachable" crlf)
  (assert (wm-fact (key domain fact mps-side-approachable args? m ?mps side ?mps-side) (type BOOL) (value TRUE)))
)

(defrule execution-monitoring-wp-put-slide-cc
" Adapting params of wp-put-slide-cc if the number of rings of rs-before differs
  from the number of rings the rs is filled with "
  (declare (salience ?*MONITORING-SALIENCE*))
  ?pa <- (plan-action (state PENDING)
               (action-name wp-put-slide-cc)
               (id ?action-id)
               (param-values ?robot ?wp ?rs ?rs-before ?rs-after)
                      (precondition ?grounding-id)
  )
  (grounded-pddl-predicate (grounding ?grounding-id) (is-satisfied FALSE))
  (wm-fact (key domain fact rs-filled-with args? m ?rs n ?rs-num&:(not (eq ?rs-num ?rs-before))))
  (wm-fact (key domain fact rs-inc args? summand ?rs-num sum ?rs-num-after))
  =>
  (printout t "execuction monitoring wp-put-slide-cc: rs-before is unequal to rs-filled-with" crlf)
  (modify ?pa (param-values ?robot ?wp ?rs ?rs-num ?rs-num-after))
)
