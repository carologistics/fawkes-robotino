;---------------------------------------------------------------------------
;  schedule.clp - CLIPS executive - goal to run all sub-goals to completion
;
;  Created: Mon Jun 04 15:00:20 2018
;  Copyright  2017-2018  Tim Niemueller [www.niemueller.de]
;             2020       Mostafa Gomaa  [mostafa.go@gmail.com]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; Sub-type: RUN-ALL-OF-SUBGOALS
; Perform: one goal at a time, ordered by goal priority
; Succeed: if all sub-goal succeeds
; Fail:    if exactly one sub-goal fails
; Reject:  if any sub-goals is rejected
;
; A RUN-ALL parent goal will order the goals by priority and then
; start performing them in order. If any goal fails, the parent
; fails. If any sub-goal is rejected, the parent is rejected. If all
; goals have been completed successfully, the parent goal succeeds.
;
; Interactions:
; - User: FORMULATES goal
; - User: SELECTS goal
;   On SELECTED goal
;    * User FORMULATES sub-goals with parent ID equal the RUN-ALL goal ID
;                                                     (populating the children)
;    * Automatic: FAIL goal: if no sub-goal formulated
;    * Automatic: REJECT goal: if all sub-goals rejected
;    * USER: SELECTS sub-goal(s)
; - Automatic: if all selected sub-goal EXPANDED   -> goal EXPANDED
;                                                         (bottom up expantion)
; - Automatic: if root with all sub-goals expanded -> goal COMMITTED
; - Automatic: if parent committed                 -> goal COMMITTED
;         (top down committment to one or more goals with the hightst priority)
; - Automatic: if a sub-goal DISPATCHED            -> goal  DISPATCHED
;                                                       (bottom up DISPATCHING)
; - User: handle sub-goal expansion, committing, dispatching, evaluating
; - Automatic: when sub-goal is EVALUATED, outcome determines parent goal:
;   * REJECTED: mode FINISHED, outcome REJECTED, message
;   * FAILED: mode FINISHED, outcome FAILED, message
;   * COMPLETED: mode FINISHED, outcome COMPLETED
;   -> Sub-goal is RETRACTED.
; User: EVALUATE goal
; User: RETRACT goal


(defrule schedule-goal-selected-timer-start
     (declare (salience ?*SALIENCE-HIGH*))
     ?gf <- (goal (id ?g) (sub-type SCHEDULE-SUBGOALS) (mode SELECTED))
     =>
     (assert (timer (name (sym-cat [ ?g ] - selected) ) (time (now)) (seq 1)))
)


(defrule schedule-goal-expand
  " Expand if a plan exists and/or all sub-goals are expanded."
  ?g <- (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS) (mode SELECTED))
  (or  (plan (goal-id ?g-id))
       (goal (parent ?g-id)))
  (not (goal (parent ?g-id) (mode ~EXPANDED)))
  ?tf <-(timer (name ?n&:(eq ?n (sym-cat [ ?g-id ] - selected))))
  =>
  (modify ?g (mode EXPANDED) (params (create$ )))
  (retract ?tf)
)

(defrule schedule-goal-expand-failed
     (declare (salience ?*SALIENCE-LOW*))
     (time $?now)
     ?gf <- (goal (id ?g) (sub-type SCHEDULE-SUBGOALS) (mode SELECTED))
     (not (goal (type ACHIEVE) (parent ?g) (mode FORMULATED|SELECTED|EXPANDED)))
     (not (plan (goal-id ?g)))
      ?tf <-(timer (name ?n&:(eq ?n (sym-cat [ ?g ] - selected)))
                   (time $?t&:(timeout ?now ?t 3.0))
                   (seq ?seq))
     =>
     (modify ?gf (mode FINISHED) (outcome FAILED)
                         (error NO-SUB-GOALS)
                         (message (str-cat "No sub-goals or plans for SCHEDULE goal '" ?g "'" )))
     (retract ?tf)
)

(defrule schedule-goal-commit-to-all-children-on-time
     (declare (salience ?*SALIENCE-HIGH*))
     (time $?now)
     ?gf <- (goal (id ?goal-id) (sub-type SCHEDULE-SUBGOALS) (mode EXPANDED)
                  (committed-to $?ct&:(eq ?ct (create$)))
                  (meta scheduled-start ?d-time&:(< ?d-time (nth 1 ?now)) $? ))
  =>
     (bind ?committed-to (create$))
     (do-for-all-facts ((?subf goal)) (eq ?subf:parent ?goal-id)
       (bind ?committed-to (append$ ?committed-to ?subf:id ))
     )
     (do-for-all-facts ((?subf plan)) (eq ?subf:goal-id ?goal-id)
       (bind ?committed-to (append$ ?committed-to ?subf:id ))
     )
     (modify ?gf  (committed-to ?committed-to))
     (assert (timer (name (sym-cat [ ?goal-id ] - scheduled-start) ) (time ?now) (seq 1)))
     (printout error (sym-cat [ ?goal-id ] - scheduled-start)  ": " delay " (+" (- (nth$ 1 (now)) ?d-time ) ")"  crlf)
)

(defrule schedule-goal-commit-next
     ?gf <- (goal (id ?g-id) (parent ?pg) (sub-type SCHEDULE-SUBGOALS)
                  (committed-to $?committed) (type ACHIEVE) (mode EXPANDED)
                  (required-resources $?req)
                  (meta scheduled-start ?s-time&:(< ?s-time (nth$ 1 (now))) scheduled-finish ?f-time))
     ;All children are in committed-to
     (not (plan (id ?child-id&:(not (member$ ?child-id ?committed))) (goal-id ?g-id)))
     (not (goal (id ?child-id&:(not (member$ ?child-id ?committed))) (parent ?g-id)))
     ;All children are completed
     (not (goal (parent ?g-id) (mode ~RETRACTED) (outcome ~COMPLETED)))
     ;Next in line for all resource
     ;(not (goal (required-resources $? ?resource&:(member$ ?resource (replace-unbound ?req)) $?)
     (not (goal (required-resources $? ?resource&:(member$ ?resource ?req) $?)
                (meta scheduled-start ?s2-time&:(< ?s2-time ?s-time) $?)
                (outcome ~COMPLETED)
                ))
     =>
     (bind ?req (replace-unbound ?req))
     (modify ?gf (mode COMMITTED) (required-resources ?req))
)

(defrule schedule-goal-error-delayed
     (declare (salience ?*SALIENCE-LOW*))
     (time $?now)
     ?gf <- (goal (id ?g-id) (parent ?pg) (sub-type SCHEDULE-SUBGOALS)
                  (committed-to $?committed) (type ACHIEVE) (mode ?mode)
                  (required-resources $?req) (meta dispatch-time ?d-time))
     ?tf <-(timer (name ?n&:(eq ?n (sym-cat SCHED-DELAY[ ?g-id ])))
            (time $?t&:(timeout ?now ?t 1.0))
            (seq ?seq))
    =>
    ;(bind ?req (replace-unbound ?req))
    (modify ?tf (time ?now) (seq (+ ?seq 1)))
    (printout error  "SCHED-DELAY:--> (" ?mode ") " ?g-id " (+" (- (nth$ 1 ?now) ?d-time ) ")"  crlf)
    ; Unfinished subgoals
    (do-for-all-facts ((?g2f goal)) (and (neq ?g2f:outcome COMPLETED)
                                         (eq ?g2f:parent ?g-id))
        (printout warn  "SCHED-DEYAL     Subgoal ("?g2f:mode ") " ?g2f:id   crlf)
    )
    ; In-use resources
    (progn$ (?rs ?req)
      (do-for-all-facts ((?g2f goal)) (and (neq ?g2f:outcome COMPLETED)
                                           (member$ ?rs ?g2f:required-resources)
                                           (< (nth$ 2 ?g2f:meta) ?d-time))
        (printout warn "SCHED-DELAY:     [" ?rs "] used by (" ?g2f:mode ")  " ?g2f:id  crlf)
      )
    )
)

(defrule schedule-goal-dispatch
     ?gf <- (goal (id ?goal-id) (type ACHIEVE) (sub-type SCHEDULE-SUBGOALS) (mode COMMITTED)
                  (required-resources $?req)
                  (acquired-resources $?acq&:(subsetp ?req ?acq))
                  (meta dispatch-time ?d-time))
     ?tf <-(timer (name ?n&:(eq ?n (sym-cat SCHED-DELAY[ ?goal-id ]))))
    =>
    (printout error ?n ": " DISPATCHED " (+" (- (nth$ 1 (now)) ?d-time ) ")"  crlf)
    (retract ?tf)
    (modify ?gf (mode DISPATCHED))
)


(defrule schedule-goal-evaluated
     ?gf <- (goal (id ?id) (type ACHIEVE) (sub-type SCHEDULE-SUBGOALS) (mode EVALUATED))
     =>
     (modify ?gf (mode RETRACTED) (committed-to (create$ )))
)


(defrule schedule-subgoal-rejected-resources-clear
     ?gf <- (goal (id ?id) (type ACHIEVE) (sub-type SCHEDULE-SUBGOALS)
                  (outcome ~REJECTED))
     (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (acquired-resources)
           (mode RETRACTED) (outcome REJECTED))
     =>
     (modify ?gf (mode FINISHED) (outcome REJECTED)
                 (error SUB-GOAL-REJECTED)
                 (message (str-cat "Sub-goal '" ?sub-goal
                        "' of SCHEDULE goal '" ?id  "' was rejected")))
)


(defrule schedule-subgoal-failed-resources-clear
     ?gf <- (goal (id ?id) (type ACHIEVE) (sub-type SCHEDULE-SUBGOALS)
                  (outcome ~FAILED))
     ?sg <- (goal (id ?sub-goal) (type ACHIEVE) (parent ?id) (acquired-resources)
                  (mode RETRACTED) (outcome FAILED))
     =>
     (modify ?gf (mode FINISHED) (outcome FAILED)
                 (error SUB-GOAL-FAILED ?sub-goal)
                 (message (str-cat "Sub-goal '" ?sub-goal
                       "' of SCHEDULE goal '" ?id "' had failed")))
)

(defrule schedule-goal-failed-resources-clear
     (goal (id ?id) (type ACHIEVE) (sub-type SCHEDULE-SUBGOALS) (acquired-resources)
           (mode RETRACTED) (outcome FAILED))
     ?sg <- (goal (id ?sub-goal) (type ACHIEVE) (parent ?id) (outcome ~FAILED))
     =>
     (modify ?sg (mode FINISHED) (outcome FAILED)
                 (error PARENT-GOAL-REJECTED ?id)
                 (message (str-cat "Parent-goal '" ?id
                           "' of SCHEDULE goal '" ?sub-goal "' had failed")))
)

(defrule schedule-goal-rejected-resources-clear
     (goal (id ?id) (type ACHIEVE) (sub-type SCHEDULE-SUBGOALS) (acquired-resources)
           (mode RETRACTED) (outcome REJECTED))
     ?sg <- (goal (id ?sub-goal) (type ACHIEVE) (parent ?id) (outcome ~REJECTED))
     =>
     (modify ?sg (mode FINISHED) (outcome REJECTED)
                 (error PARENT-GOAL-REJECTED ?id)
                 (message (str-cat "Parent-goal '" ?id
                           "' of SCHEDULE goal '" ?sub-goal "' was rejected")))
)

(defrule schedule-goal-subgoal-completed-all-resources-clear
     ?gf <- (goal (id ?id) (type ACHIEVE) (sub-type SCHEDULE-SUBGOALS) (mode DISPATCHED)
                  (committed-to $?committed))
     (forall (goal (id ?sub-goal&:(member$ ?sub-goal ?committed)) (parent ?id))
             (goal (id ?sub-goal) (acquired-resources) (mode RETRACTED) (outcome COMPLETED)))
     (not (and (plan (id ?plan-id) (goal-id ?goal-id))
               (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state ~FINAL))))
     =>
     (printout t " Scheduled Goal " ?id " is COMPLETED" crlf)
     (modify ?gf (mode FINISHED) (outcome COMPLETED) (committed-to (create$ )))
)

