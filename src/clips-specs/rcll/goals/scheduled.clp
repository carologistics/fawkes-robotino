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


(defrule schedule-goal-expand
  " Expand if a plan exists and/or all sub-goals are expanded."
  ?g <- (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS) (mode SELECTED))
  (or  (plan (goal-id ?g-id))
       (goal (parent ?g-id)))
  (not (goal (parent ?g-id) (mode ~EXPANDED)))
  =>
  (modify ?g (mode EXPANDED) (params (create$ )))
)

(defrule schedule-goal-expand-failed
     (declare (salience ?*SALIENCE-LOW*))
     ?gf <- (goal (id ?id) (type ACHIEVE) (sub-type SCHEDULE-SUBGOALS)
                                    (mode SELECTED))
     (not (goal (type ACHIEVE) (parent ?id) (mode FORMULATED|SELECTED|EXPANDED)))
     (not (plan (goal-id ?id)))
     =>
     (modify ?gf (mode FINISHED) (outcome FAILED)
                         (error NO-SUB-GOALS)
                         (message (str-cat "No sub-goals or plans for SCHEDULE goal '" ?id "'" )))
)

(defrule schedule-goal-commit-to-all-children-on-time
     (declare (salience ?*SALIENCE-HIGH*))
     (time ?now-sec ?)
     ?gf <- (goal (id ?g-id) (parent ?pg) (sub-type SCHEDULE-SUBGOALS)
                  (committed-to $?committed) (type ACHIEVE) (mode EXPANDED)
                  (meta dispatch-time ?d-time&:(< ?d-time ?now-sec)))
     (or (plan (id ?child-id&:(not (member$ ?child-id ?committed))) (goal-id ?g-id))
         (goal (id ?child-id&:(not (member$ ?child-id ?committed))) (parent ?g-id)))
  =>
     (modify ?gf  (committed-to (create$ ?committed ?child-id)))
)

(defrule schedule-goal-commit-next
     ?gf <- (goal (id ?g-id) (parent ?pg) (sub-type SCHEDULE-SUBGOALS)
                  (committed-to $?committed) (type ACHIEVE) (mode EXPANDED)
                  (required-resources $?req)
                  (meta dispatch-time ?d-time&:(< ?d-time (nth$ 1 (now)))))
     ;All children are in committed-to
     (not (plan (id ?child-id&:(not (member$ ?child-id ?committed))) (goal-id ?g-id)))
     (not (goal (id ?child-id&:(not (member$ ?child-id ?committed))) (parent ?g-id)))
     ;All children are completed
     (not (goal (parent ?g-id) (mode ~RETRACTED) (outcome ~COMPLETED)))
     ;Next in line for all resource
     (not (goal (required-resources $? ?resource&:(member$ ?resource ?req) $?)
                (meta dispatch-time ?d2-time&:(< ?d2-time ?d-time))
                (outcome ~COMPLETED)
                ))
     =>
     (bind ?req (replace-unbound ?req))
     (modify ?gf (mode COMMITTED) (required-resources ?req))
)

(defrule schedule-goal-error-delayed-commit-resource-scheduled
     ?gf <- (goal (id ?g-id) (parent ?pg) (sub-type SCHEDULE-SUBGOALS)
                  (committed-to $?committed) (type ACHIEVE) (mode EXPANDED)
                  (required-resources $?req)
                  (meta dispatch-time ?d-time&:(< ?d-time (nth$ 1 (now)))))
     (goal (required-resources $? ?resource&:(member$ ?resource ?req) $?)
           (meta dispatch-time ?d2-time&:(< ?d2-time ?d-time))
           (outcome ~COMPLETED)
           (mode ?g2-mode)
           (id ?g2-id))
     =>
    (printout error "Goal " ?g-id " delayed, resource [" ?resource "] needed by goal " ?g2-id " in ( " ?g2-mode ")"  crlf)
)

(defrule schedule-goal-error-delayed-commit-child-not-completed
     ?gf <- (goal (id ?g-id) (parent ?pg) (sub-type SCHEDULE-SUBGOALS)
                  (committed-to $?committed) (type ACHIEVE) (mode EXPANDED)
                  (required-resources $?req)
                  (meta dispatch-time ?d-time&:(< ?d-time (nth$ 1 (now)))))
     (goal (id ?g2-id) (parent ?g-id) (mode ?g2-mode) (outcome ~COMPLETED))
     =>
    (printout error "Goal " ?g-id " delayed, child goal " ?g2-id " in (" ?g2-mode ")" crlf)
)

(defrule schedule-goal-error-delayed-commit
     (time ?now-sec ?)
     ?gf <- (goal (id ?g-id) (parent ?pg) (sub-type SCHEDULE-SUBGOALS)
                  (committed-to $?committed) (type ACHIEVE) (mode EXPANDED)
                  (required-resources $?req)
                  (meta dispatch-time ?d-time&:(< ?d-time (nth$ 1 (now)))))
     =>
    (printout warn "Goal " ?g-id " delayed by (" (- ?now-sec ?d-time) ")"  crlf)
)
(defrule schedule-goal-dispatch
     (time ?now-sec ?)
     ?gf <- (goal (id ?goal-id) (type ACHIEVE) (sub-type SCHEDULE-SUBGOALS) (mode COMMITTED)
                  (committed-to $? ?sub-goal $?)
                  (required-resources $?req)
                  (acquired-resources $?acq&:(subsetp ?req ?acq))
                  (meta dispatch-time ?d-time))
    =>
    (printout t "Goal " ?goal-id " DISPATCHED (" (- ?now-sec ?d-time)  " sec from sched)"crlf)

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

