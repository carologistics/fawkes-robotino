;---------------------------------------------------------------------------
;  tactical-help.clp - Some helper-functions/rules for tactical decisions
;
;  Created: Fri Mar 21 16:36:48 2014
;  Copyright  2014 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; find the best T1
; @param goal machine name where the S1 should be brought to
; @returns name of the best T1
(deffunction tac-find-best-T1 (?goal ?goal-x ?goal-y ?pos-x ?pos-y ?team)
  ; the best T1 is currently not used by another robot (not locked)
  ; and located between the robot and the goal machine
  
  ;find middle
  (bind ?mid-x (/ (+ ?pos-x ?goal-x) 2))
  (bind ?mid-y (/ (+ ?pos-y ?goal-y) 2))
  ;find best T1
  (bind ?best-T1 NONE)
  (bind ?best-dist 1000.0)
  (do-for-all-facts ((?m machine)) (and (eq T1 ?m:mtype) (eq ?m:team ?team) (not (any-factp ((?lock locked-resource)) (eq ?lock:resource ?m:name))) (eq (nth$ 1 ?m:out-of-order-until) 0))
    (bind ?dist (distance ?mid-x ?mid-y ?m:x ?m:y))
    (if (< ?dist ?best-dist) then
      (bind ?best-T1 ?m:name)
      (bind ?best-dist ?dist)
    )
  )
  (return ?best-T1)
)


(deffunction tac-check-for-secondary-ins (?ins ?inssec ?game-time)
  (if
    (or 
        (and (any-factp ((?inslock locked-resource)) (eq ?inslock:resource ?ins)) (not (any-factp ((?seclock locked-resource)) (eq ?seclock:resource ?inssec))) (>= 60 (nth$ 1 ?game-time)))
        (any-factp ((?secowned locked-resource)) (and (eq ?secowned:resource ?inssec) (eq ?secowned:agent ?*ROBOT-NAME*)))
    )
    then
    (return TRUE)

    else
    (return FALSE)
  )
)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; prioritize machines (which T2,T3/T4 to load first)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrule tac-prioritize-T3-T4
  "Prioritize T3 and T4 machines to determine which should be loaded first."
  (declare (salience ?*PRIORITY-WM*))
  (team-color ?team&~nil)
  (received-machine-info)
  (machine (team ?team) (priority 0) (mtype T3|T4))
  =>
  ;if we are cyan sort machines from right to left, else the other way
  (bind $?prios (create$))
  (if (eq ?team CYAN)
    then
    (bind ?prios (create$ 4 3 2 1))
    else
    (bind ?prios (create$ 1 2 3 4))
  )
  ;find rightmost T3 or T4
  (bind $?prioritized (create$))
  (bind ?first-type NONE)
  (do-for-fact ((?m machine))
	         (and (eq ?m:team ?team)
		      (or (eq ?m:mtype T3) (eq ?m:mtype T4))
		      (not (any-factp ((?m2 machine))
				        (and (eq ?m2:team ?team)
					     (or (eq ?m2:mtype T3) (eq ?m2:mtype T4))
					     (> ?m2:x ?m:x)))))
    (modify ?m (priority (nth$ 1 ?prios)))
    (bind ?prioritized (append$ ?prioritized ?m:name))
    (bind ?first-type ?m:mtype)
  )
  ;find rightmost of other type
  (if (eq ?first-type T3)
      then (bind ?second-type T4)
      else (bind ?second-type T3)
  )
  (do-for-fact ((?m machine))
	         (and (eq ?m:team ?team)
		      (eq ?m:mtype ?second-type)
		      (not (any-factp ((?m2 machine))
				        (and (eq ?m2:team ?team)
					     (eq ?m2:mtype ?second-type)
					     (> ?m2:x ?m:x)))))
    (modify ?m (priority (nth$ 2 ?prios)))
    (bind ?prioritized (append$ ?prioritized ?m:name))
  )
  ;add two left machines from right to left
  (do-for-fact ((?m machine))
	         (and (eq ?m:team ?team)
		      (not (member$ ?m:name ?prioritized))
		      (or (eq ?m:mtype T3) (eq ?m:mtype T4))
		      (not (any-factp ((?m2 machine))
				        (and (eq ?m2:team ?team)
					     (not (member$ ?m2:name ?prioritized))
					     (or (eq ?m2:mtype T3) (eq ?m2:mtype T4))
					     (> ?m2:x ?m:x)))))
    (modify ?m (priority (nth$ 3 ?prios)))
    (bind ?prioritized (append$ ?prioritized ?m:name))
  )
  (do-for-fact ((?m machine))
	         (and (eq ?m:team ?team)
		      (not (member$ ?m:name ?prioritized))
		      (or (eq ?m:mtype T3) (eq ?m:mtype T4)))
    (modify ?m (priority (nth$ 4 ?prios)))
    (bind ?prioritized (append$ ?prioritized ?m:name))
  )
)

(defrule tac-switch-T3-T4-prioritization
  "When the first P1 or P2 was produced the next highest priority is to produce the other product type to satisfy both orders"
  (declare (salience ?*PRIORITY-WM*))
  (team-color ?team&~nil)
  ?m-finished <- (machine (name ?name-finished) (mtype T3|T4) (team ?team) (priority 4) (produced-puck P1|P2))
  ?m-unfinished <- (machine (name ?name-unfinished) (mtype T3|T4) (team ?team) (priority 3) (produced-puck NONE))
  =>
  (printout t "Changing Priorities of " ?name-finished " and " ?name-unfinished crlf)
  (modify ?m-finished (priority 3))
  (modify ?m-unfinished (priority 4))
)

(defrule tac-prioritize-T2
  "Prioritize T2 machines to determine which should be loaded first."
  (declare (salience ?*PRIORITY-WM*))
  (team-color ?team&~nil)
  (received-machine-info)
  (machine (team ?team) (priority 0) (mtype T2))
  (machine (name ?first-T3_T4) (mtype T3|T4) (team ?team) (priority 4) (x ?first-x) (y ?first-y))
  (machine (name ?second-T3_T4) (mtype T3|T4) (team ?team) (priority 3) (x ?second-x) (y ?second-y))
  (input-storage ?team ? ?ins-x ?ins-y)
  =>
  ;highest priority between first T3_T4 and ins
  ;second highest priority between second T3_T4 and ins
  ;lowest priority for every other T2
 
  ;first T2:
  (bind ?mid-x (/ (+ ?first-x ?ins-x) 2))
  (bind ?mid-y (/ (+ ?first-y ?ins-y) 2))
  ;find T2 nearest to this middle
  (bind ?best-T2 NONE)
  (bind ?best-dist 1000.0)
  (do-for-all-facts ((?m machine)) (and (eq T2 ?m:mtype) (eq ?m:team ?team))
    (bind ?dist (distance ?mid-x ?mid-y ?m:x ?m:y))
    (if (< ?dist ?best-dist) then
      (bind ?best-T2 ?m:name)
      (bind ?best-dist ?dist)
    )
  )
  (do-for-fact ((?m machine)) (eq ?best-T2 ?m:name)
    (modify ?m (priority 3))
  )
  
  ;second T2
  (bind ?mid-x (/ (+ ?second-x ?ins-x) 2))
  (bind ?mid-y (/ (+ ?second-y ?ins-y) 2))
  ;find T2 nearest to this middle
  (bind ?best-T2 NONE)
  (bind ?best-dist 1000.0)
  (do-for-all-facts ((?m machine)) (and (eq T2 ?m:mtype) (eq ?m:team ?team) (eq 0 ?m:priority))
    (bind ?dist (distance ?mid-x ?mid-y ?m:x ?m:y))
    (if (< ?dist ?best-dist) then
      (bind ?best-T2 ?m:name)
      (bind ?best-dist ?dist)
    )
  )
  (do-for-fact ((?m machine)) (eq ?best-T2 ?m:name)
    (modify ?m (priority 2))
  )
  
  ;last T2
  (do-for-fact ((?m machine)) (and (eq T2 ?m:mtype) (eq 0 ?m:priority))
    (modify ?m (priority 1))
  )
)

(deffacts tac-wait-positions
  "facts to generate wait positions for each machine (Did not include this in the config because I would need a List of Lists)"
  (common-wait-point WAIT_FOR_DELIVER_1 M12)
  (common-wait-point WAIT_FOR_DELIVER_2 M24)
  (common-wait-point WAIT_FOR_ROW_1 M21 M22 M23)
  (common-wait-point WAIT_FOR_ROW_2 M18 M19 M20)
  (common-wait-point WAIT_FOR_ROW_3 M15 M16 M17)
  (common-wait-point WAIT_FOR_ROW_4 M13 M14 R2)
  (common-wait-point WAIT_FOR_ROW_5 M1 M2 R1)
  (common-wait-point WAIT_FOR_ROW_6 M3 M4 M5)
  (common-wait-point WAIT_FOR_ROW_7 M6 M7 M8)
  (common-wait-point WAIT_FOR_ROW_8 M9 M10 M11)
)

(defrule tac-create-wait-point-facts
  "Convert the previously defined wait-point-lists to get a wait-point fact as for Ins and deliver"
  ?cwp <- (common-wait-point ?wait-point $?reses)
  (team-color CYAN|MAGENTA) ;to ensure the robot name is already set
  =>
  (foreach ?res ?reses
    (assert (wait-point ?res ?*ROBOT-NAME* ?wait-point))
  )
  (retract ?cwp)
)

(defrule tac-no-more-P3-needed
  "Decide when we need no more P3 (there are only three orders for it)"
  (phase PRODUCTION)
  (game-time $?time)
  (order (id ?id1) (product P3)
	 (end ?end1&:(> (nth$ 1 ?time) ?end1)))
  (order (id ?id2&:(neq ?id1 ?id2)) (product P3)
	 (end ?end2&:(> (nth$ 1 ?time) ?end2)))
  (order (id ?id3&:(and (neq ?id3 ?id2) (neq ?id3 ?id1))) (product P3)
	 (end ?end3&:(> (nth$ 1 ?time) ?end3)))  
  =>
  (assert (no-more-needed P3))
)

(deffunction tac-can-use-timeslot (?current-time ?begin ?end ?estimated-time-needed)
  "Can the agent finish a task in a certain time slot with estimated execution time"
  (return (and (>= ?current-time (- ?begin ?estimated-time-needed))
	       (<= ?current-time (- ?end ?estimated-time-needed))))
)
