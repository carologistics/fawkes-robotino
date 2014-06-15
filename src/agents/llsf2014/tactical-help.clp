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
  (do-for-all-facts ((?m machine)) (and (eq T1 ?m:mtype) (eq ?m:team ?team) (not (any-factp ((?lock locked-resource)) (eq ?lock:resource ?m:name))))
    (bind ?dist (distance ?mid-x ?mid-y ?m:x ?m:y))
    (if (< ?dist ?best-dist) then
      (bind ?best-T1 ?m:name)
      (bind ?best-dist ?dist)
    )
  )
  (return ?best-T1)
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