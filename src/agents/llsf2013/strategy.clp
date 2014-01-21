
;---------------------------------------------------------------------------
;  sim.clp - Robotino agent -- Production strategy (figures out which machines to use)
;
;  Created: Thu Jun 06 17:02:03 2013
;  Copyright  2013 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defrule strat-allow-t5
  ?m <- (machine (mtype T5) (name ?name))
  =>
  (assert (machine-alloc (machine ?name) (role P3)))
)

;choose machines with the maximum distance to the T5
(defrule strat-allow-P1P2
  (machine (mtype T5) (name ?name-t5) (x ?x5) (y ?y5))
  =>
  (bind ?t1 BLA)
  (bind ?t2 BLA)
  (bind ?t34 BLA)
  (bind ?r BLA)
  (bind ?t1-dist 0.0)
  (bind ?t2-dist 0.0)
  (bind ?t34-dist 0.0)
  (bind ?r-dist 0.0)
  (do-for-all-facts ((?m machine)) TRUE
    (bind ?dist (distance ?x5 ?y5 ?m:x ?m:y))
    (switch ?m:mtype
      (case T1 then (if (> ?dist ?t1-dist) then (bind ?t1 ?m) (bind ?t1-dist ?dist) (bind ?t1-name ?m:name)))
      (case T2 then (if (> ?dist ?t2-dist) then (bind ?t2 ?m) (bind ?t2-dist ?dist) (bind ?t2-name ?m:name)))
      (case T3 then (if (> ?dist ?t34-dist) then (bind ?t34 ?m) (bind ?t34-dist ?dist) (bind ?t34-name ?m:name)))
      (case T4 then (if (> ?dist ?t34-dist) then (bind ?t34 ?m) (bind ?t34-dist ?dist) (bind ?t34-name ?m:name)))
    )
  )
  (assert (machine-alloc (machine ?t1-name) (role P1P2)))
  (assert (machine-alloc (machine ?t2-name) (role P1P2)))
  (assert (machine-alloc (machine ?t34-name) (role P1P2)))
)

;allow machines for role P1 and P2 (disjunct)
(defrule strat-allow-role-P1-and-role-P2
  (machine (mtype T3) (name ?name-t3) (x ?x3) (y ?y3))
  (machine (mtype T4) (name ?name-t4) (x ?x4) (y ?y4))
  =>
  (assert (machine-alloc (machine ?name-t3) (role P1))
	  (machine-alloc (machine ?name-t4) (role P2))
  )
  ;nearest machines to the T3
  (bind ?p1-t1-dist 100.0)
  (bind ?p1-t2-dist 100.0)
  (do-for-all-facts ((?m machine)) TRUE
    (bind ?dist (distance ?x3 ?y3 ?m:x ?m:y))
    (switch ?m:mtype
      (case T1 then (if (< ?dist ?p1-t1-dist) then (bind ?p1-t1 ?m) (bind ?p1-t1-dist ?dist) (bind ?p1-t1-name ?m:name)))
      (case T2 then (if (< ?dist ?p1-t2-dist) then (bind ?p1-t2 ?m) (bind ?p1-t2-dist ?dist) (bind ?p1-t2-name ?m:name)))
    )
  )
  (assert (machine-alloc (machine ?p1-t1-name) (role P1))
	  (machine-alloc (machine ?p1-t2-name) (role P1))
  )
  ;nearest machines to the T4 that are not allowed for P1
  (bind ?p2-t1-dist 100.0)
  (bind ?p2-t2-dist 100.0)
  (do-for-all-facts ((?m machine)) (and (neq ?m:name ?p1-t1-name) (neq ?m:name ?p1-t2-name))
    (bind ?dist (distance ?x4 ?y4 ?m:x ?m:y))
    (switch ?m:mtype
      (case T1 then (if (< ?dist ?p2-t1-dist) then (bind ?p2-t1 ?m) (bind ?p2-t1-dist ?dist) (bind ?p2-t1-name ?m:name)))
      (case T2 then (if (< ?dist ?p2-t2-dist) then (bind ?p2-t2 ?m) (bind ?p2-t2-dist ?dist) (bind ?p2-t2-name ?m:name)))
    )
  )
  (assert (machine-alloc (machine ?p2-t1-name) (role P2))
	  (machine-alloc (machine ?p2-t2-name) (role P2))
  )
)

(defrule strat-show-allowed-machines
  (machine-alloc (machine ?name) (role ?role))
  (machine (name ?name) (mtype ?t))
  =>
  (printout t "Allowing " ?name " as " ?t " for the role " ?role crlf)
)

(deffunction strat-allow-all (?type ?role)
  (delayed-do-for-all-facts ((?machine machine)) (eq ?machine:mtype ?type)
    (assert (machine-alloc (machine ?machine:name) (role ?role)))
  )
  (if (eq ?type T3)
    then
    (delayed-do-for-all-facts ((?machine machine)) (eq ?machine:mtype T4)
      (assert (machine-alloc (machine ?machine:name) (role ?role)))
    )
    else
    (if (eq ?type T4)
      then
      (delayed-do-for-all-facts ((?machine machine)) (eq ?machine:mtype T3)
        (assert (machine-alloc (machine ?machine:name) (role ?role)))
      )
    )
  )
)

;determine if the roles are dynamic
(defrule strat-config-dynamic-roles
  (confval (path "/clips-agent/llsf2013/dynamic-role-change/enabled") (type BOOL) (value ?enabled))
  =>
  (assert (dynamic-role-change ?enabled))
)

;change role from p1p2 to p3 if the first product is delivered
(defrule strat-p1p2-to-p3-after-delivery
  (dynamic-role-change true)
  (confval (path "/clips-agent/llsf2013/dynamic-role-change/p1p2-to-p3-after-delivery") (type BOOL) (value true))
  (phase PRODUCTION)
  ?r <- (role P1P2)
  (delivered-p1p2)
  =>
  (printout warn "Changing Role from P1P2 to P3 because I have finished the first product." crlf)
  (retract ?r)
  (assert (role P3))
)

;change role from p1p2 to p3 if the estimated time for production is greater than the time left
(defrule strat-p1p2-to-p3-no-time
  (dynamic-role-change true)
  (confval (path "/clips-agent/llsf2013/dynamic-role-change/p1p2-to-p3-time") (type BOOL) (value true))
  ?r <- (role P1P2)
  (confval (path "/clips-agent/llsf2013/dynamic-role-change/production-durations/p1p2") (type UINT) (value ?p-duration))
  (phase PRODUCTION)
  ;before starting a new p1/p2 production the agent has no puck and there is no allowed machine whick is loaded
  (holding NONE)
  (forall (machine-alloc (machine ?name) (role P1P2))
	  (machine (name ?name) (loaded-with $?lw&:(eq (length$ ?lw) 0)))
  )
  ;(not (machine (name ?name) (loaded-with $?lw&:(> (length$ ?lw) 0))))
  ;not enough time:
  (game-duration ?g-duration)
  (game-time $?gt&:(> ?p-duration (- ?g-duration (nth$ 1 ?gt))))
  =>
  (printout warn "Changing Role from P1P2 to P3 because there is not enough time for an other p1 or p2" crlf)
  (retract ?r)
  (assert (role P3))       
)

;change role from p1p2,p1 or p2 to p3 if something went wrong
(defrule strat-p1p2-to-p3-after-delivery
  (dynamic-role-change true)
  (confval (path "/clips-agent/llsf2013/dynamic-role-change/p1p2-p1-p2-to-p3-after-fail") (type BOOL) (value true))
  (phase PRODUCTION)
  ?r <- (role P1P2|P1|P2)
  (unknown-fail)
  =>
  (printout warn "Changing Role from P1P2 to P3 because I do not know why my previous production went wrong." crlf)
  (retract ?r)
  (assert (role P3))
)

;allow recycle machine for p3 if the role can change to p3
(defrule strat-allow-recycle-for-p3-after-role-change
  (dynamic-role-change true)
  (role ?r)
  (machine-alloc (machine ?r-name) (role ?r))
  (machine (name ?r-name) (mtype RECYCLE))
  =>
  (assert (machine-alloc (machine ?r-name) (role P3)))
)
