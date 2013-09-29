
;---------------------------------------------------------------------------
;  sim.clp - Robotino agent -- Production strategy (figures out which machines to use)
;
;  Created: Thu Jun 06 17:02:03 2013
;  Copyright  2013 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defrule strat-allow-t5
  ?m <- (machine (mtype T5) (name ?name) (allowed FALSE))
  =>
  (modify ?m (allowed TRUE))
)

;choose machines with the maximum distance to the T5
(defrule strat-allow-other-machines
  (machine (mtype T5) (name ?name-t5) (x ?x5) (y ?y5))
  =>
  (bind ?t1 BLA)
  (bind ?t2 BLA)
  (bind ?t34 BLA)
  (bind ?t1-dist 0.0)
  (bind ?t2-dist 0.0)
  (bind ?t34-dist 0.0)
  (do-for-all-facts ((?m machine)) TRUE
    (bind ?dist (distance ?x5 ?y5 ?m:x ?m:y))
    (switch ?m:mtype
      (case T1 then (if (> ?dist ?t1-dist) then (bind ?t1 ?m) (bind ?t1-dist ?dist)))
      (case T2 then (if (> ?dist ?t2-dist) then (bind ?t2 ?m) (bind ?t2-dist ?dist)))
      (case T3 then (if (> ?dist ?t34-dist) then (bind ?t34 ?m) (bind ?t34-dist ?dist)))
      (case T4 then (if (> ?dist ?t34-dist) then (bind ?t34 ?m) (bind ?t34-dist ?dist)))
    )
  )
  (modify ?t1 (allowed TRUE))
  (modify ?t2 (allowed TRUE))
  (modify ?t34 (allowed TRUE))
)

(defrule strat-show-allowed-machines
  ?m <- (machine (mtype ?type) (name ?name) (allowed TRUE))
  =>
  (printout t "Allowing " ?name " as " ?type crlf)
)

(deffunction strat-allow-all (?type)
  (delayed-do-for-all-facts ((?machine machine)) (eq ?machine:mtype ?type)
    (modify ?machine (allowed TRUE))
  )
  (if (eq ?type T3)
    then
    (delayed-do-for-all-facts ((?machine machine)) (eq ?machine:mtype T4)
      (modify ?machine (allowed TRUE))
    )
    else
    (if (eq ?type T4)
      then
      (delayed-do-for-all-facts ((?machine machine)) (eq ?machine:mtype T3)
        (modify ?machine (allowed TRUE))
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
  ?r <- (role P1P2)
  (delivered-p1p2)
  =>
  (printout t "Changing Role from P1P2 to P3 because I have finished the first product" crlf)
  (retract ?r)
  (assert (role P3))
)