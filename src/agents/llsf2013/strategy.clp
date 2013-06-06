
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
  (printout "Allowing " ?name " as T5" crlf)
)

;choose machines with the maximum distance to the T5
(defrule strat-allow-other-machines
  (machine (mtype T5) (name ?name-t5) (x ?x5) (y ?y5))
  =>
  (bind ?t1 BLA)
  (bind ?t2 BLA)
  (bind ?t34 BLA)
  (bind ?t1-dist 100.0)
  (bind ?t2-dist 100.0)
  (bind ?t34-dist 100.0)
  (do-for-all-facts ((?m machine)) TRUE
    (bind ?dist (distance ?x5 ?y5 ?m:x ?m:y))
    (switch ?m:mype
      (case T1 then (if (< ?dist ?t1-dist) then (bind ?t1 ?m) (bind ?t1-dist ?dist)))
      (case T2 then (if (< ?dist ?t2-dist) then (bind ?t2 ?m) (bind ?t2-dist ?dist)))
      (case T3 then (if (< ?dist ?t34-dist) then (bind ?t34 ?m) (bind ?t34-dist ?dist)))
      (case T4 then (if (< ?dist ?t34-dist) then (bind ?t34 ?m) (bind ?t34-dist ?dist)))
    )
  )
  (modify ?t1 (allowed TRUE))
  (modify ?t2 (allowed TRUE))
  (modify ?t34 (allowed TRUE))
  (printout "Allowing " ?t1:name " as T1" crlf)
  (printout "Allowing " ?t2:name " as T2" crlf)
  (printout "Allowing " ?t34:name " as T3/T4" crlf)
)
