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