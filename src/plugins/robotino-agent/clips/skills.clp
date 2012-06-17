
;---------------------------------------------------------------------------
;  skills.clp - Robotino agent -- skill interaction
;
;  Created: Sun Jun 17 12:51:10 2012
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; --- RULES for skill outcome

(defrule get-s0-succeeds
  ?s <- (state GET-S0)
  ?h <- (holding NONE)
  =>
  (if (debug 1) then (printout t "get-s0 succeeded" crlf))
  (retract ?s ?h)
  (assert (holding S0))
  (assert (state IDLE))
)


(defrule get-s0-fails
  ?s <- (state GET-S0)
  ?h <- (holding NONE)
  ?l <- (s0-left ?n&:(<= ?n 0))
  =>
  (if (debug 1) then (printout t "NO MORE S0 AVAILABLE" crlf))
)


(defrule goto-succeeds-s0
  ?s  <- (state GOTO)
  ?gs <- (goto-final ?now-holding)
  ?h  <- (holding S0)
  (goto-target ?node)
  =>
  (retract ?s ?gs ?h)
  (assert (holding ?now-holding))
  (assert (state GOTO-FINAL))
)


(defrule goto-succeeds-s1
  ?s  <- (state GOTO)
  ?gs <- (goto-final ?now-holding)
  ?h  <- (holding S1)
  (goto-target ?node)
  =>
  (retract ?s ?gs ?h)
  (assert (holding ?now-holding))
  (assert (state GOTO-FINAL))
)

(defrule goto-succeeds-s2
  ?s  <- (state GOTO)
  ?gs <- (goto-final)
  ?h  <- (holding ?now-holding)
  (goto-target ?node)
  =>
  (retract ?s ?gs ?h)
  (assert (holding P))
  (assert (state GOTO-FINAL))
)

(defrule goto-succeeds-p
  ?s  <- (state GOTO)
  ?gs <- (goto-final ?now-holding)
  ?h  <- (holding P)
  (goto-target ?node)
  =>
  (if (debug 1) then (printout t "***** DELIVERED a P! *****" crlf))
  (retract ?s ?gs ?h)
  (assert (holding ?now-holding))
  (assert (state GOTO-FINAL))
)
