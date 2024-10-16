
;---------------------------------------------------------------------------
;  skills.clp - Robotino agent -- skill interaction
;
;  Created: Sun Jun 17 12:51:10 2012
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; --- RULES for skill outcome

(deffunction goto-machine (?name ?holding)
  (skill-call finish_puck_at goto_name ?name)
)

(deffunction get-s0 ()
  (skill-call get_s0)
)

(defrule get-s0-final
  ?s  <- (state GET-S0)
  ?gf <- (get-s0-final)
  ?h  <- (holding NONE)
  =>
  (if (debug 1) then (printout t "get-s0 succeeded" crlf))
  (retract ?s ?gf ?h)
  (assert (holding S0))
  (assert (state IDLE))
)


(defrule get-s0-fails
  ?s <- (state GET-S0)
  ?gf <- (get-s0-failed)
  =>
  (if (debug 1) then (printout t "get-s0 FAILED" crlf))
  (retract ?s ?gf)
  (assert (state IDLE))
)

(defrule goto-succeeds
  ?s  <- (state GOTO)
  ?gs <- (goto-final ?light)
  =>
  (retract ?s ?gs)
  (assert (light ?light))
  (assert (state GOTO-FINAL))
)

(defrule goto-fails
  ?s <- (state GOTO)
  ?gf <- (goto-failed)
  =>
  (retract ?s ?gf)
  (assert (state GOTO-FAILED))
)
