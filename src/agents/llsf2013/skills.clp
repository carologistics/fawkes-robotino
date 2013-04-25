
;---------------------------------------------------------------------------
;  skills.clp - Robotino agent -- skill interaction
;
;  Created: Sun Jun 17 12:51:10 2012
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; --- RULES for skill outcome

(deffunction goto-machine (?name)
  (skill-call finish_puck_at goto_name ?name)
  (assert (state GOTO))
  (assert (goto-target ?name))
)

(deffunction get-s0 ()
  (skill-call get_s0)
)

(defrule skill-get-s0-done
  ?sf <- (state GET-S0)
  (skill (name "get_s0") (status ?s&FINAL|FAILED))
  =>
  (if (debug 1) then (printout (if (eq ?s FAILED) then warn else t) "get-s0 done: " ?s crlf))
  (retract ?sf)
  (assert (state (sym-cat GET-S0- ?s)))
)

(defrule skill-goto-done
  ?sf <- (state GOTO)
  (skill (name "finish_puck_at") (status ?s&FINAL|FAILED))
  =>
  (retract ?sf)
  (assert (state (sym-cat GOTO- ?s)))
)
