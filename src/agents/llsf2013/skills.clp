
;---------------------------------------------------------------------------
;  skills.clp - Robotino agent -- skill interaction
;
;  Created: Sun Jun 17 12:51:10 2012
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; --- RULES for skill outcome

(deftemplate skill-done
  (slot name)
  (slot status (allowed-values FINAL FAILED))
)

;dirty robocup function to use a product even if the wait_prodce has a timeout
(deffunction goto-machine-final-product (?name ?mtype)
  (skill-call finish_puck_at place ?name mtype ?mtype final_product TRUE)
  (assert (state GOTO))
  (assert (goto-target ?name))
)

(deffunction goto-machine (?name ?mtype)
  (skill-call finish_puck_at place ?name mtype ?mtype)
  (assert (state GOTO))
  (assert (goto-target ?name))
)

(deffunction get-s0 ()
  (skill-call get_s0)
)

(deffunction get-consumed (?goal)
  (skill-call get_consumed place ?goal)
  (assert (get-consumed-target ?goal))
)

(defrule skill-done
  (skill (name ?name) (status ?s&FINAL|FAILED))
  =>
  (assert (skill-done (name ?name) (status ?s)))
)

(defrule skill-get-s0-done
  ?sf <- (state GET-S0)
  ?df <- (skill-done (name "get_s0") (status ?s))
  =>
  (if (debug 1) then (printout (if (eq ?s FAILED) then warn else t) "get-s0 done: " ?s crlf))
  (retract ?sf ?df)
  (assert (state (sym-cat GET-S0- ?s)))
)

(defrule skill-get-consumed-done
  ?sf <- (state GET-CONSUMED)
  ?df <- (skill-done (name "get_consumed") (status ?s))
  =>
  (if (debug 1) then (printout (if (eq ?s FAILED) then warn else t) "get-consumed done: " ?s crlf))
  (printout t "skill-get-consumed-done" crlf)
  (retract ?sf ?df)
  (assert (state (sym-cat GET-CONSUMED- ?s)))
)

(defrule skill-goto-done
  ?sf <- (state GOTO)
  ?df <- (skill-done (name "finish_puck_at") (status ?s))
  =>
  (retract ?sf ?df)
  (assert (state (sym-cat GOTO- ?s)))
)
