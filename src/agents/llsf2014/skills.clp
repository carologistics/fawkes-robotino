
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

;;;;;;;;;;;;;;;;;;
; calling skills
;;;;;;;;;;;;;;;;;;

(defrule skill-call-finish_puck_at
  ?es <- (execute-skill finish_puck_at ?name ?mtype ?dont-wait)
  (wait-for-lock (res ?name) (state use))
  ?s <- (state WAIT-FOR-LOCK)
  (machine (name ?name) (mtype ?mtype) (loaded-with $?lw))
  =>
  (retract ?es ?s)
  (assert (goto-target ?name)
	  (state GOTO)
	  (goto-dont-wait ?dont-wait)
  )
  (if (eq ?dont-wait true) then
    ;we have to wait if we bring a puck without starting the production
    ;becaule loading needs some time
    (if (or (eq ?mtype T5)
  	    (and (or (eq ?mtype T3) (eq ?mtype T4)) (eq (length$ ?lw) 2))
	    (and (eq ?mtype T2) (eq (length$ ?lw) 1)))
      then
      (bind ?dont-wait true)   

      else
      (bind ?dont-wait false)
    )
  )
  (skill-call finish_puck_at place ?name mtype ?mtype dont_wait ?dont-wait)
)

(defrule skill-call-finish_puck_at_deliver
  ?es <- (execute-skill deliver ?place)
  (wait-for-lock (res ?place) (state use))
  ?s <- (state WAIT-FOR-LOCK)
  =>
  (retract ?es ?s)
  (assert (goto-target ?place)
	  (state GOTO)
  )
  (skill-call finish_puck_at place ?place mtype DE dont_wait false)
)

(defrule skill-call-get-s0
  ?es <- (execute-skill get_s0 ?place)
  (wait-for-lock (res ?place) (state use))
  ?s <- (state WAIT-FOR-LOCK)
  =>
  (retract ?es ?s)
  (assert (get-s0-target ?place)
          (state GET-S0)
  )
  (skill-call get_s0 place ?place)
)

(defrule skill-call-get-produced
  ?es <- (execute-skill get_produced ?machine)
  (wait-for-lock (res ?machine) (state use))
  ?s <- (state WAIT-FOR-LOCK)
  =>
  (retract ?es ?s)
  (assert (get-produced-target ?machine)
	  (state GET-PRODUCED)
  )
  (skill-call get_produced place ?machine)
)

(defrule skill-call-get-consumed
  ?es <- (execute-skill get_consumed ?machine)
  (wait-for-lock (res ?machine) (state use))
  ?s <- (state WAIT-FOR-LOCK)
  =>
  (retract ?es ?s)
  (assert (get-consumed-target ?machine)
	  (state GET-CONSUMED)
  )
  (skill-call get_consumed place ?machine)
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;
; handle skill final/failed
;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule skill-done
  (skill (name ?name) (status ?s&FINAL|FAILED))
  =>
  (assert (skill-done (name ?name) (status ?s)))
)

(defrule skill-get-s0-done
  ?sf <- (state GET-S0)
  ?df <- (skill-done (name "get_s0") (status ?s))
  ?wfl <- (wait-for-lock (res ?place) (state use))
  ?gst <- (get-s0-target ?place)
  =>
  (if (debug 1) then (printout (if (eq ?s FAILED) then warn else t) "get-s0 done: " ?s crlf))
  (if (eq ?s FINAL)
    then
    (retract ?sf ?df ?gst)
    (assert (state (sym-cat GET-S0- ?s)))
    (modify ?wfl (state finished))
    
    else
    ;recall skill
    (retract ?df)
    (skill-call get_s0 place ?place)
  )
)

(defrule skill-get-produced-done
  ?sf <- (state GET-PRODUCED)
  ?df <- (skill-done (name "get_produced") (status ?s))
  (get-produced-target ?goal)
  ?wfl <- (wait-for-lock (res ?goal) (state use))
  =>
  (if (debug 1) then (printout (if (eq ?s FAILED) then warn else t) "get-produced done: " ?s crlf))
  (printout t "skill-get-consumed-done" crlf)
  (retract ?sf ?df)
  (assert (state (sym-cat GET-PRODUCED- ?s)))
  (modify ?wfl (state finished))
)

(defrule skill-get-consumed-done
  ?sf <- (state GET-CONSUMED)
  ?df <- (skill-done (name "get_consumed") (status ?s))
  (get-consumed-target ?goal)
  ?wfl <- (wait-for-lock (res ?goal) (state use))
  =>
  (if (debug 1) then (printout (if (eq ?s FAILED) then warn else t) "get-consumed done: " ?s crlf))
  (printout t "skill-get-consumed-done" crlf)
  (retract ?sf ?df)
  (assert (state (sym-cat GET-CONSUMED- ?s)))
  (modify ?wfl (state finished))
)

(defrule skill-goto-done
  ?sf <- (state GOTO)
  ?df <- (skill-done (name "finish_puck_at") (status ?s))
  (goto-target ?place)
  ?wfl <- (wait-for-lock (res ?place) (state use))
  =>
  (retract ?sf ?df)
  (assert (state (sym-cat GOTO- ?s)))
  (modify ?wfl (state finished))
)

(defrule ignore-arriving-at-wait-point
  ?sd <- (skill-done (name "ppgoto") (status FINAL|FAILED))
  (state WAIT_AND_LOOK_FOR_ALTERATIVE)
  =>
  (retract ?sd)
)