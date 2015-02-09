
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

(deffunction skill-call-stop ()
  (skill-call motor_move x 0.0 y 0.0)
)

(defrule skill-done
  (skill (name ?name) (status ?s&FINAL|FAILED))
  =>
  (assert (skill-done (name ?name) (status ?s)))
)

;;;;;;;;;;;;;;;;;;
; common skill calls/final-/failed-handling
; special cases done by rules with higher priority
;;;;;;;;;;;;;;;;;;
(defrule skill-common-call
  "call an arbitory skill with a set of args (alternating parameter-name and value)"
  (declare (salience ?*PRIORITY-SKILL-DEFAULT*))
  ?ste <- (skill-to-execute (skill ?skill&~finish_puck_at&~deliver)
			    (args $?args) (state wait-for-lock) (target ?target))
  (wait-for-lock (res ?target) (state use))
  ?s <- (state WAIT-FOR-LOCK)
  =>
  (retract ?s)
  (assert (state SKILL-EXECUTION))
  (modify ?ste (state running))
  (skill-call ?skill ?args)
)

(defrule skill-common-done
  (declare (salience ?*PRIORITY-SKILL-DEFAULT*))
  ?sf <- (state SKILL-EXECUTION)
  ?df <- (skill-done (name ?skill-str) (status ?s))
  ?wfl <- (wait-for-lock (res ?place) (state use))
  ?ste <- (skill-to-execute (skill ?skill&:(eq ?skill (sym-cat ?skill-str)))
			    (state running))
  =>
  (printout t "skill " ?skill-str " is " ?s crlf)
  (retract ?sf ?df)
  (modify ?wfl (state finished))
  (if (eq ?s FINAL)
    then
    (modify ?ste (state final))
    else
    (modify ?ste (state failed))
  )
  (assert (state (sym-cat SKILL- ?s)))
)
;;;;;;;;;;;;;;;;;;
; special cases for skill calls/final-/failed-handling
;;;;;;;;;;;;;;;;;;
(defrule skill-call-finish_puck_at
  "Call skill finish_puck_at. You need to specify if you want to wait at the machine in (dont-wait ?). The rule calculates if dont-wait has to be set nevertheless and the mtype and out-of-order parameters for the skill."
  (declare (salience ?*PRIORITY-SKILL-SPECIAL-CASE*))
  ?ste <- (skill-to-execute (skill finish_puck_at) (args $?args) (state wait-for-lock) (target ?machine))
  (wait-for-lock (res ?machine) (state use))
  (machine (name ?machine) (mtype ?mtype))
  ?s <- (state WAIT-FOR-LOCK)
  (machine (name ?name) (mtype ?mtype) (loaded-with $?lw))
  (dont-wait ?dont-wait)
  =>
  (retract ?s)
  (assert (state SKILL-EXECUTION))
  (modify ?ste (state running))
  (bind ?out-of-order "abort")
  (if (eq ?dont-wait true) then
    ;we have to wait if we bring a puck without starting the production
    ;becaule loading needs some time
    (if (or (eq ?mtype T5)
  	    (and (or (eq ?mtype T3) (eq ?mtype T4)) (eq (length$ ?lw) 2)))
      then
      (bind ?dont-wait true)   
      (bind ?out-of-order "leave")

      else
      (bind ?dont-wait false)
    )
  )
  (skill-call finish_puck_at (insert$ ?args 1 (create$ mtype ?mtype dont_wait ?dont-wait out_of_order ?out-of-order)))
)

(defrule skill-call-deliver
  "Call deliver skill which also uses the finish_puck_at_skill"
  (declare (salience ?*PRIORITY-SKILL-SPECIAL-CASE*))
  ?ste <- (skill-to-execute (skill deliver) (args $?args) (state wait-for-lock) (target ?deliver))
  (wait-for-lock (res ?deliver) (state use))
  ?s <- (state WAIT-FOR-LOCK)
  =>
  (retract ?s)
  (assert (state SKILL-EXECUTION))
  (modify ?ste (state running))
  (skill-call finish_puck_at (insert$ ?args 1 (create$ mtype DE dont_wait false out_of_order ignore)))
)

(defrule skill-deliver-done
  (declare (salience ?*PRIORITY-SKILL-DEFAULT*))
  ?sf <- (state SKILL-EXECUTION)
  ?df <- (skill-done (name "finish_puck_at") (status ?s))
  ?wfl <- (wait-for-lock (res ?place) (state use))
  ?ste <- (skill-to-execute (skill deliver) (state running))
  =>
  (printout t "skill deliver is " ?s crlf)
  (retract ?sf ?df)
  (modify ?wfl (state finished))
  (if (eq ?s FINAL)
    then
    (modify ?ste (state final))
    else
    (modify ?ste (state failed))
  )
  (assert (state (sym-cat SKILL- ?s)))
)

(defrule skill-get_s0-failed
  "different to the common skill handling we restart get_s0 directly if it fails"
  (declare (salience ?*PRIORITY-SKILL-SPECIAL-CASE*))
  ?sf <- (state SKILL-EXECUTION)
  ?df <- (skill-done (name "get_s0") (status FAILED))
  ?ste <- (skill-to-execute (skill get_s0) (args $?args) (state running))
  =>
  (printout t "get_s0 failed, restarting it" crlf)
  (retract ?df)
  (skill-call get_s0 ?args)
)

(defrule skill-take-puck-to-failed
  "different to the common skill handling we want to restart if we still have a puck"
  (declare (salience ?*PRIORITY-SKILL-SPECIAL-CASE*))
  ?sf <- (state SKILL-EXECUTION)
  ?df <- (skill-done (name "take_puck_to") (status FAILED))
  ?ste <- (skill-to-execute (skill take_puck_to) (args $?args) (state running))
  (puck-in-gripper TRUE)
  =>
  (printout t "take_puck_to failed, restarting it because I still have a puck" crlf)
  (retract ?df)
  (skill-call take_puck_to ?args)
)

(defrule skill-deliver-failed-retry-if-holding-puck
  (declare (salience ?*PRIORITY-SKILL-SPECIAL-CASE*))
  ?sf <- (state SKILL-EXECUTION)
  ?df <- (skill-done (name "finish_puck_at") (status FAILED))
  ?ste <- (skill-to-execute (skill deliver) (args $?args) (state running))
  (puck-in-gripper TRUE)
  =>
  (printout t "take_puck_to failed, restarting it because I still have a puck" crlf)
  (retract ?df)
  (skill-call finish_puck_at (insert$ ?args 1 (create$ mtype DE dont_wait false out_of_order ignore)))
)