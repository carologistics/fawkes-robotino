
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
  ?skill <- (skill (name ?name) (status ?s&FINAL|FAILED))
  =>
  (retract ?skill)
  (assert (skill-done (name ?name) (status ?s)))
)

;;;;;;;;;;;;;;;;;;
; common skill calls/final-/failed-handling
; special cases done by rules with higher priority
;;;;;;;;;;;;;;;;;;
(defrule skill-common-call
  "call an arbitory skill with a set of args (alternating parameter-name and value)"
  (declare (salience ?*PRIORITY-SKILL-DEFAULT*))
  ?ste <- (skill-to-execute (skill ?skill)
			    (args $?args) (state wait-for-lock) (target ?target))
  (wait-for-lock (res ?res&:(or (eq ?res ?target) (eq ?res (sym-cat ?target "-I")) (eq ?res (sym-cat ?target "-O")))) (state use))
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
(defrule skill-get-product-from-shelf
  "If we get a base from a shelf we need to determine the correct slot as late as possible"
  (declare (salience ?*PRIORITY-SKILL-SPECIAL-CASE*))
  ?ste <- (skill-to-execute (skill get_product_from)
			    (args place ?mps shelf TRUE) (state wait-for-lock) (target ?target))
  ?cs <- (cap-station (name ?mps) (caps-on-shelf ?caps))
  ;(wait-for-lock (res ?target) (state use))
  (wait-for-lock (res ?res&:(or (eq ?res (sym-cat ?target "-I")) (eq ?res (sym-cat ?target "-O")))) (state use))
  ?s <- (state WAIT-FOR-LOCK)
  =>
  (retract ?s)
  (assert (state SKILL-EXECUTION))
  (modify ?ste (state running))
  (switch ?caps
    (case 3
      then
      (bind ?sslot RIGHT)
      (synced-modify ?cs caps-on-shelf 2)
    )
    (case 2
      then
      (bind ?sslot MIDDLE)
      (synced-modify ?cs caps-on-shelf 1)
    )
    (case 1
      then
      (bind ?sslot LEFT)
      (synced-modify ?cs caps-on-shelf 3)
    )
  )
  (skill-call get_product_from place ?mps shelf ?sslot)
)
