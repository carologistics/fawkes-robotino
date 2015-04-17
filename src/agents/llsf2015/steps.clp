;---------------------------------------------------------------------------
;  steps.clp - rules how to execute the concrete steps of tasks
;
;  Created: Sat Feb 07 19:43:19 2015
;  Copyright  2015 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defrule step-get-from-shelf
  (declare (salience ?*PRIORITY-STEP-START*))
  (phase PRODUCTION)
  ?step <- (step (name get-from-shelf) (state wait-for-activation) (task-priority ?p)
		 (machine ?mps) (shelf-slot ?sslot))
  (tag-matching (machine ?mps) (tag-id ?tag) (side INPUT) (team ?team))
  ?state <- (state STEP-STARTED)
  (team-color ?team)
  (game-time $?game-time)
  =>
  (retract ?state)
  (modify ?step (state running))
  (printout warn "TODO: use skill to get puck from shelf" crlf)
  (assert (state WAIT-FOR-LOCK)
	  (skill-to-execute (skill ppgoto) (args place (get-input ?mps)) (target ?mps))
	  (wait-for-lock (priority ?p) (res ?mps))
  )
)

(defrule step-insert-start
  (declare (salience ?*PRIORITY-STEP-START*))
  (phase PRODUCTION)
  ?step <- (step (name insert) (state wait-for-activation) (task-priority ?p)
		 (machine ?mps))
  (tag-matching (machine ?mps) (tag-id ?tag) (side INPUT) (team ?team))
  ?state <- (state STEP-STARTED)
  (team-color ?team)
  (game-time $?game-time)
  =>
  (retract ?state)
  (modify ?step (state running))
  (printout warn "TODO: use skill to insert a puck into an MPS" crlf)
  (assert (state WAIT-FOR-LOCK)
	  (skill-to-execute (skill ppgoto) (args place (get-input ?mps)) (target ?mps))
	  (wait-for-lock (priority ?p) (res ?mps))
  )
)

(defrule step-get-output-start
  (declare (salience ?*PRIORITY-STEP-START*))
  (phase PRODUCTION)
  ?step <- (step (name get-output) (state wait-for-activation) (task-priority ?p)
		 (machine ?mps))
  (tag-matching (machine ?mps) (tag-id ?tag) (side INPUT) (team ?team))
  ?state <- (state STEP-STARTED)
  (team-color ?team)
  (game-time $?game-time)
  =>
  (retract ?state)
  (modify ?step (state running))
  (printout warn "TODO: use skill to get a puck from an MPS" crlf)
  (assert (state WAIT-FOR-LOCK)
	  (skill-to-execute (skill ppgoto) (args place (get-output ?mps)) (target ?mps))
	  (wait-for-lock (priority ?p) (res ?mps))
  )
)


; (defrule step-get-S0-start
;   (declare (salience ?*PRIORITY-STEP-START*))
;   (phase PRODUCTION)
;   ?step <- (step (name get-s0) (state wait-for-activation) (task-priority ?p))
;   ?state <- (state STEP-STARTED)
;   (team-color ?team)
;   (input-storage ?team ?ins ? ? )
;   (secondary-storage ?team ?inssec ? ?)
;   (game-time $?game-time)
;   =>
;   (retract ?state)
;   (assert (state WAIT-FOR-LOCK))
;   (modify ?step (state running))
;   ; (if (tac-check-for-secondary-ins ?ins ?inssec ?game-time)
;   ;   then
;   ;   (assert (skill-to-execute (skill get_s0) (args place ?inssec) (target ?inssec))
;   ; 	    (wait-for-lock (priority ?p) (res ?inssec))
;   ;   )
;   ;   else
;   ;   (assert (skill-to-execute (skill get_s0) (args place ?ins) (target ?ins))
;   ; 	    (wait-for-lock (priority ?p) (res ?ins))
;   ;   )
;   ; )
; )

; (defrule step-produce-at-start
;   (declare (salience ?*PRIORITY-STEP-START*))
;   (phase PRODUCTION)
;   ?step <- (step (name produce-at) (state wait-for-activation) (machine ?machine))
;   ?state <- (state STEP-STARTED)
;   (holding ~NONE)
;   (machine (name ?machine) (mtype ?mtype))
;   =>
;   (retract ?state)
;   (assert (state WAIT-FOR-LOCK)
; 	  (skill-to-execute (skill finish_puck_at) (args place ?machine) (target ?machine))
; 	  (dont-wait false)
; 	  (wait-for-lock (res ?machine))
;   )
;   (modify ?step (state running))
; )

; (defrule step-deliver-start
;   (declare (salience ?*PRIORITY-STEP-START*))
;   (phase PRODUCTION)
;   ?step <- (step (name deliver) (state wait-for-activation) (task-priority ?p))
;   ?state <- (state STEP-STARTED)
;   (holding P1|P2|P3)
;   (team-color ?team)
;   (deliver ?team ?deliver ? ?)
;   =>
;   (retract ?state)
;   (assert (state WAIT-FOR-LOCK)
; 	  (skill-to-execute (skill deliver) (args place ?deliver) (target ?deliver))
; 	  (wait-for-lock (priority ?p) (res ?deliver))
;   )
;   (modify ?step (state running))
; )

;;;;;;;;;;;;;;;;
; common finish:
;;;;;;;;;;;;;;;;
(defrule step-common-finish
  (declare (salience ?*PRIORITY-STEP-FINISH*))
  (phase PRODUCTION)
  ?step <- (step (name get-from-shelf|insert|get-output) (state running))
  ?state <- (state SKILL-FINAL)
  ?ste <- (skill-to-execute (skill ppgoto)
			    (args $?args) (state final))
  ; TODO add new skills with |skill
  =>
  (retract ?state ?ste)
  (assert (state STEP-FINISHED))
  (modify ?step (state finished))
)

;;;;;;;;;;;;;;;;
; common fail:
;;;;;;;;;;;;;;;;
(defrule step-common-fail
  (declare (salience ?*PRIORITY-STEP-FAILED*))
  (phase PRODUCTION)
  ?step <- (step (name get-from-shelf|insert|get-output) (state running))
  ?state <- (state SKILL-FAILED)
  ?ste <- (skill-to-execute (skill ppgoto)
			    (args $?args) (state failed))
  ; TODO add new skills with |skill
  =>
  (retract ?state ?ste)
  (assert (state STEP-FAILED))
  (modify ?step (state failed))
)
