;---------------------------------------------------------------------------
;  steps.clp - rules how to execute the concrete steps of tasks
;
;  Created: Sat Feb 07 19:43:19 2015
;  Copyright  2015 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defrule step-get-S0-start
  (declare (salience ?*PRIORITY-STEP-START*))
  (phase PRODUCTION)
  ?step <- (step (name get-s0) (state wait-for-activation) (task-priority ?p))
  ?state <- (state STEP-STARTED)
  (team-color ?team)
  (input-storage ?team ?ins ? ? )
  (secondary-storage ?team ?inssec ? ?)
  (game-time $?game-time)
  =>
  (retract ?state)
  (assert (state WAIT-FOR-LOCK))
  (modify ?step (state running))
  ; (if (tac-check-for-secondary-ins ?ins ?inssec ?game-time)
  ;   then
  ;   (assert (skill-to-execute (skill get_s0) (args place ?inssec) (target ?inssec))
  ; 	    (wait-for-lock (priority ?p) (res ?inssec))
  ;   )
  ;   else
  ;   (assert (skill-to-execute (skill get_s0) (args place ?ins) (target ?ins))
  ; 	    (wait-for-lock (priority ?p) (res ?ins))
  ;   )
  ; )
)

(defrule step-produce-at-start
  (declare (salience ?*PRIORITY-STEP-START*))
  (phase PRODUCTION)
  ?step <- (step (name produce-at) (state wait-for-activation) (machine ?machine))
  ?state <- (state STEP-STARTED)
  (holding ~NONE)
  (machine (name ?machine) (mtype ?mtype))
  =>
  (retract ?state)
  (assert (state WAIT-FOR-LOCK)
	  (skill-to-execute (skill finish_puck_at) (args place ?machine) (target ?machine))
	  (dont-wait false)
	  (wait-for-lock (res ?machine))
  )
  (modify ?step (state running))
)

(defrule step-deliver-start
  (declare (salience ?*PRIORITY-STEP-START*))
  (phase PRODUCTION)
  ?step <- (step (name deliver) (state wait-for-activation) (task-priority ?p))
  ?state <- (state STEP-STARTED)
  (holding P1|P2|P3)
  (team-color ?team)
  (deliver ?team ?deliver ? ?)
  =>
  (retract ?state)
  (assert (state WAIT-FOR-LOCK)
	  (skill-to-execute (skill deliver) (args place ?deliver) (target ?deliver))
	  (wait-for-lock (priority ?p) (res ?deliver))
  )
  (modify ?step (state running))
)

;;;;;;;;;;;;;;;;
; common finish:
;;;;;;;;;;;;;;;;
(defrule step-common-finish
  (declare (salience ?*PRIORITY-STEP-FINISH*))
  (phase PRODUCTION)
  ?step <- (step (name get-s0|produce-at|deliver) (state running))
  ?state <- (state SKILL-FINAL)
  ?ste <- (skill-to-execute (skill get_s0|finish_puck_at|deliver)
			    (args $?args) (state final))
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
  ?step <- (step (name get-s0|produce-at|deliver) (state running))
  ?state <- (state SKILL-FAILED)
  ?ste <- (skill-to-execute (skill get_s0|finish_puck_at|deliver)
			    (args $?args) (state failed))
  =>
  (retract ?state ?ste)
  (assert (state STEP-FAILED))
  (modify ?step (state failed))
)
