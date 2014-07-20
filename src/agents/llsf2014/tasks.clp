;---------------------------------------------------------------------------
;  tasks.clp - rules to execute tasks
;
;  Created: Mon Feb 10 17:27:20 2014
;  Copyright  2014 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------
; All tasks are proposed first in production.clp
; Task execution is decided in cooperation.clp
;
; Overview over all tasks:
; - load-with-S0              get S0, bring it to T2/T3/T4/T5
; - load-with-T1              get S0, produce at T1 and bring to T2/T3/T4
; - pick-and-load             get S2 from T2 and bring it to T3/T4
; - pick-and-deliver          get P1/P2/P3 from T3/T4/T5 and deliver it
; - recycle                   get CO and recycle
; - just-in-time-P3           get S0, wait at T5, start production shortly before order, deliver, again until no more P3's are needed
; - pick-and-store            get produced puck and store it at some place
;---------------------------------------------------------------------------

;;;;;;;;;;;;;;
;load-with-S0:
;;;;;;;;;;;;;;
(defrule task-load-with-S0--start-get-S0
  (declare (salience ?*PRIORITY-SUBTASK-1*))
  (phase PRODUCTION)
  ?t <- (task (name load-with-S0) (args $?a) (state ~finished) (priority ?p))
  (holding NONE)
  ?s <- (state TASK-ORDERED)
  (team-color ?team)
  (input-storage ?team ?ins ? ? )
  (secondary-storage ?team ?inssec ? ?)
  (game-time $?game-time)
  =>
  (retract ?s)
  (if (tac-check-for-secondary-ins ?ins ?inssec ?game-time)

    then
    (assert (execute-skill get_s0 ?inssec)
      (state WAIT-FOR-LOCK)
	    (wait-for-lock (priority ?p) (res ?inssec))
    )
    (modify ?t (state running))

    else
    (assert (execute-skill get_s0 ?ins)
      (state WAIT-FOR-LOCK)
  	  (wait-for-lock (priority ?p) (res ?ins))
    )
    (modify ?t (state running))
  )
)

(defrule task-load-with-S0--bring-to-machine
  (declare (salience ?*PRIORITY-SUBTASK-2*))
  (phase PRODUCTION)
  ?t <- (task (name load-with-S0) (args $?a) (state ~finished) (priority ?p))
  (holding S0)
  (machine (name ?m&:(eq ?m (nth$ 1 ?a))) (mtype ?mtype))
  ?s <- (state GET-S0-FINAL|TASK-ORDERED)
  =>
  (retract ?s)
  (assert (execute-skill finish_puck_at ?m ?mtype true)
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (priority ?p) (res ?m))
  )
  (modify ?t (state running))
)

(defrule task-load-with-S0--finish
  (declare (salience ?*PRIORITY-SUBTASK-3*))
  (phase PRODUCTION)
  ?t <- (task (name load-with-S0) (args $?a) (state ~finished))
  (holding NONE)
  ;is the machine loaded with S0?
  (machine (name ?m&:(eq ?m (nth$ 1 ?a))) (loaded-with $?lw&:(subsetp (create$ S0) ?lw)))
  ?s <- (state GOTO-FINAL)
  =>
  (modify ?t (state finished))
  (retract ?s)
  (assert (state TASK-FINISHED))
)

;;;;;;;;;;;;;;
;load-with-S1:
;;;;;;;;;;;;;;
(defrule task-load-with-S1--start-get-S0
  (declare (salience ?*PRIORITY-SUBTASK-1*))
  (phase PRODUCTION)
  ?t <- (task (name load-with-S1) (args $?a) (state ~finished) (priority ?p))
  (holding NONE)
  ?s <- (state TASK-ORDERED)
  (team-color ?team)
  (input-storage ?team ?ins ? ?)
  (secondary-storage ?team ?inssec ? ?)
  (game-time $?game-time)
  =>
  (retract ?s)
  (if (tac-check-for-secondary-ins ?ins ?inssec ?game-time)

    then
    (assert (execute-skill get_s0 ?inssec)
      (state WAIT-FOR-LOCK)
	    (wait-for-lock (priority ?p) (res ?inssec))
    )
    (modify ?t (state running))

    else
    (assert (execute-skill get_s0 ?ins)
      (state WAIT-FOR-LOCK)
  	  (wait-for-lock (priority ?p) (res ?ins))
    )
    (modify ?t (state running))
  )
)

(defrule task-load-with-S1--produce-S0-at-T1
  (declare (salience ?*PRIORITY-SUBTASK-2*))
  (phase PRODUCTION)
  ?t <- (task (name load-with-S1) (args $?a) (state ~finished) (priority ?p))
  (holding S0)
  (machine (name ?m&:(eq ?m (nth$ 1 ?a))) (x ?goal-x) (y ?goal-y))
  (pose (x ?pos-x) (y ?pos-y))
  (team-color ?team&~nil)
  ?s <- (state GET-S0-FINAL|TASK-ORDERED|GOTO-FINAL-OUT-OF-ORDER)
  =>
  (bind ?m-T1 (tac-find-best-T1 (nth$ 1 ?a) ?goal-x ?goal-y ?pos-x ?pos-y ?team))
  (retract ?s)
  (assert (execute-skill finish_puck_at ?m-T1 T1 false)
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (priority ?p) (res ?m-T1))
  )
  (modify ?t (state running))
)

(defrule task-load-with-S1--bring-S1-to-machine
  (declare (salience ?*PRIORITY-SUBTASK-2*))
  (phase PRODUCTION)
  ?t <- (task (name load-with-S1) (args $?a) (state ~finished) (priority ?p))
  (holding S1)
  (machine (name ?m&:(eq ?m (nth$ 1 ?a))) (mtype ?mtype))
  ?s <- (state GOTO-FINAL|TASK-ORDERED)
  =>
  (retract ?s)
  (assert (execute-skill finish_puck_at ?m ?mtype true)
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (priority ?p) (res ?m))
  )
  (modify ?t (state running))
)

(defrule task-load-with-S1--finish
  (declare (salience ?*PRIORITY-SUBTASK-3*))
  (phase PRODUCTION)
  ?t <- (task (name load-with-S1) (args $?a) (state ~finished))
  (holding NONE)
  ;is the machine loaded with S1?
  (machine (name ?m&:(eq ?m (nth$ 1 ?a))) (loaded-with $?lw&:(subsetp (create$ S1) ?lw)))
  ?s <- (state GOTO-FINAL)
  =>
  (modify ?t (state finished))
  (retract ?s)
  (assert (state TASK-FINISHED))
)

;;;;;;;;;;;;;;;;;;
;pick and deliver:
;;;;;;;;;;;;;;;;;;
(defrule task-pick-and-deliver--start-get-produced-puck
  (declare (salience ?*PRIORITY-SUBTASK-1*))
  (phase PRODUCTION)
  ?t <- (task (name pick-and-deliver) (args $?a) (state ~finished) (priority ?p))
  (holding NONE)
  ?s <- (state TASK-ORDERED)
  =>
  (retract ?s)
  (assert (execute-skill get_produced (nth$ 1 ?a))
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (priority ?p) (res (nth$ 1 ?a)))
  )
  (modify ?t (state running))
)

(defrule task-pick-and-deliver--deliver
  (declare (salience ?*PRIORITY-SUBTASK-2*))
  (phase PRODUCTION)
  ?t <- (task (name pick-and-deliver) (args $?a) (state ~finished) (priority ?p))
  (machine (name ?m&:(eq ?m (nth$ 1 ?a))) (output ?output))
  (holding ?output)
  ?s <- (state GET-PRODUCED-FINAL)
  (team-color ?team)
  (deliver ?team ?deliver ? ?)
  =>
  (retract ?s)
  (assert (execute-skill deliver ?deliver)
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (priority ?p) (res ?deliver))
  )
  (modify ?t (state running))
)

(defrule task-pick-and-deliver--finish
  (declare (salience ?*PRIORITY-SUBTASK-3*))
  (phase PRODUCTION)
  ?t <- (task (name pick-and-deliver) (args $?a) (state ~finished))
  (holding NONE)
  ?s <- (state GOTO-FINAL)
  =>
  (modify ?t (state finished))
  (retract ?s)
  (assert (state TASK-FINISHED))
)

;;;;;;;;;;;;;;;;;;
;pick and load:
;;;;;;;;;;;;;;;;;;
(defrule task-pick-and-load--start-get-produced-puck
  (declare (salience ?*PRIORITY-SUBTASK-1*))
  (phase PRODUCTION)
  ?t <- (task (name pick-and-load) (args $?a) (state ~finished) (priority ?p))
  (holding NONE)
  ?s <- (state TASK-ORDERED)
  =>
  (retract ?s)
  (assert (execute-skill get_produced (nth$ 1 ?a))
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (priority ?p) (res (nth$ 1 ?a)))
  )
  (modify ?t (state running))
)

(defrule task-pick-and-load--load-machine
  (declare (salience ?*PRIORITY-SUBTASK-2*))
  (phase PRODUCTION)
  ?t <- (task (name pick-and-load) (args $?a) (state ~finished) (priority ?p))
  (machine (name ?m1&:(eq ?m1 (nth$ 1 ?a))) (output ?puck))
  (holding ?puck)
  (machine (name ?m2&:(eq ?m2 (nth$ 2 ?a))) (mtype ?mtype))
  ?s <- (state GET-PRODUCED-FINAL)
  =>
  (retract ?s)
  (assert (execute-skill finish_puck_at ?m2 ?mtype true)
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (priority ?p) (res ?m2))
  )
  (modify ?t (state running))
)

(defrule task-pick-and-load--finish
  (declare (salience ?*PRIORITY-SUBTASK-3*))
  (phase PRODUCTION)
  ?t <- (task (name pick-and-load) (args $?a) (state ~finished))
  (holding NONE)
  ?s <- (state GOTO-FINAL)
  =>
  (modify ?t (state finished))
  (retract ?s)
  (assert (state TASK-FINISHED))
)

;;;;;;;;;;;;;;;;;;
;recylce:
;;;;;;;;;;;;;;;;;;
(defrule task-recycle--start-get-consumed-puck
  (declare (salience ?*PRIORITY-SUBTASK-1*))
  (phase PRODUCTION)
  ?t <- (task (name recycle) (args $?a) (state ~finished) (priority ?p))
  (holding NONE)
  ?s <- (state TASK-ORDERED)
  =>
  (retract ?s)
  (assert (execute-skill get_consumed (nth$ 1 ?a))
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (priority ?p) (res (nth$ 1 ?a)))
  )
  (modify ?t (state running))
)

(defrule task-recycle--recycle
  (declare (salience ?*PRIORITY-SUBTASK-2*))
  (phase PRODUCTION)
  ?t <- (task (name recycle) (args $?a) (state ~finished) (priority ?p))
  (machine (name ?m&:(eq ?m (nth$ 2 ?a))) (mtype RECYCLE))
  (holding CO)
  ?s <- (state GET-CONSUMED-FINAL)
  =>
  (retract ?s)
  (assert (execute-skill finish_puck_at ?m RECYCLE false)
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (priority ?p) (res ?m))
  )
  (modify ?t (state running))
)

(defrule task-recycle--finish
  (declare (salience ?*PRIORITY-SUBTASK-3*))
  (phase PRODUCTION)
  ?t <- (task (name recycle) (args $?a) (state ~finished))
  (holding S0)
  ?s <- (state GOTO-FINAL)
  =>
  (modify ?t (state finished))
  (retract ?s)
  (assert (state TASK-FINISHED))
)

(defrule task-recycle--handle-fail
  (phase PRODUCTION)
  ?t <- (task (name recycle) (args $?a) (state ~finished))
  ?s <- (state GET-CONSUMED-FINAL)
  =>
  ;just end the taks
  (modify ?t (state finished))
  (retract ?s)
  (assert (state TASK-FINISHED))
)

;;;;;;;;;;;;;;;;;;;;
;goto-error-handling
;;;;;;;;;;;;;;;;;;;;
(defrule task-common--goto-failed
  (phase PRODUCTION)
  ?t <- (task (name ?name) (args $?a) (state ~finished))
  ?s <- (state GOTO-FAILED)
  =>
  (modify ?t (state failed))
  (retract ?s)
  (assert (state TASK-FAILED))
)

;;;;;;;;;;;;;;;;;;;;
;goto-handle-out-of-order-abort
;;;;;;;;;;;;;;;;;;;;
(defrule task-common--goto-out-of-order-aborted
  "let task fail after we aborted at a out of order machine"
  (phase PRODUCTION)
  ?t <- (task (name ?name) (args $?a) (state ~finished))
  ?s <- (state GOTO-FINAL-OUT-OF-ORDER)
  =>
  (modify ?t (state failed))
  (retract ?s)
  (assert (state TASK-FAILED))
)

;;;;;;;;;;;;;;
;load-with-S2: (this happens only if we are unintentionally holding a S2, normally we use pick-and-load to load a S2)
;;;;;;;;;;;;;;
(defrule task-load-with-S2--start
  (declare (salience ?*PRIORITY-SUBTASK-1*))
  (phase PRODUCTION)
  ?t <- (task (name load-with-S2) (args $?a) (state ~finished) (priority ?p))
  (holding S2)
  (machine (name ?m&:(eq ?m (nth$ 1 ?a))) (mtype ?mtype))
  ?s <- (state TASK-ORDERED)
  =>
  (retract ?s)
  (assert (execute-skill finish_puck_at ?m ?mtype true)
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (priority ?p) (res ?m))
  )
  (modify ?t (state running))
)

(defrule task-load-with-S2--finish
  (declare (salience ?*PRIORITY-SUBTASK-2*))
  (phase PRODUCTION)
  ?t <- (task (name load-with-S2) (args $?a) (state ~finished))
  (holding NONE)
  ;is the machine loaded with S2?
  (machine (name ?m&:(eq ?m (nth$ 1 ?a))) (loaded-with $?lw&:(member$ S2 ?lw)))
  ?s <- (state GOTO-FINAL)
  =>
  (modify ?t (state finished))
  (retract ?s)
  (assert (state TASK-FINISHED))
)

;;;;;;;;;;;;;;
;deliver: (this happens only if we are unintentionally holding a produced puck, normally we use pick-and-deliver for delivery)
;;;;;;;;;;;;;;
(defrule task-deliver--start
  (declare (salience ?*PRIORITY-SUBTASK-1*))
  (phase PRODUCTION)
  ?t <- (task (name deliver) (args $?a) (state ~finished) (priority ?p))
  (holding P1|P2|P3)
  ?s <- (state TASK-ORDERED)
  (team-color ?team)
  (deliver ?team ?deliver ? ?)
  =>
  (retract ?s)
  (assert (execute-skill deliver ?deliver)
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (priority ?p) (res ?deliver))
  )
  (modify ?t (state running))
)

(defrule task-deliver--finish
  (declare (salience ?*PRIORITY-SUBTASK-3*))
  (phase PRODUCTION)
  ?t <- (task (name deliver) (args $?a) (state ~finished))
  (holding NONE)
  ?s <- (state GOTO-FINAL)
  =>
  (modify ?t (state finished))
  (retract ?s)
  (assert (state TASK-FINISHED))
)

;;;;;;;;;;;;;;
;recycle-holding: (this happens only if we are unintentionally holding a CO, normally we use recycle)
;;;;;;;;;;;;;;
(defrule task-recycle-holding--start
  (declare (salience ?*PRIORITY-SUBTASK-1*))
  (phase PRODUCTION)
  ?t <- (task (name recycle-holding) (args $?a) (state ~finished) (priority ?p))
  (holding CO)
  (machine (name ?m&:(eq ?m (nth$ 1 ?a))) (mtype RECYCLE))
  ?s <- (state TASK-ORDERED)
  =>
  (retract ?s)
  (assert (execute-skill finish_puck_at ?m RECYCLE false)
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (priority ?p) (res ?m))
  )
  (modify ?t (state running))
)

(defrule task-recycle-holding--finish
  (declare (salience ?*PRIORITY-SUBTASK-3*))
  (phase PRODUCTION)
  ?t <- (task (name recycle-holding) (args $?a) (state ~finished))
  (holding S0)
  ?s <- (state GOTO-FINAL)
  =>
  (modify ?t (state finished))
  (retract ?s)
  (assert (state TASK-FINISHED))
)


;;;;;;;;;;;;;;
; just-in-time-P3
;;;;;;;;;;;;;;
(defrule task-just-in-time-P3--start-get-S0
  (declare (salience ?*PRIORITY-SUBTASK-1*))
  (phase PRODUCTION)
  ?t <- (task (name just-in-time-P3) (args $?a) (state ~finished) (priority ?p))
  (holding NONE)
  ?s <- (state TASK-ORDERED|TAKE-PUCK-TO-FAILED|GOTO-FINAL)
  (team-color ?team)
  (input-storage ?team ?ins ? ?)
  (secondary-storage ?team ?inssec ? ?)
  (game-time $?game-time)
  =>
  (retract ?s)
  (if (tac-check-for-secondary-ins ?ins ?inssec ?game-time)

    then
    (assert (execute-skill get_s0 ?inssec)
      (state WAIT-FOR-LOCK)
	    (wait-for-lock (priority ?p) (res ?inssec))
    )
    (modify ?t (state running))

    else
    (assert (execute-skill get_s0 ?ins)
      (state WAIT-FOR-LOCK)
  	  (wait-for-lock (priority ?p) (res ?ins))
    )
    (modify ?t (state running))
  )
)

(defrule task-just-in-time-P3--drive-to-machine
  (declare (salience ?*PRIORITY-SUBTASK-2*))
  (phase PRODUCTION)
  ?t <- (task (name just-in-time-P3) (args $?a) (state ~finished) (priority ?p))
  (holding S0)
  (machine (name ?m&:(eq ?m (nth$ 1 ?a))) (mtype ?mtype))
  ?s <- (state GET-S0-FINAL|TASK-ORDERED)
  =>
  (retract ?s)
  (assert (execute-skill drive_to ?m true)
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (priority ?p) (res ?m) (state use));so the locking is ommited
  )
  (modify ?t (state running))
)

(defrule task-just-in-time-P3--wait-for-order
  (declare (salience ?*PRIORITY-SUBTASK-2*))
  (phase PRODUCTION)
  (task (name just-in-time-P3) (args $?a) (state ~finished) (priority ?p))
  (holding S0)
  (machine (name ?m&:(eq ?m (nth$ 1 ?a))) (mtype ?mtype))
  ?s <- (state DRIVE-TO-FINAL)
  =>
  (retract ?s)
  (assert (state WAITING-FOR-P3-ORDER))
)

(defrule task-just-in-time-P3--produce
  (declare (salience ?*PRIORITY-SUBTASK-2*))
  (phase PRODUCTION)
  ?t <- (task (name just-in-time-P3) (args $?a) (state ~finished) (priority ?p))
  (holding S0)
  (machine (name ?m&:(eq ?m (nth$ 1 ?a))) (mtype T5))
  ?s <- (state WAITING-FOR-P3-ORDER)
  (game-time $?time)
  (production-time T5 ?proc-min-time-t5 ?proc-max-time-t5)
  (order (product P3) (quantity-requested ?qr) (quantity-delivered ?qd&:(< ?qd ?qr))
	 (begin ?begin&:(<= (- ?begin ?proc-min-time-t5) (nth$ 1 ?time)))
	 (end ?end&:(>= (- ?end (+ ?proc-max-time-t5 ?*SKILL-DURATION-DELIVER*)) (nth$ 1 ?time))))
  =>
  (retract ?s)
  (assert (execute-skill finish_puck_at ?m T5 false)
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (priority ?p) (res ?m) (state use));so the locking is ommited
  )
  (modify ?t (state running))
)

(defrule task-just-in-time-P3--deliver
  (declare (salience ?*PRIORITY-SUBTASK-2*))
  (phase PRODUCTION)
  ?t <- (task (name just-in-time-P3) (args $?a) (state ~finished) (priority ?p))
  (holding P3)
  ?s <- (state GOTO-FINAL|TASK-ORDERED)
  (team-color ?team)
  (deliver ?team ?deliver ? ?)
  =>
  (retract ?s)
  (assert (execute-skill deliver ?deliver)
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (priority ?p) (res ?deliver))
  )
  (modify ?t (state running))
)

(defrule task-just-in-time-P3--finish
  (declare (salience ?*PRIORITY-SUBTASK-3*))
  (phase PRODUCTION)
  ?t <- (task (name just-in-time-P3) (args $?a) (state ~finished))
  (holding NONE|S0)
  ?s <- (state TASK-ORDERED|GOTO-FINAL|GET-S0-FINAL|TAKE-PUCK-TO-FINAL|WAITING-FOR-P3-ORDER)
  (no-more-needed P3)
  =>
  (modify ?t (state finished))
  (retract ?s)
  (assert (state TASK-FINISHED))
)

;;;;;;;;;;;;;;;;;;
; pick and store:
;;;;;;;;;;;;;;;;;;
(defrule task-pick-and-store--start-get-produced-puck
  (declare (salience ?*PRIORITY-SUBTASK-1*))
  (phase PRODUCTION)
  ?t <- (task (name pick-and-store) (args $?a) (state ~finished) (priority ?p))
  (holding NONE)
  ?s <- (state TASK-ORDERED)
  =>
  (retract ?s)
  (assert (execute-skill get_produced (nth$ 1 ?a))
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (priority ?p) (res (nth$ 1 ?a)))
  )
  (modify ?t (state running))
)

(defrule task-pick-and-store--store
  (declare (salience ?*PRIORITY-SUBTASK-2*))
  (phase PRODUCTION)
  ?t <- (task (name pick-and-store) (args $?a) (state ~finished) (priority ?p))
  (machine (name ?m&:(eq ?m (nth$ 1 ?a))) (output ?output))
  (holding ?output)
  ?s <- (state GET-PRODUCED-FINAL|STORE-PUCK-FAILED)
  (team-color ?team)
  (puck-storage (name ?storage&:(eq ?storage (nth$ 3 ?a))))
  =>
  (retract ?s)
  (assert (execute-skill store_puck ?storage)
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (priority ?p) (res ?storage))
  )
  (modify ?t (state running))
)

(defrule task-pick-and-store--finish
  (declare (salience ?*PRIORITY-SUBTASK-3*))
  (phase PRODUCTION)
  ?t <- (task (name pick-and-store) (args $?a) (state ~finished))
  (holding NONE)
  ?s <- (state STORE-PUCK-FINAL)
  =>
  (modify ?t (state finished))
  (retract ?s)
  (assert (state TASK-FINISHED))
)

;;;;;;;;;;;;;;;;;;
; store:
;;;;;;;;;;;;;;;;;;

(defrule task-store--store
  (declare (salience ?*PRIORITY-SUBTASK-1*))
  (phase PRODUCTION)
  ?t <- (task (name store) (args $?a) (state ~finished) (priority ?p))
  ?s <- (state TASK-ORDERED)
  (team-color ?team)
  (puck-storage (name ?storage&:(eq ?storage (nth$ 1 ?a))))
  =>
  (retract ?s)
  (assert (execute-skill store_puck ?storage)
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (priority ?p) (res ?storage))
  )
  (modify ?t (state running))
)

(defrule task-store--finish
  (declare (salience ?*PRIORITY-SUBTASK-3*))
  (phase PRODUCTION)
  ?t <- (task (name store) (args $?a) (state ~finished))
  (holding NONE)
  ?s <- (state STORE-PUCK-FINAL)
  =>
  (modify ?t (state finished))
  (retract ?s)
  (assert (state TASK-FINISHED))
)

;;;;;;;;;;;;;;;;;;;;;;;;;
; get stored and deliver:
;;;;;;;;;;;;;;;;;;;;;;;;;
(defrule task-get-stored-and-deliver--start-get-stored-puck
  (declare (salience ?*PRIORITY-SUBTASK-1*))
  (phase PRODUCTION)
  ?t <- (task (name get-stored-and-deliver) (args $?a) (state ~finished) (priority ?p))
  (holding NONE)
  ?s <- (state TASK-ORDERED)
  =>
  (retract ?s)
  (assert (execute-skill get_stored_puck (nth$ 1 ?a))
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (priority ?p) (res (nth$ 1 ?a)))
  )
  (modify ?t (state running))
)

(defrule task-get-stored-and-deliver--deliver
  (declare (salience ?*PRIORITY-SUBTASK-2*))
  (phase PRODUCTION)
  ?t <- (task (name get-stored-and-deliver) (args $?a) (state ~finished) (priority ?p))
  (holding ?puck&:(eq ?puck (nth$ 2 ?a)))
  ?s <- (state GET-STORED-PUCK-FINAL)
  (team-color ?team)
  (deliver ?team ?deliver ? ?)
  =>
  (retract ?s)
  (assert (execute-skill deliver ?deliver)
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (priority ?p) (res ?deliver))
  )
  (modify ?t (state running))
)

(defrule task-get-stored-and-deliver--finish
  (declare (salience ?*PRIORITY-SUBTASK-3*))
  (phase PRODUCTION)
  ?t <- (task (name get-stored-and-deliver) (args $?a) (state ~finished))
  (holding NONE)
  ?s <- (state GOTO-FINAL|GET-STORED-PUCK-FAILED)
  =>
  (modify ?t (state finished))
  (retract ?s)
  (assert (state TASK-FINISHED))
)

;;;;;;;;;;;;;;;;;
;produce-with-S0:
;;;;;;;;;;;;;;;;;
(defrule task-produce-with-S0--start-get-S0
  (declare (salience ?*PRIORITY-SUBTASK-1*))
  (phase PRODUCTION)
  ?t <- (task (name produce-with-S0) (args $?a) (state ~finished) (priority ?p))
  (holding NONE)
  ?s <- (state TASK-ORDERED)
  (team-color ?team)
  (input-storage ?team ?ins ? ? )
  (secondary-storage ?team ?inssec ? ?)
  (game-time $?game-time)
  =>
  (retract ?s)
  (if (tac-check-for-secondary-ins ?ins ?inssec ?game-time)

    then
    (assert (execute-skill get_s0 ?inssec)
      (state WAIT-FOR-LOCK)
	    (wait-for-lock (priority ?p) (res ?inssec))
    )
    (modify ?t (state running))

    else
    (assert (execute-skill get_s0 ?ins)
      (state WAIT-FOR-LOCK)
  	  (wait-for-lock (priority ?p) (res ?ins))
    )
    (modify ?t (state running))
  )
)

(defrule task-produce-with-S0--produce-at-machine
  (declare (salience ?*PRIORITY-SUBTASK-2*))
  (phase PRODUCTION)
  ?t <- (task (name produce-with-S0) (args $?a) (state ~finished) (priority ?p))
  (holding S0)
  (machine (name ?m&:(eq ?m (nth$ 1 ?a))) (mtype ?mtype))
  ?s <- (state GET-S0-FINAL|TASK-ORDERED)
  =>
  (retract ?s)
  (assert (execute-skill finish_puck_at ?m ?mtype false)
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (priority ?p) (res ?m))
  )
  (modify ?t (state running))
)

(defrule task-produce-with-S0--finish
  (declare (salience ?*PRIORITY-SUBTASK-3*))
  (phase PRODUCTION)
  ?t <- (task (name produce-with-S0) (args $?a) (state ~finished))
  (holding ?output)
  (machine (name ?m&:(eq ?m (nth$ 1 ?a))) (output ?output))
  ?s <- (state GOTO-FINAL)
  =>
  (modify ?t (state finished))
  (retract ?s)
  (assert (state TASK-FINISHED))
)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Fallback for unhandled situation
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrule task-fallback-end-task
  (declare (salience ?*PRIORITY-CLEANUP*))
  (phase PRODUCTION)
  ?t <- (task (name ?name) (args $?a) (state ~finished))
  ?s <- (state GOTO-FINAL|GOTO-FAILED|GET-CONSUMED-FINAL|GET-CONSUMED-FAILED|GET-PRODUCED-FINAL|GET-PRODUCED-FAILED|GET-S0-FINAL|GET-S0-FAILED|STORE-PUCK-FAILED|GET-STORED-PUCK-FAILED)
  (time $?now)
  =>
  (printout error "Unhandled situation in agent!!!" crlf)
  (printout error "Ending task " ?name crlf)
  (printout error "Saving fact-list to  agent-snapshot-" (nth$ 1 ?now) ".clp" crlf)
  (save-facts (str-cat "agent-snapshot-" (nth$ 1 ?now) ".clp") visible)

  (modify ?t (state failed))
  (retract ?s)
  (assert (state TASK-FAILED))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Canceling tasks in specific situations
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule task-cancle-deliver--store-after-order-over
  "If the order is over, we are delivering a puck and there is a free storage point, store the puck instead (cancle taks first)."
  (declare (salience ?*PRIORITY-SUBTASK-3*))
  (phase PRODUCTION)
  ?t <- (task (name pick-and-deliver|deliver|get-stored-and-deliver) (args $?a) (state ~finished))
  (holding ?puck&P3)
  ?s <- (state GOTO)
  ?gt <- (goto-target deliver1|deliver2)
  (game-time $?time)
  (not (order (product ?puck) (quantity-requested ?qr) (id  ?order-id)
	      (begin ?begin&:(<= (- ?begin (+ ?*SKILL-DURATION-DELIVER* ?*SKILL-DURATION-GET-PRODUCED*)) (nth$ 1 ?time)))
	      (end ?end&:(<= (nth$ 1 ?time) ?end))))
  ?wfl <- (wait-for-lock (res ?goal) (state use))
  (team-color ?team-color&~nil)
  (puck-storage (puck NONE) (team ?team-color) (incoming $?i-st&~:(member$ STORE_PUCK ?i-st)))
  (skill (name "finish_puck_at") (status RUNNING))
  ;we are not moving under the rfid right now
  (not (pose (x ?x&:(or (> ?x 4.75) (< ?x -4.75)))
	     (y ?y&:(and (< ?y 3.5) (> ?y 2.2)))))
  =>
  (printout warn "Stopping delivering puck because the order is over." crlf)
  (modify ?t (state finished))
  (modify ?wfl (state finished))
  (retract ?s ?gt)
  (assert (state TASK-FINISHED))
)

(defrule task-cancle-before-deliver--store-after-order-over
  "If the order is over, we were about to deliver a puck and there is a free storage point, store the puck instead (cancle taks first). Additional rule before delivering to avoid calling two skills immediately."
  (declare (salience ?*PRIORITY-SUBTASK-3*))
  (phase PRODUCTION)
  ?t <- (task (name pick-and-deliver|get-stored-and-deliver) (args $?a) (state ~finished))
  (holding ?puck&P3)
  ?s <- (state GET-PRODUCED-FINAL|GET-STORED-PUCK-FINAL)
  (game-time $?time)
  (not (order (product ?puck) (quantity-requested ?qr) (id  ?order-id)
	      (begin ?begin&:(<= (- ?begin (+ ?*SKILL-DURATION-DELIVER* ?*SKILL-DURATION-GET-PRODUCED*)) (nth$ 1 ?time)))
	      (end ?end&:(<= (nth$ 1 ?time) ?end))))
  (team-color ?team-color&~nil)
  (puck-storage (puck NONE) (team ?team-color) (incoming $?i-st&~:(member$ STORE_PUCK ?i-st)))
  =>
  (printout warn "Stopping delivering puck because the order is over" crlf)
  (modify ?t (state finished))
  (retract ?s)
  (assert (state TASK-FINISHED))
)

(defrule task-cancle-store--deliver-for-new-order
  "If a new order pops up for a puck we are currently storing, deliver it instead (cancle task first)"
  (declare (salience ?*PRIORITY-SUBTASK-3*))
  (phase PRODUCTION)
  ?t <- (task (name pick-and-store|store) (args $?a) (state ~finished))
  (holding ?puck)
  ?s <- (state STORE-PUCK)
  (game-time $?time)
  (order (product ?puck) (quantity-requested ?qr) (id  ?order-id)
	 (in-delivery ?in-delivery&:(< ?in-delivery ?qr)) (begin ?begin)
	 (end ?end&:(tac-can-use-timeslot (nth$ 1 ?time) ?begin ?end ?*SKILL-DURATION-DELIVER*)))
  ?st <- (store-puck-target ?goal)
  ?wfl <- (wait-for-lock (res ?goal) (state use))
  (skill (name "store_puck") (status RUNNING))
  =>
  (printout warn "Stopping storing puck because there is a new order" crlf)
  (modify ?t (state finished))
  (modify ?wfl (state finished))
  (retract ?s ?st)
  (assert (state TASK-FINISHED))
)

(defrule task-pick-and-store--deliver-after-get-produced
  "If a new order pops up for a puck we have got, deliver it instead. (rule task-pick-and-store--deliver does not work here because then two skills get called together and the last one gets dropped)"
  (declare (salience ?*PRIORITY-SUBTASK-3*))
  (phase PRODUCTION)
  ?t <- (task (name pick-and-store) (args $?a) (state ~finished))
  (holding ?puck)
  ?s <- (state GET-PRODUCED-FINAL)
  (game-time $?time)
  (order (product ?puck) (quantity-requested ?qr) (id  ?order-id)
	 (in-delivery ?in-delivery&:(< ?in-delivery ?qr)) (begin ?begin)
	 (end ?end&:(tac-can-use-timeslot (nth$ 1 ?time) ?begin ?end ?*SKILL-DURATION-DELIVER*)))
  =>
  (printout warn "Stopping storing puck because there is a new order" crlf)
  (modify ?t (state finished))
  (retract ?s)
  (assert (state TASK-FINISHED))
)

(defrule task-abort-loading-machine-if-out-of-order
  "If a new order pops up for a puck we are currently storing, deliver it instead (cancle task first)"
  (declare (salience ?*PRIORITY-SUBTASK-3*))
  (phase PRODUCTION)
  ?t <- (task (name load-with-S0|load-with-S1|load-with-S2|pick-and-load|produce-with-S0) (args $?a) (state ~finished))
  ?s <- (state GOTO)
  ?st <- (goto-target ?target)
  (machine (name ?target) (out-of-order-until $?ooo&~:(eq (nth$ 1 ?ooo) 0)))
  ?wfl <- (wait-for-lock (res ?target) (state use))
  ?gtdw <- (goto-dont-wait ?dont-wait)
  (skill (name "finish_puck_at") (status RUNNING))
  =>
  (printout warn "Stopping loading machine " ?target ", because it is out of order." crlf)
  (modify ?t (state finished))
  (modify ?wfl (state finished))
  (retract ?s ?st ?gtdw)
  (assert (state TASK-FINISHED))
)