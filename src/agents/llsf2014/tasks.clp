;---------------------------------------------------------------------------
;  tasks.clp - rules to execute tasks
;
;  Created: Mon Feb 10 17:27:20 2014
;  Copyright  2014 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; Overview over all tasks:
; - load-with-S0              get S0, bring it to T2/T3/T4/T5
; - load-with-T1              get S0, produce at T1 and bring to T2/T3/T4
; - pick-and-load             get S2 from T2 and bring it to T3/T4
; - pick-and-deliver          get P1/P2/P3 from T3/T4/T5 and deliver it
; - recycle                   get CO and recycle

;;;;;;;;;;;;;;
;load-with-S0:
;;;;;;;;;;;;;;
(defrule task-load-with-S0--start-get-S0
  (declare (salience ?*PRIORITY-SUBTASK-1*))
  (phase PRODUCTION)
  ?t <- (task (name load-with-S0) (args $?a) (state ~finished))
  (holding NONE)
  ?s <- (state TASK-ORDERED)
  =>
  (retract ?s)
  (assert (execute-skill get_s0 Ins1)
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (res Ins1))
  )
  (modify ?t (state running))
)
(defrule task-load-with-S0--bring-to-machine
  (declare (salience ?*PRIORITY-SUBTASK-2*))
  (phase PRODUCTION)
  ?t <- (task (name load-with-S0) (args $?a) (state ~finished))
  (holding S0)
  (machine (name ?m&:(eq ?m (nth$ 1 ?a))) (mtype ?mtype))
  ?s <- (state GET-S0-FINAL|TASK-ORDERED)
  =>
  (retract ?s)
  (assert (execute-skill finish_puck_at ?m ?mtype true)
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (res ?m))
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
  ?t <- (task (name load-with-S1) (args $?a) (state ~finished))
  (holding NONE)
  ?s <- (state TASK-ORDERED)
  =>
  (retract ?s)
  (assert (execute-skill get_s0 Ins1)
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (res Ins1))
  )
  (modify ?t (state running))
)
(defrule task-load-with-S1--produce-S0-at-T1
  (declare (salience ?*PRIORITY-SUBTASK-2*))
  (phase PRODUCTION)
  ?t <- (task (name load-with-S1) (args $?a) (state ~finished))
  (holding S0)
  (machine (name ?m&:(eq ?m (nth$ 1 ?a))) (mtype ?mtype))
  ?s <- (state GET-S0-FINAL|TASK-ORDERED)
  =>
  (retract ?s)
  (assert (execute-skill finish_puck_at ?m ?mtype false)
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (res ?m))
  )
  (modify ?t (state running))
)
(defrule task-load-with-S1--bring-S1-to-machine
  (declare (salience ?*PRIORITY-SUBTASK-2*))
  (phase PRODUCTION)
  ?t <- (task (name load-with-S1) (args $?a) (state ~finished))
  (holding S1)
  (machine (name ?m&:(eq ?m (nth$ 2 ?a))) (mtype ?mtype))
  ?s <- (state GOTO-FINAL|TASK-ORDERED)
  =>
  (retract ?s)
  (assert (execute-skill finish_puck_at ?m ?mtype true)
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (res ?m))
  )
  (modify ?t (state running))
)
(defrule task-load-with-S1--finish
  (declare (salience ?*PRIORITY-SUBTASK-3*))
  (phase PRODUCTION)
  ?t <- (task (name load-with-S1) (args $?a) (state ~finished))
  (holding NONE)
  ;is the machine loaded with S1?
  (machine (name ?m&:(eq ?m (nth$ 2 ?a))) (loaded-with $?lw&:(subsetp (create$ S1) ?lw)))
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
  ?t <- (task (name pick-and-deliver) (args $?a) (state ~finished))
  (holding NONE)
  ?s <- (state TASK-ORDERED)
  =>
  (retract ?s)
  (assert (execute-skill get_produced (nth$ 1 ?a))
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (res (nth$ 1 ?a)))
  )
  (modify ?t (state running))
)
(defrule task-pick-and-deliver--deliver
  (declare (salience ?*PRIORITY-SUBTASK-2*))
  (phase PRODUCTION)
  ?t <- (task (name pick-and-deliver) (args $?a) (state ~finished))
  (machine (name ?m&:(eq ?m (nth$ 1 ?a))) (output ?output))
  (holding ?output)
  ?s <- (state GET-PRODUCED-FINAL)
  =>
  (retract ?s)
  (assert (execute-skill deliver deliver1)
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (res deliver1))
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
  ?t <- (task (name pick-and-load) (args $?a) (state ~finished))
  (holding NONE)
  ?s <- (state TASK-ORDERED)
  =>
  (retract ?s)
  (assert (execute-skill get_produced (nth$ 1 ?a))
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (res (nth$ 1 ?a)))
  )
  (modify ?t (state running))
)
(defrule task-pick-and-load--load-machine
  (declare (salience ?*PRIORITY-SUBTASK-2*))
  (phase PRODUCTION)
  ?t <- (task (name pick-and-load) (args $?a) (state ~finished))
  (machine (name ?m1&:(eq ?m1 (nth$ 1 ?a))) (output ?puck))
  (holding ?puck)
  (machine (name ?m2&:(eq ?m2 (nth$ 2 ?a))) (mtype ?mtype))
  ?s <- (state GET-PRODUCED-FINAL)
  =>
  (retract ?s)
  (assert (execute-skill finish_puck_at ?m2 ?mtype true)
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (res ?m2))
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
  ?t <- (task (name recycle) (args $?a) (state ~finished))
  (holding NONE)
  ?s <- (state TASK-ORDERED)
  =>
  (retract ?s)
  (assert (execute-skill get_consumed (nth$ 1 ?a))
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (res (nth$ 1 ?a)))
  )
  (modify ?t (state running))
)
(defrule task-recycle--recycle
  (declare (salience ?*PRIORITY-SUBTASK-2*))
  (phase PRODUCTION)
  ?t <- (task (name recycle) (args $?a) (state ~finished))
  (machine (name ?m&:(eq ?m (nth$ 2 ?a))) (mtype RECYCLE))
  (holding CO)
  ?s <- (state GET-CONSUMED-FINAL)
  =>
  (retract ?s)
  (assert (execute-skill finish_puck_at ?m RECYCLE false)
          (state WAIT-FOR-LOCK)
	  (wait-for-lock (res ?m))
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