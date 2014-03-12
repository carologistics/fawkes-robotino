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
; - recycle-and-load-with-S0  get CO, recycle and bring it to T2/T3/T4/T5
; - recycle-and-load-with-T1  get CO, recycle, produce at T1 and bring it to T2/T3/T4

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
  (assert (state GET-S0)
	  (lock-and-execute (skill get-s0) (res Ins1))
  )
  (modify ?t (state running))
)
(defrule task-load-with-S0--bring-to-machine
  (declare (salience ?*PRIORITY-SUBTASK-2*))
  (phase PRODUCTION)
  ?t <- (task (name load-with-S0) (args $?a) (state ~finished))
  (holding S0)
  (machine (name ?m&:(eq ?m (nth$ 1 ?a))))
  ?s <- (state GET-S0-FINAL)
  =>
  (retract ?s)
  (assert (state GOTO)
	  (lock-and-execute (skill load-with) (res ?m))
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
  (assert (state GET-S0)
	  (lock-and-execute (skill get-s0) (res Ins1))
  )
  (modify ?t (state running))
)
(defrule task-load-with-S1--bring-S0-to-T1
  (declare (salience ?*PRIORITY-SUBTASK-2*))
  (phase PRODUCTION)
  ?t <- (task (name load-with-S1) (args $?a) (state ~finished))
  (holding S0)
  (machine (name ?m&:(eq ?m (nth$ 1 ?a))) (mtype ?mtype))
  ?s <- (state GET-S0-FINAL)
  =>
  (retract ?s)
  (assert (state GOTO)
	  (lock-and-execute (skill load-with) (res ?m))
	  ;TODO: use skill which produces and waits for completion
  )
  (modify ?t (state running))
)
(defrule task-load-with-S1--bring-S1-to-machine
  (declare (salience ?*PRIORITY-SUBTASK-2*))
  (phase PRODUCTION)
  ?t <- (task (name load-with-S1) (args $?a) (state ~finished))
  (holding S1)
  (machine (name ?m&:(eq ?m (nth$ 2 ?a))) (mtype ?mtype))
  ?s <- (state GOTO-FINAL)
  =>
  (retract ?s)
  (assert (state GOTO)
	  (lock-and-execute (skill load-with) (res ?m))
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
