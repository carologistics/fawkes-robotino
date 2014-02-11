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
; - recycle-and-load-with-T1  get CO, recycle and bring it to T2/T3/T4/T5


;common template for a task
(deftemplate task
  (slot name (type SYMBOL) (allowed-values load-with-S0 load-with-T1 pick-and-load pick-and-deliver recycle-and-load-with-S0 recycle-and-load-with-T1))
  (multislot args (type SYMBOL)) ;in chronological order
  (slot state (type SYMBOL) (allowed-values ordered running finished) (default ordered))
)

(defrule prod-get-s0-done
  (phase PRODUCTION)
  ?sf <- (state GET-S0-FINAL|GET-S0-FAILED)
;  (Position3DInterface (id "Pose") (translation $?pos))
  =>
  (retract ?sf)
  ; (if (debug 3) then (printout t "Want to release Lock for INS, waiting till 0.5m away" crlf))
  (assert (state IDLE)
	  ; (want-to-release INS $?pos)
  )
)

(defrule prod-goto-final
  (phase PRODUCTION)
  ?sf <- (state GOTO-FINAL)
;  (not (goto-target ?))
;  ?f <- (goto-has-locked ?goal)
;  (Position3DInterface (id "Pose") (translation $?pos))
  =>
  (retract ?sf); ?f)
  (assert (state IDLE)
;	  (want-to-release ?goal $?pos)
  )
)


;load-with-S0:
(defrule task-load-with-S0--start-get-S0
  (declare (salience ?*PRIORITY-SUBTASK-1*))
  (phase PRODUCTION)
  ?t <- (task (name load-with-S0) (args $?a) (state ~finished))
  (holding NONE)
  ?s <- (state IDLE)
  =>
  (retract ?s)
  (assert (state GET-S0))
  (modify ?t (state running))
  (get-s0)
)
(defrule task-load-with-S0--bring-to-machine
  (declare (salience ?*PRIORITY-SUBTASK-2*))
  (phase PRODUCTION)
  ?t <- (task (name load-with-S0) (args $?a) (state ~finished))
  (holding S0)
  (machine (name ?m&:(eq ?m (nth$ 1 ?a))) (mtype ?mtype))
  ?s <- (state IDLE)
  =>
  (retract ?s)
  (assert (state GOTO))
  (modify ?t (state running))
  (goto-machine ?m ?mtype)
  ;TODO: use skill which only loads the machine/starts the production and then leaves the machine
)
(defrule task-load-with-S0--finish
  (declare (salience ?*PRIORITY-SUBTASK-3*))
  (phase PRODUCTION)
  ?t <- (task (name load-with-S0) (args $?a) (state ~finished))
  (holding NONE)
  ;is the machine loaded with S0?
  (machine (name ?m&:(eq ?m (nth$ 1 ?a))) (loaded-with $?lw&:(subsetp (create$ S0) ?lw)))
  (state IDLE)
  =>
  (modify ?t (state finished))
)
