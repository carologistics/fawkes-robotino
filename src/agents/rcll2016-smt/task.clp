;---------------------------------------------------------------------------
;  task.clp - abstract task execution engine
;
;  Created: Sat Feb 07 20:59:39 2015
;  Copyright  2015 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------
; All tasks are proposed first in production.clp
; Task execution is decided in cooperation.clp
; Here the task is executed by switching through the steps of a task
; The steps themselves are executed/finished/aborted in steps.clp
;---------------------------------------------------------------------------

(defrule task-start
  "Start execution of a task by activating its first step"
  (phase PRODUCTION)
  ?state <- (state TASK-ORDERED)
  ?task <- (task (state ordered) (steps $?steps) (name ?name) (robot ?r&:(eq ?r ?*ROBOT-NAME*)))
  ?step <- (step (id ?step-id&:(eq ?step-id (nth$ 1 ?steps))) (state inactive))
  =>
  (retract ?state)
  (assert (state STEP-STARTED))
  (modify ?task (state running) (current-step (nth$ 1 ?steps)))
  (modify ?step (state wait-for-activation))
  (printout info "Timelog: Task " ?name " started." crlf)
)

(defrule task-switch-to-next-step
  "If a step finished, we activate the next step of the task"
  (phase PRODUCTION)
  ?state <- (state STEP-FINISHED)
  ?task <- (task (state running) (steps $?steps) (current-step ?id-finished) (name ?name) (robot ?r&:(eq ?r ?*ROBOT-NAME*)))
  (step (id ?id-finished) (state finished))
  ?step-next <- (step (id ?id-next&:(eq ?id-next 
					(nth$ (+ 1 (member$ ?id-finished ?steps)) ?steps)))
		      (state inactive))
  =>
  (retract ?state)
  (assert (state STEP-STARTED))
  (modify ?task (current-step ?id-next))
  (modify ?step-next (state wait-for-activation))
  (printout info "Timelog: Task " ?name " switched to next step." crlf)
)

(defrule task-finish
  "Finish a task if all steps finished"
  (phase PRODUCTION)
  ?state <- (state STEP-FINISHED)
  ?task <- (task (state running) (steps $?steps) (current-step ?id-finished) (name ?task-name) (robot ?r&:(eq ?r ?*ROBOT-NAME*)))
  (step (id ?id-finished) (state finished))
  ;there is no next task
  (not (step (id ?id-next&:(eq ?id-next 
			       (nth$ (+ 1 (member$ ?id-finished ?steps)) ?steps)))
	     (state inactive)))
  =>
  (retract ?state)
  (assert (state TASK-FINISHED))
  (modify ?task (state finished))
  (printout info "Timelog: Task " ?task-name " finished." crlf)
)

(defrule task-fail-wait
  "Fail a task if one of the steps failed, wait before failing to avoid
  calling two skills directly after each other"
  (phase PRODUCTION)
  ?state <- (state STEP-FAILED)
  ?task <- (task (state running) (current-step ?id-failed) (name ?task-name) (robot ?r&:(eq ?r ?*ROBOT-NAME*)))
  (step (id ?id-failed) (state failed) (name ?step-name))
  (time $?now)
  =>
  (printout warn "Task " ?task-name " failed in step " ?step-name crlf)
  (printout t "Waiting 1s to avoid calling two skills instantly" crlf)
  (retract ?state)
  (assert (state TASK-FAILED-WAITING)
          (timer (name wait-after-fail)))
  (modify ?task (state failed))
  (printout info "Timelog: Task " ?task-name " failed." crlf)
  (printout t "Calling motor_move to stop current skill" crlf)
  (skill-call motor_move x 0.0)
)

(defrule task-fail
  "Fail a task if one of the steps failed, after waiting"
  (phase PRODUCTION)
  ?state <- (state TASK-FAILED-WAITING)
  (time $?now)
  ?tm <- (timer (name wait-after-fail) (time $?t&:(timeout ?now ?t 1.0)))
  =>
  (printout t "Waiting 1s finished" crlf)
  (retract ?state ?tm)
  (assert (state TASK-FAILED))
)
