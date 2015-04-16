;---------------------------------------------------------------------------
;  production.clp - Planning for LLSF production
;                   Here we plan which job to execute when the robot is idle
;                   The job-execution is located in tasks.clp
;
;  Created: Sat Jun 16 12:35:16 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;                   Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; decision rules for the next most important task to propose
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrule prod-propose-task-idle
  "If we are idle change state to the proposed task."
  (declare (salience ?*PRIORITY-LOW*))
  (phase PRODUCTION)
  ?sf <- (state IDLE)
  (task (state proposed))
  =>
  (retract ?sf)
  (assert (state TASK-PROPOSED))
)

(defrule prod-change-to-more-important-task-when-waiting-for-lock
  "If we run a low-priority task and look for an alternative AND a task with a higher priority is proposed, drop the current work and change to the priorized task."
  (declare (salience ?*PRIORITY-LOW*))
  (phase PRODUCTION)
  ?t <- (task (state running) (priority ?old-p))
  ?pt <- (task (state proposed) (priority ?p&:(> ?p ?old-p)))
  ?sf1 <- (state WAIT_AND_LOOK_FOR_ALTERATIVE)
  ?sf2 <- (state WAIT-FOR-LOCK)
  ?lock-get <- (lock (type GET) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?res))
  ?lock-ref <- (lock (type REFUSE) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?res))
  ?wfl <- (wait-for-lock (res ?res) (state get))
  ?exec <- (execute-skill ? ?)
  =>
  (retract ?sf1 ?sf2 ?wfl ?lock-get ?exec ?lock-ref)
  (modify ?t (state finished))
  (assert
    (state TASK-PROPOSED)
	  (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?res))
  )
)

(defrule prod-remove-proposed-tasks
  "Remove all proposed tasks, when you are neither idle nor looking for an alternative."
  (declare (salience ?*PRIORITY-LOW*))
  (phase PRODUCTION)
  (not (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE))
  ?pt <- (task (state proposed))
  =>
  (retract ?pt)
)


;(defrule prod-prefill-cap-station
;  "Feed a CS with a cap from its shelf so that afterwards it can directly put the cap on a product."
;  (declare (salience ?*PRIORITY-PREFILL-CS*))
;  (phase PRODUCTION)
;  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;  (team-color ?team-color&~nil)
;  (holding NONE)
;  (machine (mtype CS) (loaded-id 0) (incoming $?i&~:(member$ FILL_CAP ?i))
;	   (name ?machine) (produced-id 0) (team ?team-color)
;	   (out-of-order-until $?ooo&:(is-working ?ooo)))
;  (cap-station (name ?machine) (cap-loaded NONE)
;	       (assigned-cap-color ?cap-color&~NONE))
;  ;check that the task was not rejected before
;  (not (and (task (name fill-cap) (state rejected) (id ?rej-id))
;	    (step (name insert) (id ?rej-st&:(eq ?rej-st (+ ?rej-id 2))) (machine ?machine))))
;  (not (task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-PREFILL-CS*))))
;  ?c3 <- (confval (path "/clips-agent/llsf2015/cap-station/shelf-slot-new-caps") (value ?shelf-slot))
;  =>
;  (printout t "PROD: FILL " ?machine " with " ?cap-color " cap from shelf" crlf)
;  (bind ?task-id (random-id))
;  (assert (task (name fill-cap) (id ?task-id) (state proposed)
;		(steps (create$ (+ ?task-id 1) (+ ?task-id 2) (+ ?task-id 3)))
;		(priority ?*PRIORITY-PREFILL-CS*))
;	  (step (name get-from-shelf) (id (+ ?task-id 1))
;		(task-priority ?*PRIORITY-PREFILL-CS*)
;		(machine ?machine) (shelf-slot (sym-cat ?shelf-slot)))
;	  (step (name insert) (id (+ ?task-id 2))
;		(task-priority ?*PRIORITY-PREFILL-CS*)
;		(machine ?machine))
;	  (step (name get-output) (id (+ ?task-id 3))
;		(task-priority ?*PRIORITY-PREFILL-CS*)
;		(machine ?machine))
;	  (needed-task-lock (task-id ?task-id) (action FILL_CAP) (place ?machine))
;  )
;)

(defrule prod-produce-c0
  "Produce a C0"
  (declare (salience ?*PRIORITY-PRODUCE-C0*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (holding NONE)
  (game-time $?game-time)
  (machine (mtype CS) (incoming $?i&~:(member$ FILL_CAP ?i))
    (name ?machine) (team ?team-color)
    (out-of-order-until $?ooo&:(is-working ?ooo)))
  (cap-station (name ?machine) (cap-loaded ?cap-color) (assigned-cap-color ?cap-color))
  ;check that the task was not rejected before
  (not (and (task (name produce-c0) (state rejected) (id ?rej-id))
            (step (name insert) (id ?rej-st&:(eq ?rej-st (+ ?rej-id 2))) (machine ?machine))))
  (not (task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-PRODUCE-C0*))))
  ;check for open C0 order
  (product (id ?product-id) (rings $?r&:(eq 0 (length$ ?r))) (cap ?cap-color))
  (order (product-id ?product-id)
    (quantity-requested ?qr) (quantity-delivered ?qd&:(> ?qr ?qd))
    (begin ?begin) 
    (end ?end&:(tac-can-use-timeslot (nth$ 1 ?game-time) ?begin ?end (+ ?*SKILL-DURATION-GET-PRODUCED* ?*SKILL-DURATION-DELIVER*)))
    (in-production 0)
  )
  =>
  (printout warn "TODO: production durations not yet implemented")
  (printout t "PROD: PRODUCE C0 with " ?cap-color " at " ?machine crlf)
  (bind ?task-id (random-id))
  (assert (task (name produce-c0) (id ?task-id) (state proposed)
    (steps (create$ (+ ?task-id 1) (+ ?task-id 2)))
    (priority ?*PRIORITY-PRODUCE-C0*))
    (step (name get-base) (id (+ ?task-id 1))
      (task-priority ?*PRIORITY-PRODUCE-C0*)
      (machine ?machine))
    (step (name insert) (id (+ ?task-id 2))
      (task-priority ?*PRIORITY-PRODUCE-C0*)
      (machine ?machine)
      (machine-feature CONVEYOR))
    (needed-task-lock (task-id ?task-id) (action PROD-C0) (place ?machine))
  )
)


; (defrule prod-produce-P3-and-deliver
;   "Complete production of a P3 at the beginning without leaving the T5 machine while producing."
;   (declare (salience ?*PRIORITY-PRODUCE-T5-AT-BEGIN*))
;   (phase PRODUCTION)
;   (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;   (team-color ?team-color&~nil)
;   (machine (mtype T5) (loaded-with $?l&~:(member$ S0 ?l))
;     (incoming $?i&~:(member$ BRING_S0 ?i)) (name ?name) (produced-puck NONE) (team ?team-color)
;     (out-of-order-until $?ooo&:(eq (nth$ 1 ?ooo) 0))
;   )
;   (not (and (task (name produce-with-S0) (state rejected) (id ?rej-id))
; 	    (step (name produce-at) (id ?rej-st&:(eq ?rej-st (+ ?rej-id 2))) (machine ?name))))
;   (holding NONE|S0)
;   (not (task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-PRODUCE-T5-AT-BEGIN*))))
;   (not (no-more-needed P3))
;   =>
;   (printout t "PROD: PRODUCE P3 at T5 " ?name " waiting and deliver afterweards" crlf)
;   ;generate random task id
;   (bind ?task-id (random 0 1000000000))
;   (assert (task (name produce-p3-and-deliver) (id ?task-id) (state proposed)
; 		(steps (create$ (+ ?task-id 1) (+ ?task-id 2) (+ ?task-id 3)))
; 		(priority ?*PRIORITY-PRODUCE-T5-AT-BEGIN*))
; 	  (step (name get-s0) (id (+ ?task-id 1))
; 		(task-priority ?*PRIORITY-PRODUCE-T5-AT-BEGIN*))
; 	  (step (name produce-at) (id (+ ?task-id 2)) (machine ?name))
; 	  (step (name deliver) (id (+ ?task-id 3)) (product-type P3)
; 		(task-priority ?*PRIORITY-DELIVER-P3*))
; 	  (needed-task-lock (task-id ?task-id) (action BRING_S0) (place ?name))
;   )
; )
