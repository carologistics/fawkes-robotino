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


(defrule prod-prefill-cap-station
  "Feed a CS with a cap from its shelf so that afterwards it can directly put the cap on a product."
  (declare (salience ?*PRIORITY-PREFILL-CS*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (holding NONE)
  ?m-f <- (machine (mtype CS) (loaded-id 0) (incoming $?i&~:(member$ FILL_CAP ?i))
                   (name ?machine) (produced-id 0) (team ?team-color)
    (out-of-order-until $?ooo&:(is-working ?ooo)))
  (cap-station (name ?machine) (cap-loaded NONE)
	  (assigned-cap-color ?cap-color&~NONE))
  ;check that the task was not rejected before
  (not (and (task (name fill-cap) (state rejected) (id ?rej-id))
            (step (name insert) (id ?rej-st&:(eq ?rej-st (+ ?rej-id 2))) (machine ?machine))))
  (not (task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-PREFILL-CS*))))
  ?c3 <- (confval (path "/clips-agent/llsf2015/cap-station/shelf-slot-new-caps") (value ?shelf-slot))
  (found-tag (name ?machine))
  =>
  (printout t "PROD: FILL " ?machine " with " ?cap-color " cap from shelf" crlf)
  (bind ?task-id (random-id))
  (assert (task (name fill-cap) (id ?task-id) (state proposed)
		        (steps (create$ (+ ?task-id 1) (+ ?task-id 2) (+ ?task-id 3)))
		        (priority ?*PRIORITY-PREFILL-CS*))
	  (step (name get-from-shelf) (id (+ ?task-id 1))
		  (task-priority ?*PRIORITY-PREFILL-CS*)
		  (machine ?machine) (shelf-slot (sym-cat ?shelf-slot)))
	  (step (name insert) (id (+ ?task-id 2))
		  (task-priority ?*PRIORITY-PREFILL-CS*)
		  (machine ?machine)
      (machine-feature CONVEYOR))
	  (step (name get-output) (id (+ ?task-id 3))
		  (task-priority ?*PRIORITY-PREFILL-CS*)
		  (machine ?machine))
	  (needed-task-lock (task-id ?task-id) (action FILL_CAP) (place ?m-f))
  )
)

(defrule insert-unknown-base-to-rs
  "Insert a base with unknown color in a RS for preparation"
  (declare (salience ?*PRIORITY-PREFILL-RS*))
  (phase PRODUCTION)
  (state IDLE)
  (team-color ?team-color&~nil)
  (holding ?product-id&~NONE)
  (product (id ?product-id) (base UNKNOWN))
  ?m-f <- (machine (mtype RS)
                   (name ?rs) (team ?team-color)
                   (out-of-order-until $?ooo&:(is-working ?ooo)))
  (ring-station (name ?rs) (bases-needed ?bases&:(> ?bases 0)))
  ;check that the task was not rejected before
  (not (and 
    (task (name fill-rs) (state rejected) (id ?rej-id))
    (step (name insert) (id ?rej-st&:(eq ?rej-st (+ ?rej-id 1))) (machine ?rs) (machine-feature SLIDE))
  ))
  (not (task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-PREFILL-RS*))))
  (found-tag (name ?rs))
  =>
  (printout t "PROD: INSERT unknown base " ?product-id " into " ?rs crlf)
  (bind ?task-id (random-id))
  (assert 
    (task (name fill-rs) (id ?task-id) (state proposed)
      (steps (create$ (+ ?task-id 1)))
      (priority ?*PRIORITY-PREFILL-RS*))
    (step (name insert) (id (+ ?task-id 1))
      (task-priority ?*PRIORITY-PREFILL-RS*)
      (machine ?rs)
      (machine-feature SLIDE))
    (needed-task-lock (task-id ?task-id) (action PREFILL-RS) (place ?m-f))
  )
)

(defrule discard-unknown-base
  "Discard a base with unknown color if no RS has to be pre-filled"
  (declare (salience ?*PRIORITY-DISCARD-UNKNOWN*))
  (phase PRODUCTION)
  (state IDLE)
  (team-color ?team-color&~nil)
  (holding ?product-id&~NONE)
  (product (id ?product-id) (base UNKNOWN))
  (not (ring-station (bases-needed ?bases&:(> ?bases 0))))
  =>
  (printout t "PROD: Discard unneeded unknown base " ?product-id crlf)
  (bind ?task-id (random-id))
  (assert
    (task (name discard-unknown) (id ?task-id) (state proposed)
      (steps (create$ (+ ?task-id 1)))
      (priority ?*PRIORITY-DISCARD-UNKNOWN*))
    (step (name discard) (id (+ ?task-id 1))
      (task-priority ?*PRIORITY-DISCARD-UNKNOWN*))
  )
)
  
(defrule prod-produce-c0
  "Produce a C0"
  (declare (salience ?*PRIORITY-PRODUCE-C0*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (holding NONE)
  (game-time $?game-time)
  ?m-f <- (machine (mtype CS) (incoming $?i&~:(member$ PROD_CAP ?i))
                   (name ?cs) (team ?team-color) (produced-id 0)
                   (out-of-order-until $?ooo&:(is-working ?ooo)))
  (cap-station (name ?cs) (cap-loaded ?cap-color) (assigned-cap-color ?cap-color))
  (found-tag (name ?cs))
  (machine (mtype BS) 
    (name ?bs) (team ?team-color)
    (out-of-order-until $?ooo&:(is-working ?ooo)))
  (found-tag (name ?bs))
  ;check that the task was not rejected before
  (not (and (task (name produce-c0) (state rejected) (id ?rej-id))
            (step (name insert) (id ?rej-st&:(eq ?rej-st (+ ?rej-id 2))) (machine ?cap-station))))
  (not (task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-PRODUCE-C0*))))
  ;check for open C0 order
  (product (id ?product-id) (rings $?r&:(eq 0 (length$ ?r))) (cap ?cap-color) (base ?base-color))
  (order (product-id ?product-id)
    (quantity-requested ?qr) (quantity-delivered ?qd&:(> ?qr ?qd))
    (begin ?begin&:(< ?begin (+ (nth$ 1 ?game-time) 40)))
    (end ?end)
    (in-production 0)
  )
  =>
  (printout warn "TODO: production durations not yet implemented")
  (printout t "PROD: PRODUCE C0 with " ?cap-color " cap at " ?cs crlf)
  (bind ?task-id (random-id))
  (assert (task (name produce-c0) (id ?task-id) (state proposed)
    (steps (create$ (+ ?task-id 1) (+ ?task-id 2)))
    (priority ?*PRIORITY-PRODUCE-C0*))
    (step (name get-base) (id (+ ?task-id 1))
      (task-priority ?*PRIORITY-PRODUCE-C0*)
      (machine ?bs) (machine-feature CONVEYOR)
      (base ?base-color))
    (step (name insert) (id (+ ?task-id 2))
      (task-priority ?*PRIORITY-PRODUCE-C0*)
      (machine ?cs)
      (machine-feature CONVEYOR))
    (needed-task-lock (task-id ?task-id) (action PROD-CAP) (place ?m-f))
  )
)

(defrule prod-deliver-c0
  "Deliver C0"
  (declare (salience ?*PRIORITY-DELIVER*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (holding NONE)
  (game-time $?game-time)
  ?cs-f <- (machine (mtype CS) (produced-id ?produced-id&~0)
                    (name ?cs) (team ?team-color)
                    (out-of-order-until $?ooo&:(is-working ?ooo)))
  (product
    (id ?produced-id)
    (rings $?r&:(eq 0 (length$ ?r)))
    (cap ?cap-color)
    (base ?base-color)
  )
  ?ds-f <- (machine (mtype DS)
                    (name ?ds) (team ?team-color)
                    (out-of-order-until $?ooo&:(is-working ?ooo)))
  ;check that the task was not rejected before
  (not (and (task (name deliver-c0) (state rejected) (id ?rej-id))
            (step (name insert) (id ?rej-st&:(eq ?rej-st (+ ?rej-id 2))) (machine ?cs))))
  (not (task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-DELIVER*))))
  (product
    (id ?product-id)
    (rings $?r)
    (cap ?cap-color)
    (base ?base-color)
  )
  (order (product-id ?product-id)
    (quantity-requested ?qr) (quantity-delivered ?qd&:(> ?qr ?qd))
    (begin ?begin&:(< ?begin (+ (nth$ 1 ?game-time) 40)))
    (end ?end)
    (delivery-gate ?gate)
  )
  =>
  (printout t "PROD: DELIVER C0 with " ?cap-color crlf)
  (bind ?task-id (random-id))
  (assert (task (name deliver-c0) (id ?task-id) (state proposed)
    (steps (create$ (+ ?task-id 1) (+ ?task-id 2)))
    (priority ?*PRIORITY-DELIVER*))
    (step (name get-output) (id (+ ?task-id 1))
      (task-priority ?*PRIORITY-DELIVER*)
      (machine ?cs))
    (step (name insert) (id (+ ?task-id 2))
      (task-priority ?*PRIORITY-DELIVER*)
      (machine ?ds)
      (machine-feature CONVEYOR))
    (needed-task-lock (task-id ?task-id) (action GET-PROD) (place ?cs-f))
    (needed-task-lock (task-id ?task-id) (action DELIVER) (place ?ds-f))
  )
)
  
(defrule prod-find-missing-mps-exploration-catch-up
  "If we have not found all mps until the production phase, we have to find them now."
  (declare (salience ?*PRIORITY-FIND-MISSING-MPS*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (holding NONE)
  ; there is a mps not found jet
  (machine (name ?missing-mps) (team ?team-color) (mtype ~RS))
  (not (found-tag (name ?missing-mps)))
  ; zone-to-explore
  ?z-f <- (zone-exploration (name ?zone) (still-to-explore TRUE) (team ?team-color)
                            (incoming $?i&~:(member$ FIND_TAG ?i))
                            (times-searched ?times-searched))
  ; no-zone searched less times
  (not (zone-exploration (name ?z2&:(neq ?zone ?z2)) (still-to-explore TRUE) (team ?team-color)
                         (incoming $?i&~:(member$ FIND_TAG ?i))
                         (times-searched ?less-times-searched&:(< ?less-times-searched ?times-searched))))
  ;check that the task was not rejected before
  (not (and (task (name exploration-catch-up) (state rejected) (id ?rej-id))
	    (step (name find-tag) (id ?rej-st&:(eq ?rej-st (+ ?rej-id 1))) (zone ?zone))))
  (not (task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-FIND-MISSING-MPS*))))
  =>
  (printout t "PROD: " ?missing-mps " still not found!" 
            " Searching for it in zone " ?zone crlf)
  (bind ?task-id (random-id))
  (assert (task (name exploration-catch-up) (id ?task-id) (state proposed)
		(steps (create$ (+ ?task-id 1)))
		(priority ?*PRIORITY-FIND-MISSING-MPS*))
	  (step (name find-tag) (id (+ ?task-id 1))
		(task-priority ?*PRIORITY-FIND-MISSING-MPS*)
		(zone ?zone) (machine ?missing-mps))
	  (needed-task-lock (task-id ?task-id) (action FIND_TAG) (place ?z-f))
  )
)

(defrule prod-nothing-to-do-save-factbase
  "If the agent can't find any task, save the factbase to find problems"
  (declare (salience ?*PRIORITY-NOTHING-TO-DO*))
  (phase PRODUCTION)
  (state IDLE)
  (time $?now)
  (wait-point ?wait-point)
  (not (no-task-found))
  (not (task (state proposed|asked|rejected|ordered|running)))
  =>
  (printout error "Can't find any task!." crlf)
  (printout error " Waiting..." crlf)
  (save-facts (str-cat "agent-snapshot-no-task" (nth$ 1 ?now) ".clp") visible)
  (skill-call goto place (str-cat ?wait-point))
  (assert (no-task-found))
)

(defrule prod-remove-nothing-to-do-fact
  ?no-task <- (no-task-found)
  (state ~IDLE)
  =>
  (retract ?no-task)
)
