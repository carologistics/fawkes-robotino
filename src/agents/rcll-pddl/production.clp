;---------------------------------------------------------------------------
;  production.clp - Planning for PCCL production
;                   Plan execution is handled in this file
;                   Action execution is located in tasks.clp
;
;  Created: Sat Jun 16 12:35:16 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;                   Frederik Zwilling
;                   Matthias Loebach
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defrule prod-propose-task-idle
  "If we are idle change state to the proposed task."
  (declare (salience ?*PRIORITY-HIGH*))
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

(defrule prod-move-to-position-empty
  (declare (salience ?*PRIORITY-TASK*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (holding NONE)
  ?sa <- (stn-action (id ?id) (name move-to-position-empty) (state pending) 
              (cond-actions) (opts ?r ?from ?to))
  (not (stn-action (id ?oid&:(< ?oid ?id)) (state pending)))
  =>
  (printout t "PROD: Move empty to " ?to crlf)
  (bind ?task-id (random-id))
  (assert (task (name move-to-position-empty) (id ?task-id) (state proposed)
		        (steps (create$ (+ ?task-id 1)))
		        (priority ?*PRIORITY-TASK*))
	  (step (name drive-to) (id (+ ?task-id 1))
		  (task-priority ?*PRIORITY-TASK*)
		  (machine (get-mps-from-place ?to)) (side (get-side-from-place ?to)))
  )
  (synced-modify ?sa state running)
)

(defrule prod-move-to-position-holding
  (declare (salience ?*PRIORITY-TASK*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (holding ?base)
  ?sa <- (stn-action (id ?id) (name move-to-position-holding) (state pending) 
              (cond-actions) (opts ?r ?from ?to))
  (not (stn-action (id ?oid&:(< ?oid ?id)) (state pending)))
  =>
  (printout t "PROD: Move holding " ?base " to " ?to crlf)
  (bind ?task-id (random-id))
  (assert (task (name move-to-position-holding) (id ?task-id) (state proposed)
		        (steps (create$ (+ ?task-id 1)))
		        (priority ?*PRIORITY-TASK*))
	  (step (name drive-to) (id (+ ?task-id 1))
		  (task-priority ?*PRIORITY-TASK*)
		  (place ?to))
  )
  (synced-modify ?sa state running)
)

(defrule prod-pick-cc-from-shelf
  (declare (salience ?*PRIORITY-TASK*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (holding NONE)
  ?sa <- (stn-action (id ?id) (name pick-cc-from-shelf) (state pending)
              (cond-actions) (opts ?r ?mps ?to ?slot ?cc))
  (not (stn-action (id ?oid&:(< ?oid ?id)) (state pending)))
  =>
  (printout t "PROD: Pick " ?cc " from shelf " ?slot crlf)
  (bind ?task-id (random-id))
  (assert (task (name pick-cc-from-shelf) (id ?task-id) (state proposed)
		        (steps (create$ (+ ?task-id 1)))
		        (priority ?*PRIORITY-TASK*))
	  (step (name get-from-shelf) (id (+ ?task-id 1))
		  (task-priority ?*PRIORITY-TASK*)
	    (machine ?mps) (machine-feature SHELF))
  )
  (synced-modify ?sa state running)
)

(defrule prod-load-cs
  (declare (salience ?*PRIORITY-TASK*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (holding ?base)
  ?sa <- (stn-action (id ?id) (name load-cs) (state pending)
              (cond-actions) (opts ?r ?mps ?to ?cap-col ?other ?cc))
  (not (stn-action (id ?oid&:(< ?oid ?id)) (state pending)))
  =>
  (printout t "PROD: Load CS with cap " ?cap-col crlf)
  (bind ?task-id (random-id))
  (assert (task (name load-cs) (id ?task-id) (state proposed)
		        (steps (create$ (+ ?task-id 1)))
		        (priority ?*PRIORITY-TASK*))
	  (step (name insert) (id (+ ?task-id 1))
		  (task-priority ?*PRIORITY-TASK*)
	    (machine ?mps)
      (machine-feature CONVEYOR)
      (already-at-mps TRUE))
  )
  (synced-modify ?sa state running)
)

(defrule prod-pick-wp-from-cs
  (declare (salience ?*PRIORITY-TASK*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (holding NONE)
  ?sa <- (stn-action (id ?id) (name pick-wp-from-cs) (state pending)
              (cond-actions) (opts ?r ?mps ?to ?cc ?other))
  (not (stn-action (id ?oid&:(< ?oid ?id)) (state pending)))
  =>
  (printout t "PROD: Pick base from CS" crlf)
  (bind ?task-id (random-id))
  (assert (task (name pick-wp-from-cs) (id ?task-id) (state proposed)
		        (steps (create$ (+ ?task-id 1)))
		        (priority ?*PRIORITY-TASK*))
	  (step (name get-output) (id (+ ?task-id 1))
		  (task-priority ?*PRIORITY-TASK*)
	    (machine ?mps))
  )
  (synced-modify ?sa state running)
)

(defrule prod-pick-wp-from-bs
  (declare (salience ?*PRIORITY-TASK*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (holding NONE)
  ?sa <- (stn-action (id ?id) (name pick-wp-from-bs) (state pending)
              (cond-actions) (opts ?r ?bs ?to ?col))
  (not (stn-action (id ?oid&:(< ?oid ?id)) (state pending)))
  (base-station (name ?bs) (active-side ?bs-side) (fail-side ?fs&:(neq ?bs-side ?fs)))
  =>
  (printout t "PROD: Pick base from BS" crlf)
  (bind ?task-id (random-id))
  (bind ?base-color (sub-string 1 (- (str-index "-" ?col) 1) ?col))
  (assert (task (name pick-wp-from-bs) (id ?task-id) (state proposed)
		        (steps (create$ (+ ?task-id 1) (+ ?task-id 2) (+ ?task-id 3)))
		        (priority ?*PRIORITY-TASK*))
    (step (name acquire-lock) (id (+ ?task-id 1))
      (task-priority ?*PRIORITY-TASK*) (lock PREPARE-BS)) ;is released after get-base
    (step (name instruct-mps) (id (+ ?task-id 2))
      (task-priority ?*PRIORITY-TASK*)
      (machine ?bs) (base ?base-color) (side ?bs-side))
    (step (name get-base) (id (+ ?task-id 3))
      (task-priority ?*PRIORITY-TASK*)
      (machine ?bs) (machine-feature CONVEYOR)
      (base ?base-color) (side ?bs-side)) ;product-id left out
  )
  (synced-modify ?sa state running)
)
