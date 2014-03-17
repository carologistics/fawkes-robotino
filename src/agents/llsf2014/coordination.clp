;---------------------------------------------------------------------------
;  coordination.clp - coordination of task allocation
;                     Ask locking master to accept or reject a chosen task
;
;  Created: Thu Mar 06 20:35:52 2014
;  Copyright  2014 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; Approach:
; 1. production.clp proposes a task
; 2. compute all sub-tasks which have to be coordinated with the other robots
;   (e.g. task: bring produced S2 from M1 to T4 M2 => noone else is on the way to pick the produced puck from M1 and noone is on the way to bring a S2 to M2)
; 3. ask for resource locking of each sub-task to coordinate
;    (e.g. lock resource called PICK-PROD_M1 and S2_M2)
; 4a. one sub-task was rejected => reject proposed task and go on in production.clp
; 4b. all sub-tasks were accepted => order execution of the task and go on in tasks.clp
; 5b. release sub-tasks-lock after execution (this also triggers changing the world state)


(defrule coordination-compute-resources-to-lock
  ?pt <- (proposed-task (name ?task) (args $?a) (state proposed))
  (not (proposed-task (state asked)))
  ?s <- (state TASK-PROPOSED)
  =>
  (modify ?pt (state asked))
  (retract ?s)
  (assert (state TASK-PROPOSED-ASKED))
  (switch ?task
    (case load-with-S0 then
      (assert (needed-task-lock (action BRING_S0) (place (nth$ 1 ?a)) (resource (sym-cat BRING_S0 (nth$ 1 ?a)))))
    )
    (case load-with-S1 then
      (assert (needed-task-lock (action BRING_S1) (place (nth$ 2 ?a)) (resource (sym-cat BRING_S1 (nth$ 2 ?a)))))
    )
    (case pick-and-deliver then
      (assert (needed-task-lock (action PICK_PROD) (place (nth$ 1 ?a)) (resource (sym-cat PICK_PROD (nth$ 1 ?a))))
      )
    )
    (case pick-and-load then
      ;get puck type we move
      (do-for-fact ((?machine machine)) (eq ?machine:name (nth$ 1 ?a))
	(bind ?puck ?machine:produced-puck)
      )
      (assert (needed-task-lock (action PICK_PROD) (place (nth$ 1 ?a)) (resource (sym-cat PICK_PROD (nth$ 1 ?a))))
	      (needed-task-lock (action (sym-cat BRING_ ?puck)) (place (nth$ 2 ?a)) (resource (sym-cat (sym-cat BRING_ ?puck) (nth$ 2 ?a))))
      )
    )
    (default (printout warn "task-locks for " ?task " not implemented yet" crlf))
  )
)

(defrule coordination-ask-for-lock
  (needed-task-lock (resource ?res))
  =>
  (assert (lock (type GET) (agent ?*ROBOT-NAME*) (resource ?res)))
)

(defrule coordination-accept-proposed-task
  (forall (needed-task-lock (resource ?res))
	  (lock (type ACCEPT) (agent ?rn&:(eq ?rn ?*ROBOT-NAME*)) (resource ?res))
  )
  ;there is only one asked task at a time
  ?pt <- (proposed-task (name ?task) (args $?args) (state asked))
  ?s <- (state TASK-PROPOSED-ASKED)
  =>
  ;order taks
  (assert (task (name ?task) (args ?args)))
  (retract ?s)
  (assert (state TASK-ORDERED))
  ;update worldmodel
  (do-for-all-facts ((?ntl needed-task-lock)) TRUE
    (printout warn "assert wmc " ?ntl:place " " ADD_INCOMING " " ?ntl:action crlf)
    (assert (worldmodel-change (machine ?ntl:place) (change ADD_INCOMING) (value ?ntl:action)))
  )
  ;remove proposal
  (retract  ?pt)
  ;remove prevoiusly rejected proposals
  (do-for-all-facts ((?prp proposed-task)) (eq ?prp:state rejected)
    (retract ?prp)
  )
)

(defrule coordination-reject-proposed-task
  (needed-task-lock (resource ?res))
  (lock (type REFUSE) (agent ?rn&:(eq ?rn ?*ROBOT-NAME*)) (resource ?res))
  ?pt <- (proposed-task (state asked))
  ?s <- (state TASK-PROPOSED-ASKED)
  =>
  (retract ?s)
  (assert (state IDLE));look for next task
  (modify ?pt (state rejected))
  ;cleanup asks for locks
  (do-for-all-facts ((?ntl needed-task-lock) (?l lock)) (and (eq ?l:agent ?*ROBOT-NAME*)(eq ?l:resource ?ntl:resource))
    (retract ?ntl ?l)
    (assert (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?l:resource)))
  )  
)

(defrule coordination-release-after-task-finished
  ?t <- (task (name ?task) (args $?args) (state finished)) 
  ?s <- (state TASK-FINISHED)
  =>
  ;release all locks for subtask goals
  (do-for-all-facts ((?ntl needed-task-lock)) TRUE
    (assert (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?ntl:resource)))
    (assert (worldmodel-change (machine ?ntl:place) (change REMOVE_INCOMING) (value ?ntl:action)))
    (retract ?ntl)
  )
  (retract ?s ?t)
  (assert (state IDLE))
)
