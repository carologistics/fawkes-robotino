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

;common template for a proposed task
(deftemplate proposed-task
  (slot name (type SYMBOL) (allowed-values load-with-S0 load-with-S1 pick-and-load pick-and-deliver recycle-and-load-with-S0 recycle-and-load-with-T1))
  (multislot args (type SYMBOL)) ;in chronological order
  (slot state (type SYMBOL) (allowed-values proposed asked rejected) (default proposed))
)
(deftemplate needed-task-lock
  ;(slot action (type SYMBOL) (allowed-symbols S0 S1 S2 PICK-PROD PICK-CO))
  ;(slot place (type SYMBOL) (allowed-values M1 M2 M3 M4 M5 M6 M7 M8 M9 M10 R1 R2))
  (slot resource (type SYMBOL))
)


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
      (assert (needed-task-lock (resource (sym-cat S0 (nth$ 1 ?a)))))
    )
    (default (printout warn "task-locks for " ?task " not implemented yet" crlf))
  )
)

(defrule coordination-ask-for-lock
  (needed-task-lock (resource ?res))
  =>
  (assert (lock (type GET) (agent ?*ROBOT-NAME*) (resource ?res)))
)
x
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
  ;remove proposal
  (retract  ?pt)
  ;remove prevoiusly rejected proposals
  (do-for-all-facts ((?prp proposed-task)) (eq ?prp:state rejected)
    (retract ?prp)
  )
  ; following is done at releasing
  ; ;remove needed-task-locks and their accepts
  ; (do-for-all-instances ((?ntl needed-task-lock) (?l lock)) (and (= ?l:type ACCEPT) (= ?l:resource (sym-cat ?ntl:action ?ntl:place)))
  ;   (retract ?ntl ?l)
  ; )
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
  ?t <- (task (name ?task) (args ?args) (state finished)) 
  ?s <- (state TASK-FINISHED)
  =>
  ;release all locks for subtask goals
  (do-for-all-facts ((?ntl needed-task-lock)) TRUE
    (assert (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?ntl:resource)))
    (retract ?ntl)
  )
  (retract ?s ?t)
  (assert (state IDLE))
)

;TODO: use proposals
;TODO: test
