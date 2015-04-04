;---------------------------------------------------------------------------
;  coordination.clp - coordination of task allocation
;                     Ask locking master to accept or reject a chosen task
;
;  Created: Thu Mar 06 20:35:52 2014
;  Copyright  2014 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; Approach:
; 1. production.clp proposes a task and determines which subtasks have to be
;    coordinated with the other robots
;    (e.g. task: bring produced S2 from M1 to T4 M2 => noone else is on the way
;    to pick the produced puck from M1 and noone is on the way to bring a S2 to M2)
; 2. ask for resource locking of each sub-task to coordinate
;    (e.g. lock resource called PICK-PROD_M1 and S2_M2)
; 3a. one sub-task was rejected => reject proposed task and go on in production.clp
; 3b. all sub-tasks were accepted => order execution of the task and go on in tasks.clp
; 4b. release sub-tasks-lock after execution (this also triggers changing the world state)

(defrule coordination-compute-recource-string
  "The needed-task-lock fact has a action and location but for the locking
   we need to use a single string, we add here to the needed-task-lock"
  ?ntl <- (needed-task-lock (resource NONE) (action ?action) (place ?place))
  =>
  (modify ?ntl (resource (sym-cat ?action "~" ?place)))
)

(defrule coordination-all-recource-strings-computed
  "When all resource strings are computed we want to aquire the locks"
  ?s <- (state TASK-PROPOSED)
  (not (needed-task-lock (resource NONE)))
  ?pt <- (task (name ?task) (state proposed))
  =>
  (retract ?s)
  (assert (state TASK-PROPOSED-ASKED))
  (modify ?pt (state asked))
)

; (defrule coordination-compute-resources-to-lock
;   "Decides which resources need locking for the proposed task and asserts a needed-task-lock for them. Asserts state TASK-PROPOSED-ASKED."
;   ?pt <- (proposed-task (name ?task) (args $?a) (state proposed))
;   (not (proposed-task (state asked)))
;   ?s <- (state TASK-PROPOSED)
;   =>
;   (modify ?pt (state asked))
;   (retract ?s)
;   (assert (state TASK-PROPOSED-ASKED))
;   (switch ?task
;     (case load-with-S0 then
;       (assert (needed-task-lock (action BRING_S0) (place (nth$ 1 ?a))
; 				(resource (sym-cat BRING_S0 "~" (nth$ 1 ?a)))))
;     )
;     (case load-with-S1 then
;       (assert (needed-task-lock (action BRING_S1) (place (nth$ 1 ?a)) 
; 				(resource (sym-cat BRING_S1 "~" (nth$ 1 ?a)))))
;     )
;     (case load-with-S2 then
;       (assert (needed-task-lock (action BRING_S2) (place (nth$ 1 ?a)) 
; 				(resource (sym-cat BRING_S2 "~" (nth$ 1 ?a)))))
;     )
;     (case pick-and-deliver then
;       (assert (needed-task-lock (action PICK_PROD) (place (nth$ 1 ?a)) 
; 				(resource (sym-cat PICK_PROD "~" (nth$ 1 ?a))))
; 	      (needed-task-lock (action (sym-cat BRING_ (nth$ 2 ?a))) (place DELIVER) 
; 				(resource (sym-cat BRING_ (nth$ 2 ?a) "~" (nth$ 3 ?a) "~Order" (nth$ 4 ?a))))
;       )
;     )
;     (case recycle then
;       (assert (needed-task-lock (action PICK_CO) (place (nth$ 1 ?a)) 
; 				(resource (sym-cat PICK_CO "-" (nth$ 1 ?a))))
;       )
;     )
;     (case pick-and-load then
;       ;get puck type we move
;       (do-for-fact ((?machine machine)) (eq ?machine:name (nth$ 1 ?a))
; 	(bind ?puck ?machine:produced-puck)
;       )
;       (assert (needed-task-lock (action PICK_PROD) (place (nth$ 1 ?a))
; 				(resource (sym-cat PICK_PROD "~" (nth$ 1 ?a))))
; 	      (needed-task-lock (action (sym-cat BRING_ ?puck)) (place (nth$ 2 ?a))
; 				(resource (sym-cat (sym-cat BRING_ ?puck) "~" (nth$ 2 ?a))))
;       )
;     )
;     (case deliver then
;       (assert (needed-task-lock (action (sym-cat BRING_ (nth$ 2 ?a))) (place DELIVER) 
; 				(resource (sym-cat BRING_ (nth$ 2 ?a) "~" (nth$ 3 ?a) "~Order" (nth$ 4 ?a)))))
;     )
;     (case recycle-holding then
;       ;nothing has to be locked here because we want to get rid of an unintentionally holding puck
;     )
;     (case just-in-time-P3 then
;       (assert (needed-task-lock (action BRING_S0) (place (nth$ 1 ?a))
; 				(resource (sym-cat BRING_S0 "~" (nth$ 1 ?a))))
; 	      (needed-task-lock (action PICK_PROD) (place (nth$ 1 ?a))
; 				(resource (sym-cat PICK_PROD "~" (nth$ 1 ?a)))))
;     )
;     (case pick-and-store then
;       (assert (needed-task-lock (action PICK_PROD) (place (nth$ 1 ?a))
; 				(resource (sym-cat PICK_PROD "~" (nth$ 1 ?a))))
; 	      (needed-task-lock (action STORE_PUCK) (place (nth$ 3 ?a))
; 				(resource (sym-cat STORE_PUCK "~" (nth$ 3 ?a)))))
;     )
;     (case store then
;       (assert (needed-task-lock (action STORE_PUCK) (place (nth$ 1 ?a))
; 				(resource (sym-cat STORE_PUCK "~" (nth$ 1 ?a)))))
;     )
;     (case get-stored-and-deliver then
;       (assert (needed-task-lock (action PICK_PROD) (place (nth$ 1 ?a))
; 				(resource (sym-cat PICK_PROD "~" (nth$ 1 ?a))))
; 	      (needed-task-lock (action (sym-cat BRING_ (nth$ 2 ?a))) (place DELIVER)
; 					(resource (sym-cat BRING_ (nth$ 2 ?a) "~" (nth$ 3 ?a) "~Order" (nth$ 4 ?a)))))
;     )
;     (case produce-with-S0 then
;       (assert (needed-task-lock (action BRING_S0) (place (nth$ 1 ?a))
; 				(resource (sym-cat BRING_S0 "~" (nth$ 1 ?a))))
; 	      (needed-task-lock (action PICK_PROD) (place (nth$ 1 ?a))
; 				(resource (sym-cat PICK_PROD "~" (nth$ 1 ?a)))))
;     )
;     (default (printout warn "task-locks for " ?task " not implemented yet" crlf))
;   )
; )

(defrule coordination-ask-for-lock
  "Asks for a lock (type GET) that was computed in coordination-compute-resources-to-lock. Lock acceptance is handled in lock-managing.clp."
  (needed-task-lock (resource ?res&~NONE))
  =>
  (assert (lock (type GET) (agent ?*ROBOT-NAME*) (resource ?res)))
  ; Retract all lock releases for ?res that gets the lock
  (do-for-all-facts ((?release lock)) (and (eq ?release:agent ?*ROBOT-NAME*)
                                           (eq ?release:resource ?res)
                                           (eq ?release:type RELEASE))
    (retract ?release)
  )
)

(defrule coordination-accept-proposed-task
  "Processes accepted lock. Proposed-task and corresponding state are retracted. Asserts the accepted task and sends incoming/delivery facts for machines/orders to the wordmodel."
  (forall (needed-task-lock (resource ?res))
	  (lock (type ACCEPT) (agent ?rn&:(eq ?rn ?*ROBOT-NAME*)) (resource ?res))
  )
  ;there is only one asked task at a time
  ?pt <- (task (name ?task) (state asked) (priority ?p))
  ?s <- (state TASK-PROPOSED-ASKED)
  =>
  ;order taks
  (modify ?pt (state ordered))
  (retract ?s)
  (assert (state TASK-ORDERED))
  ;update worldmodel
  (do-for-all-facts ((?ntl needed-task-lock)) TRUE
    ;(printout warn "assert wmc " ?ntl:place " " ADD_INCOMING " " ?ntl:action crlf)
    (if (eq ?ntl:place DELIVER)
      then
      (printout t "delivery not in worldmodel-changes atm" crlf)
      ; (assert (worldmodel-change (order (nth$ 4 ?args)) (change SET_IN_DELIVERY)
      ; 				 (value (nth$ 2 ?args))
      ; 				 (amount (nth$ 3 ?args))))
      else
      (assert (worldmodel-change (machine ?ntl:place) (change ADD_INCOMING)
				 (value ?ntl:action)))
    )
  )
  
)

(defrule coordination-reject-proposed-task
  "Processes refused lock. Changes proposed-task to rejected and robotino state back to IDLE. Releases the refused lock."
  (needed-task-lock (resource ?res))
  (lock (type REFUSE) (agent ?rn&:(eq ?rn ?*ROBOT-NAME*)) (resource ?res))
  ?pt <- (task (state asked))
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

(deffunction coordination-remove-old-rejected-tasks ()
  "Function to remove all previously rejected tasks and their steps"
  (bind $?step-ids-to-remove (create$))
  ;remove prevoiusly rejected proposals
  (do-for-all-facts ((?prp task)) (eq ?prp:state rejected)
    (retract ?prp)
    ;remember ids to remove the steps of this task
    (bind ?step-ids-to-remove (insert$ ?step-ids-to-remove 0 ?prp:steps))
  )
  ;remove steps
  (do-for-all-facts ((?step task)) (member$ ?step:id ?step-ids-to-remove)
    (retract ?step)
  )
)

(defrule coordination-release-after-task-finished
  "If a task is finished the lock for the task is released and incoming facts are removed from the worldmodel. State is changed from TASK-FINISHED to IDLE."
  (declare (salience ?*PRIORITY-LOCK-HIGH*))
  ?t <- (task (name ?task) (state finished)) 
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
  (coordination-remove-old-rejected-tasks)
)

(defrule coordination-release-after-task-aborted
  "If a task is finished, the task state is set to finished, although the TASK-FINISHED state in the robot is missing. Release the task and remove incoming facts in worldmodel. State is changed in production.clp."
  (declare (salience ?*PRIORITY-LOCK-LOW*))
  ?t <- (task (name ?task) (state finished))
  =>
  ;release all locks for subtask goals
  (do-for-all-facts ((?ntl needed-task-lock)) TRUE
    (assert (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?ntl:resource)))
    (assert (worldmodel-change (machine ?ntl:place) (change REMOVE_INCOMING) (value ?ntl:action)))
    (retract ?ntl)
  )
  (retract ?t)
)

(defrule coordination-release-and-reject-task-after-failed
  "If a task has failed the task lock is released and incoming facts are removed. If needed a warning is printed and the state is changed from TASK-FAILED to IDLE. All rejected proposals are removed and failed task is rejected."
  ?t <- (task (name ?task) (state failed)) 
  ?s <- (state TASK-FAILED)
  =>
  ;release all locks for subtask goals
  (delayed-do-for-all-facts ((?ntl needed-task-lock) (?m machine)) (eq ?m:name ?ntl:place)
    (bind ?fails (+ ?m:fails 1))
    (if (and (eq ?m:mtype T1) (>= ?fails ?*FAILS-TO-BLOCK*))
     then
      (printout warn "Machine " ?m:name " failed " ?fails " times, blocking" crlf)
      (modify ?m (fails ?fails))
      (assert (worldmodel-change (machine ?m:name) (change SET_PRODUCE_BLOCKED)))
    )
    (assert (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?ntl:resource)))
    (assert (worldmodel-change (machine ?ntl:place) (change REMOVE_INCOMING) (value ?ntl:action)))
    (retract ?ntl)
  )
  (retract ?s )
  (assert (state IDLE))
  (coordination-remove-old-rejected-tasks)
  ;reject task because it failed
  (modify ?t (state rejected))
)