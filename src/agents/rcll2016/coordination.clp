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
  ?pt <- (task (name ?task) (state proposed) (robot ?r&:(eq ?r ?*ROBOT-NAME*)))
  =>
  (retract ?s)
  (assert (state TASK-PROPOSED-ASKED))
  (synced-modify ?pt state asked)
)

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

(deffunction coordination-get-fact-address-of-place (?place)
  (bind ?coordinated-places (create$ zone-exploration machine puck-storage))
  (progn$ (?templ ?coordinated-places)
    (do-for-fact ((?fact ?templ)) (eq ?place ?fact:name)
      (return ?fact)
    )
  )
  (printout error "Could not find place: " ?place crlf)
)

(defrule coordination-accept-proposed-task
  "Processes accepted lock. Proposed-task and corresponding state are retracted. Asserts the accepted task and sends incoming/delivery facts for machines/orders to the wordmodel."
  (forall (needed-task-lock (resource ?res))
	  (lock (type ACCEPT) (agent ?rn&:(eq ?rn ?*ROBOT-NAME*)) (resource ?res))
  )
  ;there is only one asked task at a time
  ?pt <- (task (name ?task) (state asked) (priority ?p) (robot ?r&:(eq ?r ?*ROBOT-NAME*)))
  ?s <- (state TASK-PROPOSED-ASKED)
  =>
  ;order taks
  (synced-modify ?pt state ordered)
  (retract ?s)
  (assert (state TASK-ORDERED))
  ;update worldmodel
  (delayed-do-for-all-facts ((?ntl needed-task-lock)) TRUE
    (bind ?fact-ptr (coordination-get-fact-address-of-place ?ntl:place))
    (bind ?fact-ptr (synced-add-to-multifield ?fact-ptr incoming ?ntl:action))
    (synced-add-to-multifield ?fact-ptr incoming-agent (sym-cat ?*ROBOT-NAME*))
  )
)

(defrule coordination-reject-proposed-task
  "Processes refused lock. Changes proposed-task to rejected and robotino state back to IDLE. Releases the refused lock."
  (needed-task-lock (resource ?res))
  (lock (type REFUSE) (agent ?rn&:(eq ?rn ?*ROBOT-NAME*)) (resource ?res))
  ?pt <- (task (state asked)  (robot ?r&:(eq ?r ?*ROBOT-NAME*)))
  ?s <- (state TASK-PROPOSED-ASKED)
  =>
  (retract ?s)
  (assert (state IDLE));look for next task
  (synced-modify ?pt state rejected)
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
    (bind ?step-ids-to-remove (insert$ ?step-ids-to-remove 1 ?prp:steps))
  )
  ;remove steps
  (do-for-all-facts ((?step task)) (member$ ?step:id ?step-ids-to-remove)
    (retract ?step)
  )
)

(deffunction coordination-release-all-subgoal-locks ()
  ;release all locks for subtask goals
  (delayed-do-for-all-facts ((?ntl needed-task-lock)) TRUE
    (assert (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?ntl:resource)))
    (bind ?fact-ptr (coordination-get-fact-address-of-place ?ntl:place))
    (bind ?fact-ptr (synced-remove-from-multifield ?fact-ptr incoming ?ntl:action))
    (synced-remove-from-multifield ?fact-ptr incoming-agent (sym-cat ?*ROBOT-NAME*))
    (retract ?ntl)
  )
)

(defrule coordination-release-after-task-finished
  "If a task is finished the lock for the task is released and incoming facts are removed from the worldmodel. State is changed from TASK-FINISHED to IDLE."
  (declare (salience ?*PRIORITY-LOCK-HIGH*))
  ?t <- (task (name ?task) (state finished) (steps $?steps)  (robot ?r&:(eq ?r ?*ROBOT-NAME*))) 
  ?s <- (state TASK-FINISHED)
  =>
  (coordination-release-all-subgoal-locks)
  ;remove all steps of the task
  (do-for-all-facts ((?step step)) (member$ ?step:id ?steps)
    (synced-retract ?step)
  )
  (synced-retract ?t)
  (retract ?s)
  (assert (state IDLE))
  (coordination-remove-old-rejected-tasks)
)

(defrule coordination-release-after-task-aborted
  "If a task is finished, the task state is set to finished, although the TASK-FINISHED state in the robot is missing. Release the task and remove incoming facts in worldmodel. State is changed in production.clp."
  (declare (salience ?*PRIORITY-LOCK-LOW*))
  ?t <- (task (name ?task) (state finished)  (robot ?r&:(eq ?r ?*ROBOT-NAME*)))
  =>
  (coordination-release-all-subgoal-locks)
  (synced-retract ?t)
)

(defrule coordination-release-and-reject-task-after-failed
  "If a task has failed the task lock is released and incoming facts are removed. If needed a warning is printed and the state is changed from TASK-FAILED to IDLE. All rejected proposals are removed and failed task is rejected."
  ?t <- (task (name ?task) (state failed)  (robot ?r&:(eq ?r ?*ROBOT-NAME*))) 
  ?s <- (state TASK-FAILED)
  =>
  (coordination-release-all-subgoal-locks)
  (retract ?s )
  (assert (state IDLE))
  (coordination-remove-old-rejected-tasks)
  ;reject task because it failed
  (synced-modify ?t state rejected)
)
