(deffacts asp-exec-helpers
  "Facts we don't need directly, but the used old rules need them."
  ;(lock-role ASP-Exec)
)

(deftemplate planElement
  (slot index (type INTEGER))
  (slot task (type STRING))
  (slot begin (type INTEGER))
  (slot end (type INTEGER))
  (slot done (allowed-symbols TRUE FALSE) (default FALSE))
  (slot _id)
)

(deftemplate asp-doing
  (slot index (type INTEGER))
  (slot task (type STRING))
  (multislot params)
  (slot begin (type INTEGER))
  (slot end (type INTEGER))
  (slot moveStep (type INTEGER) (default 0))
  (slot sentAfterMoveUpdate (allowed-symbols TRUE FALSE) (default FALSE))
)

(deffacts asp-exec-init
  (not-registered-rm)
  (never-asp-done)
)

(defglobal
  ?*ASP-EXPLORATION-TIME*     = 0
  ?*ASP-READ-MPS-LIGHT-TIME*  = 0
  ?*ASP-TASK-BEGIN-TOLERANCE* = 0
  ?*ASP-UPDATE-THRESHOLD*     = 0
  ?*ASP-DELIVER-TIME*         = 0
  ?*ASP-FEED-RS-TIME*         = 0
  ?*ASP-GET-BASE-TIME*        = 0
  ?*ASP-GET-PRODUCT-TIME*     = 0
  ?*ASP-MOUNT-CAP-TIME*       = 0
  ?*ASP-MOUNT-RING-TIME*      = 0
  ?*ASP-PREPARE-CS-TIME*      = 0
)

(defrule asp-bind-deliver-time
  (confval (path "/asp-agent/time-estimations/deliver-product") (type UINT) (value ?time))
  =>
  (bind ?*ASP-DELIVER-TIME*    ?time)
  (bind ?*ASP-FEED-RS-TIME*    ?time)
  (bind ?*ASP-MOUNT-CAP-TIME*  ?time)
  (bind ?*ASP-MOUNT-RING-TIME* ?time)
)

(defrule asp-bind-get-time
  (confval (path "/asp-agent/time-estimations/fetch-product") (type UINT) (value ?time))
  =>
  (bind ?*ASP-GET-BASE-TIME*    ?time)
  (bind ?*ASP-GET-PRODUCT-TIME* ?time)
)

(defrule asp-bind-prep-cs-time
  (confval (path "/asp-agent/time-estimations/prepare-cs") (type UINT) (value ?time))
  =>
  (bind ?*ASP-PREPARE-CS-TIME* ?time)
)

(defrule asp-bind-task-begin-tolerance
  (confval (path "/asp-agent/planer/time-resolution") (type UINT) (value ?res))
  (confval (path "/asp-agent/exec/task-begin-tolerance") (type UINT) (value ?tol))
  =>
  (bind ?*ASP-TASK-BEGIN-TOLERANCE* (+ ?tol (round (/ ?res 2) )))
)

(defrule asp-bind-exploration-time
  (declare (salience 10000))
  (confval (path "/asp-agent/exploration-time") (type UINT) (value ?time))
  (phase PRODUCTION)
  =>
  (bind ?*ASP-EXPLORATION-TIME* ?time)
)

(defrule asp-bind-read-mps-light
  (confval (path "/asp-agent/time-estimations/read-mps-light") (type UINT) (value ?time))
  =>
  (bind ?*ASP-READ-MPS-LIGHT-TIME* ?time)
)

(defrule asp-bind-update-threshold
  (confval (path "/asp-agent/exec/update-threshold") (type UINT) (value ?thres))
  =>
  (bind ?*ASP-UPDATE-THRESHOLD* ?thres)
)

(deffunction asp-create-feedback-bson (?action ?task)
  "Creates a BSON object for a feedback and inserts the common information."
  (bind ?doc (bson-create))
  (bson-append ?doc "action" ?action)
  (bson-append ?doc "robot" (sym-cat ?*ROBOT-NAME*))
  (bson-append ?doc "task" (sym-cat ?task))
  (return ?doc)
)

(deffunction asp-send-feedback (?doc)
  "Inserts the document in RM and destroys the object."
  (robmem-insert "syncedrobmem.planFeedback" ?doc)
  (bson-destroy ?doc)
)

(deffunction asp-game-time (?gt)
  "Adds the exploration time, when we are in the production phase."
  ;*ASP-EXPLORATION-TIME* is set to 0 before the production phase and to the correct value in the phase, so no if.
  (return (+ ?gt ?*ASP-EXPLORATION-TIME*))
)

(defrule asp-register-plan-update
  "Registers for plan updates in the rm."
  ?flag <- (not-registered-rm)
  (confval (path "/asp-agent/robot-name") (type STRING) (value ?name))
  =>
  (retract ?flag)
  (bind ?query (bson-create))
  (robmem-trigger-register "syncedrobmem.plan" ?query "robmem-plan-retract-fact")
  (bson-append ?query "robot" (sym-cat ?name))
  (robmem-trigger-register "syncedrobmem.plan" ?query "robmem-plan-filtered")
  (robmem-trigger-register "syncedrobmem.stopPlan" ?query "robmem-plan-stop")
  (bson-destroy ?query)
)

(defrule asp-plan-retract-prepare
  "Asserts a helper to retract a planElement."
  ?trigger <- (robmem-trigger (name "robmem-plan-retract-fact") (ptr ?obj))
  =>
  (if (eq (bson-get ?obj "op") "d") then
    (bind ?id (bson-get (bson-get ?obj "o") "_id"))
    (assert (retractPlanElement (sym-cat ?id)))
  )
  ;(printout t "Object: " (bson-tostring ?obj) crlf)
  (bson-destroy ?obj)
  (retract ?trigger)
)

(defrule asp-plan-retract-planElement
  "Retracts a planElement which isn't in the RM anymore."
  ?helper <- (retractPlanElement ?id)
  ?element <- (planElement (_id ?id))
  =>
  (retract ?helper ?element)
)

(defrule asp-plan-retract-helper-cleanup
  "Removes helpers, this are the delete messages for planElements of other robots."
  (declare (salience ?*PRIORITY-CLEANUP*))
  ?helper <- (retractPlanElement ?)
  =>
  (retract ?helper)
)

(defrule asp-plan-stop-task
  "We have been signaled to stop immediately with our task."
  (declare (salience ?*PRIORITY-HIGH*))
  ?trigger <- (robmem-trigger (name "robmem-plan-stop") (ptr ?obj))
  =>
  (skill-call-stop)
  (assert (asp-go-into-idle))
  (bson-destroy ?obj)
  (retract ?trigger)
)

(defrule asp-remove-task-after-stop
  (declare (salience ?*PRIORITY-HIGH*))
  (asp-go-into-idle)
  ?task <- (task)
  =>
  (retract ?task)
)

(defrule asp-remove-steps-after-stop
  (declare (salience ?*PRIORITY-HIGH*))
  (asp-go-into-idle)
  ?step <- (step)
  =>
  (retract ?step)
)

(defrule asp-release-locks-after-stop
  (declare (salience ?*PRIORITY-HIGH*))
  (asp-go-into-idle)
  ?lock <- (lock (type ACCEPT|GET|REFUSE) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?res))
  =>
  (retract ?lock)
  (assert (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?res)))
)

(defrule asp-remove-wait-for-lock-after-stop
  (declare (salience ?*PRIORITY-HIGH*))
  (asp-go-into-idle)
  ?wait <- (wait-for-lock)
  =>
  (retract ?wait)
)

(defrule asp-assert-idle
  (declare (salience ?*PRIORITY-HIGH*))
  (asp-go-into-idle)
  ?state <- (state ~IDLE)
  =>
  (retract ?state)
  (assert (state IDLE))
)

(defrule asp-remove-asp-doing-after-stop
  (declare (salience ?*PRIORITY-HIGH*))
  (asp-go-into-idle)
  ?doing <- (asp-doing)
  =>
  (retract ?doing)
)

(defrule asp-remove-planElements-after-stop
  (declare (salience ?*PRIORITY-HIGH*))
  (asp-go-into-idle)
  ?p <- (planElement (done FALSE))
  =>
  (retract ?p)
)

(defrule asp-cleanup-stop
  ?idle <- (asp-go-into-idle)
  =>
  (retract ?idle)
)

(defrule asp-plan-update
  "We have an update for our plan."
  (declare (salience ?*PRIORITY-HIGH*))
  ?update <- (robmem-trigger (name "robmem-plan-filtered") (ptr ?obj))
  =>
  (bind ?o (bson-get ?obj "o"))
  (bind ?op (bson-get ?obj "op"))
  (switch ?op
    (case "i" then
      ;A new element will be just added.
      (rm-assert-from-bson ?o)
      (bson-destroy ?obj)
    )
    (case "u" then
      ;A update of an existing element will be handled diffentrly for already started elements.
      (assert (updatePlanElement (sym-cat (bson-get ?o "_id")) ?obj))
      ;(do-for-fact ((?pE planElement)) (eq ?pE:_id (sym-cat (bson-get ?o "_id"))) (retract ?pE))
      ;(rm-assert-from-bson ?o)
    )
  )
  (retract ?update)
)

(defrule asp-plan-update-running-task
  "We should update a running task."
  (declare (salience ?*PRIORITY-HIGH*))
  ?update <- (updatePlanElement ?id ?obj)
  (planElement (_id ?id) (index ?idx) (task ?task))
  (asp-doing (index ?idx))
  =>
  (if (neq ?task (bson-get (bson-get ?obj "o") "task")) then
    ;The task was changed, this can happen due to a race condition. Our feedback didn't arrive at the planner
    ;before he changed the plan. It will happen and the planner will accept our running task, so drop this update.
    (bson-destroy ?obj)
    (retract ?update)
  )
)

(defrule asp-plan-update-merge
  "Update a not running task or only the time for it, this happens based on our feedback."
  (declare (salience (- ?*PRIORITY-HIGH* 1)))
  ?update <- (updatePlanElement ?id ?obj)
  ?pE <- (planElement (_id ?id))
  =>
  (retract ?update ?pE)
  (rm-assert-from-bson (bson-get ?obj "o"))
  (bson-destroy ?obj)
)

(deffunction asp-get-side (?side)
  (if (eq ?side I) then (return INPUT) else (return OUTPUT))
)

(deffunction asp-remove-quote (?string)
  (bind ?pos (str-index "'" ?string))
  (while (neq ?pos FALSE) do
    (bind ?string (str-cat (sub-string 1 (- ?pos 1) ?string) (sub-string (+ ?pos 1) (str-length ?string) ?string)))
    (bind ?pos (str-index "'" ?string))
  )
  (return ?string)
)

;(deffunction asp-start-exploration (?zone)
;  (assert (state EXP_LOCK_ACCEPTED))
;  (assert (exp-next-machine (sym-cat Z ?zone)))
;)

(deffunction asp-start-task (?string ?index)
  "Looks at the string and asserts the facts to actually start the task."
  (bind ?paramsBegin (str-index "(" ?string))
  (bind ?task (sub-string 1 (- ?paramsBegin 1) ?string))
  (bind ?paramsString (sub-string (+ ?paramsBegin 1) (- (str-length ?string) 1) ?string))
  (bind ?params (explode$ (asp-remove-quote ?paramsString)))
  (return (create$ ?task ?params))
)

(defrule asp-choose-next-task
  "Choose the next task, if we aren't doing anything else."
  (not (asp-doing))
  (not (asp-go-into-idle))
  (game-time ?gt ?)
  (planElement (done FALSE) (index ?idx) (task ?task) (begin ?begin&:(<= (- ?begin ?*ASP-TASK-BEGIN-TOLERANCE*) (asp-game-time ?gt))) (end ?end))
  (not (planElement (done FALSE) (index ?otherIdx&:(< ?otherIdx ?idx))))
  (state IDLE)
  (time $?time)
  =>
  (printout t "Chose Task #" ?idx ": " ?task " (" ?begin ", " ?end ")" " " ?time crlf)
  (bind ?pair (asp-start-task ?task ?idx))
  (bind ?taskName (nth$ 1 ?pair))
  (bind ?params (delete$ ?pair 1 1))
  (bind ?end (+ (- ?gt ?begin) ?end))
  (assert (asp-doing (index ?idx) (task ?taskName) (params ?params) (begin ?gt) (end ?end)))
  (bind ?gt (asp-game-time ?gt))
  (bind ?end (asp-game-time ?end))
  (bind ?doc (asp-create-feedback-bson begin ?task))
  (bson-append ?doc "begin" ?gt)
  (bson-append ?doc "end" ?end)
  (asp-send-feedback ?doc)
)

(defrule asp-done-anything
  "Removes the flag that the robot has never done anything."
  ?done <- (never-asp-done)
  (asp-doing)
  =>
  (retract ?done)
)

(defrule asp-move-from-into-field
  "Moves the robot away from the into field position, because it blocks the following robots."
  (declare (salience ?*PRIORITY-LOW*))
  ?done <- (never-asp-done)
  (state IDLE)
  =>
  (retract ?done)
  (bind ?wait (create$))
  (do-for-all-facts ((?wp zone-waitpoint)) TRUE
    (bind ?wait (insert$ ?wait 1 ?wp:name))
  )
  (bind ?wait (nth$ (random 1 (length$ ?wait)) ?wait))
  (skill-call ppgoto place (str-cat ?wait))
)

(defrule asp-no-task-rule
  "Matches if we have not defined a rule for a given task."
  (declare (salience ?*PRIORITY-LOW*))
  (asp-doing (task ?taskName))
  (state IDLE)
  =>
  (printout warn "Unknown task " ?taskName " cannot start!" crlf)
)

(defrule asp-start-deliver
  ?state <- (state IDLE)
  ;params should look like this: m ( C DS I ) 2 1
  ?doing <- (asp-doing (index ?index) (task "deliver"|"lateDeliver") (params ? ? ?team ?machine ?side ? ?order ?))
  (order (id ?order) (delivery-gate ?gate))
  =>
  (bind ?machineName (sym-cat ?team - ?machine))
  (bind ?machineSide (asp-get-side ?side))
  (bind ?taskID (* ?index 1000))
  (retract ?state)
  (assert (task (id ?taskID) (name deliver) (state ordered) (steps (create$ (+ ?taskID 1) (+ ?taskID 2))))
          (step (id (+ ?taskID 1)) (name drive-to) (machine ?machineName) (side ?machineSide))
          (step (id (+ ?taskID 2)) (name insert) (machine ?machineName) (side ?machineSide) (gate ?gate) (machine-feature CONVEYOR))
          (state TASK-ORDERED)
  )
  (modify ?doing (moveStep (+ ?taskID 1)))
)

(defrule asp-start-feed-rs
  ?state <- (state IDLE)
  ;params should look like this: m ( C RS1 I )
  ?doing <- (asp-doing (index ?index) (task "feedRS") (params ? ? ?team ?machine ?side ?))
  =>
  (bind ?machineName (sym-cat ?team - ?machine))
  (bind ?machineSide (asp-get-side ?side))
  (bind ?taskID (* ?index 1000))
  (retract ?state)
  (assert (task (id ?taskID) (name fill-rs) (state ordered) (steps (create$ (+ ?taskID 1) (+ ?taskID 2))))
          (step (id (+ ?taskID 1)) (name drive-to) (machine ?machineName) (side ?machineSide))
          (step (id (+ ?taskID 2)) (name insert) (machine ?machineName) (side ?machineSide) (machine-feature SLIDE))
          (state TASK-ORDERED)
  )
  (modify ?doing (moveStep (+ ?taskID 1)))
)

(defrule asp-start-get-base
  ?state <- (state IDLE)
  ;params should look like this: m ( C BS I ) "RED"
  ?doing <- (asp-doing (index ?index) (task "getBase") (params ? ? ?team ?machine ?side ? ?baseColor))
  =>
  (bind ?baseColor (sym-cat ?baseColor))
  (bind ?machineName (sym-cat ?team - ?machine))
  (bind ?machineSide (asp-get-side ?side))
  (bind ?taskID (* ?index 1000))
  (retract ?state)
  (assert (task (id ?taskID) (name clear-bs) (state ordered) (steps (create$ (+ ?taskID 1) (+ ?taskID 2) (+ ?taskID 3) (+ ?taskID 4))))
          (step (id (+ ?taskID 1)) (name drive-to) (machine ?machineName) (side ?machineSide))
          (step (id (+ ?taskID 2)) (name acquire-lock) (lock PREPARE-BS))
          (step (id (+ ?taskID 3)) (name instruct-mps) (machine ?machineName) (side ?machineSide) (base ?baseColor))
          (step (id (+ ?taskID 4)) (name get-base) (machine ?machineName) (side ?machineSide) (base ?baseColor) (machine-feature CONVEYOR))
          (state TASK-ORDERED)
  )
  (modify ?doing (moveStep (+ ?taskID 2)))
)

(defrule asp-start-get-product
  ?state <- (state IDLE)
  ;params should look like this: m ( C RS1 O )
  ?doing <- (asp-doing (index ?index) (task "getProduct") (params ? ? ?team ?machine ?side ?))
  =>
  (bind ?machineName (sym-cat ?team - ?machine))
  (bind ?machineSide (asp-get-side ?side))
  (bind ?taskID (* ?index 1000))
  (retract ?state)
  (assert (task (id ?taskID) (name clear-bs) (state ordered) (steps (create$ (+ ?taskID 1) (+ ?taskID 2))))
          (step (id (+ ?taskID 1)) (name drive-to) (machine ?machineName) (side ?machineSide))
          (step (id (+ ?taskID 2)) (name get-output) (machine ?machineName) (side ?machineSide) (machine-feature CONVEYOR))
          (state TASK-ORDERED)
  )
  (modify ?doing (moveStep (+ ?taskID 1)))
)

(defrule asp-start-goto
  ?state <- (state IDLE)
  ;params should look like this: m ( C RS1 O )
  ?doing <- (asp-doing (index ?index) (task "goto") (params ? ? ?team ?machine ?side ?))
  =>
  (bind ?machineName (sym-cat ?team - ?machine))
  (bind ?machineSide (asp-get-side ?side))
  (bind ?taskID (* ?index 1000))
  (retract ?state)
  (assert (task (id ?taskID) (name clear-bs) (state ordered) (steps (create$ (+ ?taskID 1))))
          (step (id (+ ?taskID 1)) (name drive-to) (machine ?machineName) (side ?machineSide))
          (state TASK-ORDERED)
  )
)

(defrule asp-start-mount-cap
  ?state <- (state IDLE)
  ;params should look like this: m ( C CS1 I ) 2 1
  ?doing <- (asp-doing (index ?index) (task "mountCap") (params ? ? ?team ?machine ?side ? ?order ?))
  =>
  (bind ?machineName (sym-cat ?team - ?machine))
  (bind ?machineSide (asp-get-side ?side))
  (bind ?taskID (* ?index 1000))
  (retract ?state)
  (assert (task (id ?taskID) (name produce-cx) (state ordered) (steps (create$ (+ ?taskID 1) (+ ?taskID 2))))
          (step (id (+ ?taskID 1)) (name drive-to) (machine ?machineName) (side ?machineSide))
          (step (id (+ ?taskID 2)) (name insert) (machine ?machineName) (side ?machineSide) (machine-feature CONVEYOR))
          (state TASK-ORDERED)
  )
  (modify ?doing (moveStep (+ ?taskID 1)))
)

(defrule asp-start-mount-ring
  ?state <- (state IDLE)
  ;params should look like this: m ( C CS1 I ) 2 1 3
  ?doing <- (asp-doing (index ?index) (task "mountRing") (params ? ? ?team ?machine ?side ? ?order ? ?ring))
  (order (id ?order) (product-id ?prod))
  (product (id ?prod) (rings $?rings))
  =>
  (bind ?machineName (sym-cat ?team - ?machine))
  (bind ?machineSide (asp-get-side ?side))
  (bind ?taskID (* ?index 1000))
  (bind ?ringColor (nth$ ?ring ?rings))
  (printout t "MountRing Order: " ?order " Ring: " ?ring " Product-ID: " ?prod " Rings: " ?rings " RingColor: " ?ringColor crlf)
  (retract ?state)
  (assert (task (id ?taskID) (name add-additional-ring) (state ordered) (steps (create$ (+ ?taskID 1) (+ ?taskID 2))))
          (step (id (+ ?taskID 1)) (name drive-to) (machine ?machineName) (side ?machineSide))
          (step (id (+ ?taskID 2)) (name insert) (machine ?machineName) (side ?machineSide) (machine-feature CONVEYOR) (ring ?ringColor))
          (state TASK-ORDERED)
  )
  (modify ?doing (moveStep (+ ?taskID 1)))
)

(defrule asp-start-prep-cs
  ?state <- (state IDLE)
  ;params should look like this: m ( C CS1 I )
  ?doing <- (asp-doing (index ?index) (task "prepareCS") (params ? ? ?team ?machine ?side ?))
  =>
  (bind ?machineName (sym-cat ?team - ?machine))
  (bind ?machineSide (asp-get-side ?side))
  (bind ?taskID (* ?index 1000))
  (retract ?state)
  (assert (task (id ?taskID) (name fill-cap) (state ordered) (steps (create$ (+ ?taskID 1) (+ ?taskID 2) (+ ?taskID 3))))
          (step (id (+ ?taskID 1)) (name drive-to) (machine ?machineName) (side ?machineSide))
          (step (id (+ ?taskID 2)) (name get-from-shelf) (machine ?machineName) (side ?machineSide) (machine-feature SHELF))
          (step (id (+ ?taskID 3)) (name insert) (machine ?machineName) (side ?machineSide) (machine-feature CONVEYOR) (already-at-mps TRUE))
          (state TASK-ORDERED)
  )
  (modify ?doing (moveStep (+ ?taskID 1)))
)

(defrule asp-move-step-done
  "Update our time estimation, when we arrived at the machine."
  (declare (salience ?*PRIORITY-HIGH*))
  ?doing <- (asp-doing (index ?idx) (task ?taskName) (moveStep ?stepId) (end ?end) (sentAfterMoveUpdate FALSE))
  (planElement (index ?idx) (task ?task))
  (step (id ?stepId) (state finished))
  (game-time ?gt ?)
  (time $?time)
  =>
  (printout t "Begin really: " ?task " " ?time crlf)
  (bind ?soll (switch ?taskName
    (case "deliver"     then ?*ASP-DELIVER-TIME*)
    (case "feedRS"      then ?*ASP-FEED-RS-TIME*)
    (case "getBase"     then ?*ASP-GET-BASE-TIME*)
    (case "getProduct"  then ?*ASP-GET-PRODUCT-TIME*)
    (case "goto"        then 0)
    (case "lateDeliver" then ?*ASP-DELIVER-TIME*)
    (case "mountCap"    then ?*ASP-MOUNT-CAP-TIME*)
    (case "mountRing"   then ?*ASP-MOUNT-RING-TIME*)
    (case "prepareCS"   then ?*ASP-PREPARE-CS-TIME*)
    (default 30)
  ))
  (bind ?ist (- ?end ?gt))
  (bind ?offset (- ?soll ?ist))
  (bind ?end (asp-game-time (+ ?end ?offset)))
  (bind ?doc (asp-create-feedback-bson update ?task))
  (bson-append ?doc "end" ?end)
  (asp-send-feedback ?doc)
  (modify ?doing (sentAfterMoveUpdate TRUE))
)

(defrule asp-task-success
  "Inform the planer about success."
  ?doing <- (asp-doing (index ?idx))
  ?state <- (state TASK-FINISHED)
  ?pE <- (planElement (index ?idx) (task ?task))
  (game-time ?gt ?)
  (time $?time)
  =>
  (printout t "Task #" ?idx " successfully executed." " " ?time crlf)
  (bind ?gt (asp-game-time ?gt))
  (bind ?doc (asp-create-feedback-bson end ?task))
  (bson-append ?doc "end" ?gt)
  (bson-append ?doc "success" TRUE)
  (asp-send-feedback ?doc)
  (modify ?pE (done TRUE))
  (retract ?state ?doing)
  (assert (state IDLE))
)

(defrule asp-task-failure
  "Inform the planer about failure."
  ?doing <- (asp-doing (index ?idx))
  ?state <- (state TASK-FAILED)
  ?pE <- (planElement (index ?idx) (task ?task))
  (game-time ?gt ?)
  =>
  (printout t "Task #" ?idx " was a failure." crlf)
  (bind ?gt (asp-game-time ?gt))
  (bind ?doc (asp-create-feedback-bson end ?task))
  (bson-append ?doc "end" ?gt)
  (bson-append ?doc "success" FALSE)
  (asp-send-feedback ?doc)
  (modify ?pE (done TRUE))
  (retract ?state ?doing)
  (assert (state DELETE-PLAN))
)

(defrule asp-delete-plan
  "Delete all not done plan elements."
  (state DELETE-PLAN)
  ?pE <- (planElement (done FALSE))
  =>
  (retract ?pE)
)

(defrule asp-delete-plan-to-idle
  "Plan is deleted, change state to idle."
  ?s <- (state DELETE-PLAN)
  (not (planElement (done FALSE)))
  =>
  (retract ?s)
  (assert (state IDLE))
)

;(defrule asp-update-time-estimation-exp
;  "Updates the time estimation for the explore task, if we have to look for the mps light."
;  (game-time ?gt ?t)
;  ?doing <- (asp-doing (index ?idx) (end ?end&:(>= (abs (- ?end (+ (asp-game-time ?gt) ?*ASP-READ-MPS-LIGHT-TIME*))) ?*ASP-UPDATE-THRESHOLD*)))
;  (planElement (index ?idx) (task ?task))
;  (state EXP_WAIT_BEFORE_DRIVE_TO_OUTPUT)
;  =>
;  (bind ?end (+ ?gt ?*ASP-READ-MPS-LIGHT-TIME*))
;  (modify ?doing (end ?end))
;  (bind ?gt (asp-game-time ?gt))
;  (bind ?end (asp-game-time ?end))
;  (bind ?doc (asp-create-feedback-bson update ?task))
;  (bson-append ?doc "end" ?end)
;  (bson-append ?doc "time" ?gt)
;  (asp-send-feedback ?doc)
;)

;(defrule asp-found-machine
;  "Sends the information about the found machine."
;  (declare (salience ?*PRIORITY-HIGH*))
;  (last-zoneinfo (search-state YES))
;  (exp-current-zone (name ?machine))
;  =>
;  (bind ?doc (asp-create-feedback-bson foundMachine ""))
;  (bson-append ?doc "machine" ?machine)
;  (asp-send-feedback ?doc)
;)

;(defrule asp-end-exploration
;  "Send an end message for the explore skill."
;  (game-time ?gt ?)
;  ?doing <- (asp-doing (index ?idx))
;  ?state <- (state EXP_IDLE)
;  ?planElement <- (planElement (index ?idx) (task ?task))
;  =>
;  (bind ?gt (asp-game-time ?gt))
;  (retract ?doing ?state)
;  (modify ?planElement (done TRUE))
;  (bind ?doc (asp-create-feedback-bson end ?task))
;  (bson-append ?doc "end" ?gt)
;  ;Todo: We have to check if it was really a success!
;  (bson-append ?doc "success" TRUE)
;  (asp-send-feedback ?doc)
;)
