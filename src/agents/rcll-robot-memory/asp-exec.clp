(deffacts asp-exec-helpers
  "Facts we don't need directly, but the used old rules need them."
  (lock-role ASP-Exec)
)

(deftemplate planElement
  (slot index (type INTEGER))
  (slot task (type STRING))
  (slot begin (type INTEGER))
  (slot end (type INTEGER))
  (slot done (allowed-symbols TRUE FALSE) (default FALSE))
  (slot _id)
)

(deffacts asp-exec-init
  (not-registered-rm)
)

(defglobal
  ?*ASP-EXPLORATION-TIME*     = 0
  ?*ASP-READ-MPS-LIGHT-TIME*  = 0
  ?*ASP-TASK-BEGIN-TOLERANCE* = 0
  ?*ASP-UPDATE-THRESHOLD*     = 0
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
  (printout t "Bound exp time " ?time crlf)
)

(defrule asp-bind-read-mps-light
  (confval (path "/asp-agent/time-estimations/read-mps-light") (type UINT) (value ?time))
  =>
  (bind ?*ASP-READ-MPS-LIGHT-TIME* ?time)
  (printout t "MP Light time " ?time crlf)
)

(defrule asp-bind-update-threshold
  (confval (path "/asp-agent/exec/update-threshold") (type UINT) (value ?thres))
  =>
  (bind ?*ASP-UPDATE-THRESHOLD* ?thres)
  (printout t "Bound Thres " ?thres crlf)
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
  (bind ?trigger (robmem-trigger-register "syncedrobmem.plan" ?query "robmem-plan-filtered"))
  (bson-destroy ?query)
)

;Doesn't work :/
(defrule asp-plan-retract-fact
  "Retracts the plan element fact which entry was deleted in the robmem."
  (robmem-trigger (name "robmem-plan-retract-fact") (ptr ?obj))
  (test (eq (bson-get ?obj "op") "d"))
  ?element <- (planElement (_id ?id))
  (test (eq (bson-get (bson-get ?obj "o") "_id") ?id))
  =>
  (printout t "Delete " ?id crlf)
  (retract ?element)
)

(defrule asp-plan-retract-prepare
  "Asserts a helper to retract a planElement."
  ?trigger <- (robmem-trigger (name "robmem-plan-retract-fact") (ptr ?obj))
  =>
  (if (eq (bson-get ?obj "op") "d") then
    (bind ?id (bson-get (bson-get ?obj "o") "_id"))
    (assert (retractPlanElement (sym-cat ?id)))
  )
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

;Not needed as long asp-plan-retract-fact doesn't work
(defrule asp-plan-retract-fact-cleanup
  "Retracts the trigger fact and destroys the bson object."
  (declare (salience ?*PRIORITY-CLEANUP*))
  ?trigger <- (robmem-trigger (name "robmem-plan-retract-fact") (ptr ?obj))
  =>
  (printout t "Retract: " (bson-tostring ?obj) crlf)
  (printout t "ID: " (bson-tostring (bson-get (bson-get ?obj "o") "_id")) crlf)
  (bson-destroy ?obj)
  (retract ?trigger)
)

(defrule asp-plan-update
  "We have an update for our plan."
  ?update <- (robmem-trigger (name "robmem-plan-filtered") (ptr ?obj))
  =>
  (bind ?o (bson-get ?obj "o"))
  (bind ?op (bson-get ?obj "op"))
  (switch ?op
    (case "i" then
      (rm-assert-from-bson ?o)
    )
    (case "u" then
      (printout t "Plan update: " (bson-tostring ?o))
      (rm-assert-from-bson ?o)
    )
  )
  (retract ?update)
  (bson-destroy ?obj)
)

(deffunction asp-start-exploration (?zone)
  (assert (state EXP_LOCK_ACCEPTED))
  (assert (exp-next-machine (sym-cat Z ?zone)))
)

(deffunction asp-start-task (?string)
  "Looks at the string and asserts the facts to actually start the task."
  (bind ?paramsBegin (str-index "(" ?string))
  (bind ?task (sub-string 1 (- ?paramsBegin 1) ?string))
  (bind ?paramsString (sub-string (+ ?paramsBegin 1) (- (str-length ?string) 1) ?string))
  (bind ?params (explode$ ?paramsString))
  (switch ?task
    (case "explore" then (asp-start-exploration (nth$ 1 ?params)))
    (default (printout warn "Unknown task " ?task " cannot start!" crlf))
  )
  (return (create$ ?task ?params))
)

(defrule asp-choose-next-task
  "Choose the next task, if we aren't doing anything else."
  (not (asp-doing $?))
  (game-time ?gt ?)
  (planElement (done FALSE) (index ?idx) (task ?task) (begin ?begin&:(<= (- ?begin ?*ASP-TASK-BEGIN-TOLERANCE*) (asp-game-time ?gt))) (end ?end))
  (not (planElement (done FALSE) (index ?otherIdx&:(< ?otherIdx ?idx))))
  =>
  (printout t "Chose Task #" ?idx ": " ?task " (" ?begin ", " ?end ")" crlf)
  (bind ?gt (asp-game-time ?gt))
  (bind ?doc (asp-create-feedback-bson begin ?task))
  (bson-append ?doc "begin" ?gt)
  (asp-send-feedback ?doc)
  (bind ?pair (asp-start-task ?task))
  (bind ?task (nth$ 1 ?pair))
  (delete$ ?pair 1 1)
  (bind ?params ?pair)
  (assert (asp-doing ?idx ?task ?params ?gt (+ (- ?gt ?begin) ?end)))
)

(defrule asp-update-time-estimation
  "Update the time estimation if we are near the end."
  (game-time ?gt ?)
  ?doing <- (asp-doing ?idx ?task ?begin ?end&:(<= ?end (- (asp-game-time ?gt) 1)))
  =>
  (bind ?gt (asp-game-time ?gt))
  (retract ?doing)
  (bind ?end (+ ?end ?*ASP-UPDATE-THRESHOLD*))
  (assert (asp-doing ?idx ?task ?begin ?end))
  (bind ?doc (asp-create-feedback-bson update ?task))
  (bson-append ?doc "end" ?end)
  (asp-send-feedback ?doc)
)

(defrule asp-update-time-estimation-exp
  "Updates the time estimation for the explore task, if we have to look for the mps light."
  (game-time ?gt ?t)
  ?doing <- (asp-doing ?idx ?task ?begin ?end&:(>= (abs (- ?end (+ (asp-game-time ?gt) ?*ASP-READ-MPS-LIGHT-TIME*))) ?*ASP-UPDATE-THRESHOLD*))
  (state EXP_WAIT_BEFORE_DRIVE_TO_OUTPUT)
  =>
  (bind ?gt (asp-game-time ?gt))
  (retract ?doing)
  (bind ?end (+ ?gt ?*ASP-READ-MPS-LIGHT-TIME*))
  (bind ?doc (asp-create-feedback-bson update ?task))
  (bson-append ?doc "end" ?end)
  (asp-send-feedback ?doc)
)

(defrule asp-end-exploration
  "Send an end message for the explore skill."
  (game-time ?gt ?)
  ?doing <- (asp-doing ?idx ?task $?)
  ?state <- (state EXP_IDLE)
  ?planElement <- (planElement (index ?idx))
  =>
  (bind ?gt (asp-game-time ?gt))
  (retract ?doing ?state)
  (modify ?planElement (done TRUE))
  (bind ?doc (asp-create-feedback-bson end ?task))
  (bson-append ?doc "end" ?gt)
  (asp-send-feedback ?doc)
)

