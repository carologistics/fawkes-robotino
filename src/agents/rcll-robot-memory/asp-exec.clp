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
  (bind ?o (bson-create))
  (bson-append ?o "robot" ?name)
  (bson-append ?query "o" ?o)
  (printout t "Query: " (bson-tostring ?query) crlf)
  (bind ?trigger (robmem-trigger-register "syncedrobmem.plan" ?query "robmem-plan"))
  (bson-destroy ?query)
)

(defrule asp-plan-update
  "We have an update for our plan."
  ?update <- (robmem-trigger (name "robmem-plan") (ptr ?obj))
  =>
  (printout t "Plan update " (bson-tostring ?obj) crlf)
  (retract ?update)
  (bson-destroy ?obj)
)
