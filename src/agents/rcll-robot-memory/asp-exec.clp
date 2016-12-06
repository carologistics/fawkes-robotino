(deffacts asp-exec-helpers
  "Facts we don't need directly, but the used old rules need them."
  (lock-role ASP-Exec)
)

(deffacts asp-exec-init
  (not-registered-rm)
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
