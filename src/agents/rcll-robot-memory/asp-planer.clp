(defrule active-robot-to-robmem
  "Updates the active-robot fact in the robot memory."
  ?ar <- (active-robot (name ?name) (last-seen ?ls-m ?ls-s))
  (not (asp-synced ?name ?ls-m ?ls-s))
  =>
  (assert (asp-synced ?name ?ls-m ?ls-s))
  (bind ?doc (rm-structured-fact-to-bson ?ar))
  (bind ?query-for-sync-id (str-cat "{\"relation\": \"active-robot\", \"name\": \"" ?name "\"}"))
  (robmem-upsert "robmem.planer" ?doc ?query-for-sync-id)
  (bson-destroy ?doc)
)

(deffacts team-color-to-robmem-init
  (asp-synced tc nil)
)

(defrule team-color-to-robmem
  "Updates the team-color fact in the robot memory."
  ?new <- (team-color ?tc)
  ?old <- (asp-synced tc ?oldTC&~?tc)
  =>
  (retract ?old)
  (assert (asp-synced tc ?tc))
  (bind ?doc (rm-ordered-fact-to-bson ?new))
  (robmem-upsert "robmem.planer" ?doc  "{\"relation\": \"team-color\"}")
  (bson-destroy ?doc)
)

