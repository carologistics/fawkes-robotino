(defrule active-robot-to-robmem
  "Updates the active-robot fact in the robot memory."
  ?ar <- (active-robot (name ?name) (last-seen ?ls-m ?ls-s))
  (not (synced ?name ?ls-m ?ls-s))
  =>
  (assert (synced ?name ?ls-m ?ls-s))
  (bind ?doc (rm-structured-fact-to-bson ?ar))
  (bind ?query-for-sync-id (str-cat "{\"relation\": \"active-robot\", \"name\": \"" ?name "\"}"))
  (robmem-upsert "robmem.planer" ?doc ?query-for-sync-id)
  (bson-destroy ?doc)
)

