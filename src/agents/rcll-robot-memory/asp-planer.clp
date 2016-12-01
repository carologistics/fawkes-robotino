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

(deffacts to-robmem-init
  (asp-synced tc nil)
  (asp-synced gt 0 PRE_GAME)
  (asp-to-sync zones)
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

(defrule game-time-to-robmem
  "Updates the game-time-phase-tuple in the robot memory."
  ?gt <- (game-time ?t ?)
  (phase ?p)
  (not (asp-synced gt ?t ?p))
  =>
  (assert (asp-synced gt ?t ?p))
  (bind ?doc (bson-create))
  (bson-append ?doc "relation" game-time)
  (bson-append ?doc "time" ?t)
  (bson-append ?doc "phase" ?p)
  (robmem-upsert "robmem.planer" ?doc "{\"relation\": \"game-time\"}")
  (bson-destroy ?doc)
)

(defrule accumulate-zones-to-explore
  "Accumulates the zones we need to explore, before we send them through the robot memory."
  (have-exp-info)
  (team-color ?team)
  (zone-exploration (team ?team) (name ?z))
  (not (asp-synced zone ?z))
  ?toSync <- (asp-to-sync zones $?zones)
  =>
  (assert (asp-synced zone ?z))
  (retract ?toSync)
  (assert (asp-to-sync zones $?zones ?z))
)

(defrule zones-to-explore-to-robmem
  "Sets the zones we need to explore in the robot memory."
  (asp-to-sync zones $?zones&:(eq (length $?zones) 12))
  (not (asp-synced zones))
  =>
  (assert (asp-synced zones))
  (bind ?doc (bson-create))
  (bson-append ?doc "relation" zones)
  (bson-append-array ?doc "zones" $?zones)
  (robmem-insert "robmem.planer" ?doc)
  (bson-destroy ?doc)
)

