(deffacts asp-planer-helper
  "Facts we need to use the old rules."
  ;(lock-role MASTER) ;So we get the zone informations.
  (simulation-is-running) ;To disable the motor enable calls, we have no motor!
)

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

(defrule ring-info-to-robmem
  "Sets the ring color cost and machine assignment in the robot memory."
  (ring (color ?color) (req-bases ?cost))
  (ring-station (name ?machine) (available-colors $?colors&:(neq (member$ ?color $?colors) FALSE)))
  (not (asp-synced ring ?color))
  =>
  (assert (asp-synced ring ?color))
  (bind ?doc (bson-create))
  (bson-append ?doc "relation" ring)
  (bson-append ?doc "color" ?color)
  (bson-append ?doc "cost" ?cost)
  (bson-append ?doc "machine" ?machine)
  (robmem-insert "robmem.planer" ?doc)
  (bson-destroy ?doc)
)

(defrule order-to-robmem
  "Sets the order info in the robot memory."
  (order (id ?number) (quantity-requested ?qty) (begin ?begin) (end ?end) (product-id ?prod))
  (product (id ?prod) (base ?base) (cap ?cap) (rings $?rings))
  (not (asp-synced order ?number))
  =>
  (assert (asp-synced order ?number))
  (bind ?doc (bson-create))
  (bson-append ?doc "relation" order)
  (bson-append ?doc "number" ?number)
  (bson-append ?doc "quantity" ?qty)
  (bson-append ?doc "begin" ?begin)
  (bson-append ?doc "end" ?end)
  (bson-append ?doc "base" ?base)
  (bson-append ?doc "cap" ?cap)
  (bson-append-array ?doc "rings" $?rings)
  (robmem-insert "robmem.planer" ?doc)
  (bson-destroy ?doc)
)

(defrule machine-to-robmem
  "Updates the machine information in the robot memory."
  (team-color ?team)
  (machine (team ?team) (name ?machine) (state ?state))
  =>
  (bind ?doc (bson-create))
  (bson-append ?doc "relation" machine)
  (bson-append ?doc "machine" ?machine)
  (bson-append ?doc "state" ?state)
  (robmem-upsert "robmem.planer" ?doc (str-cat "{\"relation\": \"machine\", \"machine\": \"" ?machine "\"}"))
  (bson-destroy ?doc)
)

