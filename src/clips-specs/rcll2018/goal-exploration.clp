(defrule goal-exploration-create-exploration-maintain
" The parent exploration goal. Allows formulation of
  visit-node and explore-zone goals only if the proper game state selected
  and the domain got loaded.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (not (goal (class EXPLORATION-MAINTAIN)))
  (wm-fact (key refbox phase) (type UNKNOWN) (value EXPLORATION))
  (wm-fact (key game state) (type UNKNOWN) (value RUNNING))
  (wm-fact (key domain fact self args? r ?robot))
  (wm-fact (key domain fact entered-field args? r ?robot))
  =>
  (goal-tree-assert-run-one EXPLORATION-MAINTAIN)
)

(defrule goal-eploration-expand-exploration-goal
"  Populate the tree structure of the exploration tree. The priority of subgoals
   is determined by the order they are asserted. Sub-goals that are asserted
   earlier get a higher priority.
"
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  (goal (id ?goal-id) (class EXPLORATION-MAINTAIN) (mode SELECTED))
  (not (goal (parent ?goal-id)))
=>
  (goal-tree-assert-subtree ?goal-id
    (goal-tree-assert-run-one EXPLORATION-SELECTOR
      (goal-tree-assert-run-one INITIAL-POSITION)
      (goal-tree-assert-run-one ZONE-EXPLORATION)
      (goal-tree-assert-run-one VISIT-NODE)))
)

(defrule goal-exploration-create-move-initial-position-goal
  (goal (class INITIAL-POSITION) (id ?maintain-id) (mode SELECTED))
  (not (exploration-reached-initial-position))
  =>  
  (assert
    (goal (id (sym-cat MOVE-INITIAL-POSITION- (gensym*)))
          (class MOVE-INITIAL-POSITION)
          (sub-type SIMPLE)
          (parent ?maintain-id))
  )
)

(defrule goal-exploration-reject-initial-position-goal
  ?g <- (goal (class INITIAL-POSITION) (id ?maintain-id) (mode SELECTED))
  (exploration-reached-initial-position)
  =>
  (modify ?g (mode RETRACTED) (outcome REJECTED))
)

(defrule goal-exploration-create-explore-zone-goal
  (goal (class ZONE-EXPLORATION) (id ?maintain-id))
  
  (wm-fact (key domain fact self args? r ?r))

  ?ze <- (wm-fact (key exploration fact time-searched args? zone ?zn) (value ?ts&:(<= ?ts ?*EXP-SEARCH-LIMIT*)))
  (wm-fact (key exploration zone ?zn args? machine UNKNOWN team ?team))
  (wm-fact (key exploration fact line-vis args? zone ?zn) (value ?vh))
  (wm-fact (key exploration fact tag-vis args? zone ?zn) (value ?tv))
  (test (or (> ?vh 0) (> ?tv 0)))

  (not (exploration-result (zone ?zn)))

  (Position3DInterface (id "Pose") (translation $?trans))
  =>
  ; Priotize zones with tag and laser findings over zones with only one
  ; Priotize zones with tag findings over zones with only laser findings
  (bind ?prio (* ?vh 150))
  (bind ?prio (+ ?prio (* ?tv 100)))

  ; Increase priority by "inverse" of current distance to zone
  ; 17 is greater than the maximum distance between two points on the current size of the field
  (bind ?dist (distance-mf ?trans (zone-center ?zn)))
  (bind ?prio (+ ?prio (- 17 (integer ?dist))))

  (assert
    (goal (id (sym-cat EXPLORE-ZONE- (gensym*)))
          (class EXPLORE-ZONE)
          (sub-type SIMPLE)
          (priority ?prio)
          (parent ?maintain-id)
          (params
            robot ?r
            zone ?zn
          )
          (required-resources ?zn (mirror-name ?zn))
    )
  )
)

(defrule goal-exploration-create-visit-node-goal
  (goal (class VISIT-NODE) (id ?maintain-id) (mode SELECTED))

  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact self args? r ?r))

  (wm-fact (id ?id&: (eq ?id (str-cat "/config/rcll/route/" ?team-color "/" ?r))) (values $?route))
  =>
  (assert
    (goal (id (sym-cat MOVE-NODE- (gensym*)))
          (class MOVE-NODE)
          (sub-type SIMPLE)
          (priority 1)
          (parent ?maintain-id)
    )
  )
)

(defrule goal-exploration-abort-move-node
  ?g <- (goal (id ?goal-id) (class MOVE-NODE) (mode DISPATCHED))
  (goal (class EXPLORE-ZONE) (mode FORMULATED))
  ?pa <- (plan-action (goal-id ?goal-id) (state RUNNING))
  (exploration-reached-initial-position)
  =>
  (modify ?pa (state EXECUTION-FAILED))
)

(defrule goal-exploration-evaluate-initial-position
  ?g <- (goal (class MOVE-INITIAL-POSITION) (mode FINISHED) (outcome ?outcome))
  (not (exploration-reached-initial-position))
  =>
  (assert (exploration-reached-initial-position))
  (modify ?g (mode EVALUATED))
)

(defrule goal-exploration-evaluate-explore-zone-completed
  ?g <- (goal (class EXPLORE-ZONE) (mode FINISHED) (outcome COMPLETED))

  (ZoneInterface (id "/explore-zone/info") (zone ?zn-str)
    (orientation ?orientation) (tag_id ?tag-id) (search_state YES)
  )
  (domain-fact (name tag-matching) (param-values ?machine ?side ?team-color ?tag-id))
  (domain-fact (name mps-type) (param-values ?machine ?mtype))

  ?ze <- (wm-fact (key exploration fact time-searched args? zone ?zn2&:(eq ?zn2 (sym-cat ?zn-str))) (value ?times-searched))
  ?zm <- (wm-fact (key exploration zone ?zn2 args? machine ? team ?team))
  ?zm2 <- (wm-fact (key exploration zone ?zn3&:(eq (mirror-name ?zn2) ?zn3) args? machine ? team ?team2))

  (not (exploration-result (machine ?machine) (zone ?zn2)))
  =>
  (modify ?ze (value (+ 1 ?times-searched)))
  (modify ?zm (key exploration zone ?zn2 args? machine ?machine team ?team))
  (modify ?zm2 (key exploration zone ?zn3 args? machine (mirror-name ?machine) team ?team2))
  (assert
    (exploration-result
      (machine ?machine) (zone ?zn2)
      (orientation ?orientation)
      (team ?team-color)
    )
    (exploration-result
      (machine (mirror-name ?machine)) (zone (mirror-name ?zn2))
      (orientation (mirror-orientation ?mtype ?zn2 ?orientation))
      (team (mirror-team ?team-color))
    )
  )
  (modify ?g (mode EVALUATED))
  (printout t "EXP exploration fact zone successfull. Found " ?machine " in " ?zn2 crlf)
)

(defrule goal-exploration-evaluate-explore-zone-failed
  ?g <- (goal (class EXPLORE-ZONE) (mode FINISHED) (outcome FAILED))
  =>
  (modify ?g (mode EVALUATED))
)