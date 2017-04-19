
;---------------------------------------------------------------------------
;  exploration.clp - Robotino agent for exploration phase
;
;  Created: Fri Apr 26 18:38:18 2013 (Magdeburg)
;  Copyright  2013  Frederik Zwilling
;             2013  Alexander von Wirth 
;             2013  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

;Read exploration rows from config
(defrule exp-cfg-get-row
  "Read configuration for exploration row order of machines"
  (declare (salience ?*PRIORITY-WM*))
  (phase EXPLORATION)
  (robot-name ?robot-name)
  (team-color ?team)
  (confval
    (path ?path&:(eq ?path (str-cat "/clips-agent/rcll2016/exploration/route/" ?team "/" ?robot-name)))
    (list-value $?route)
  )
=>
  (assert (exp-route ?route)
  )
  (printout t "Exploration route: " ?route crlf)
)

(defrule exp-start
  "Set up the state. There are two rounds. In the first the robotino drives to each machine in a defined cycle. After the first round the robotino drives to unrecognized machines again."
  (phase EXPLORATION)
  ?st <- (exploration-start)
  (team-color ?team-color)
  (NavigatorInterface (id "Navigator") (max_velocity ?max-velocity) (max_rotation ?max-rotation))
=>
  (retract ?st)
  (assert (state EXP_IDLE)
          (timer (name send-machine-reports))
          (navigator-default-vmax (velocity ?max-velocity) (rotation ?max-rotation))
  )
  (if (eq ?team-color nil) then
    (printout error "Ouch, starting exploration but I don't know my team color" crlf)
  )
  (printout t "Yippi ka yeah. I am in the exploration-phase." crlf)
)


(defrule exp-set-next-node
  (phase EXPLORATION)
  (exp-route $?route)
  (state EXP_IDLE)
=>
  (do-for-all-facts ((?nn exp-next-node)) TRUE (retract ?nn))
  (if (<= ?*EXP-ROUTE-IDX* (length$ ?route)) then
    (assert (exp-next-node (node (nth$ ?*EXP-ROUTE-IDX* ?route))))
  )
)


(defrule exp-goto-next
  (phase EXPLORATION)
  ?s <- (state ?state&:(or (eq ?state EXP_START) (eq ?state EXP_IDLE)))
  (exp-next-node (node ?next-node))
  (navgraph-node (name ?next-node))
  (exp-navigator-vmax ?max-velocity ?max-rotation)
  (or
    (NavGraphWithMPSGeneratorInterface (final TRUE))
    (NavGraphWithMPSGeneratorInterface (msgid 0))
    (not (NavGraphWithMPSGeneratorInterface))
  )
=>
  (retract ?s)
  (assert
    (state EXP_GOTO_NEXT)
  )
  (navigator-set-speed ?max-velocity ?max-rotation)
  (skill-call drive_to_local place ?next-node)
)


(defrule exp-node-blocked
  (phase EXPLORATION)
  ?s <- (state EXP_GOTO_NEXT)
  (exp-next-node (node ?next-node))
  (navgraph-node (name ?next-node) (pos $?node-trans))
  (zone-exploration (name ?zn&:(eq ?zn (get-zone ?node-trans))) (machine ?machine&~UNKNOWN&~NONE))
  (Position3DInterface (id "Pose") (translation $?pose-trans))
  (test (< 1.5 (distance
    (nth$ 1 ?node-trans)
    (nth$ 2 ?node-trans)
    (nth$ 1 ?pose-trans)
    (nth$ 2 ?pose-trans)
  )))
=>
  (printout t "Node " ?next-node " blocked by " ?machine
    " but we got close enough. Proceeding to next node." crlf)
  (retract ?s)
  (skill-call relgoto x 0 y 0)
  (bind ?*EXP-ROUTE-IDX* (+ 1 ?*EXP-ROUTE-IDX*))
  (assert (state EXP_IDLE))
)


(defrule exp-goto-next-final
  (phase EXPLORATION)
  ?s <- (state EXP_GOTO_NEXT)
  ?skill-f <- (skill-done (name "drive_to_local") (status ?))
  (navigator-default-vmax (velocity ?max-velocity) (rotation ?max-rotation))
=>
  (retract ?s ?skill-f)
  (navigator-set-speed ?max-velocity ?max-rotation)
  (bind ?*EXP-ROUTE-IDX* (+ 1 ?*EXP-ROUTE-IDX*))
  (assert
    (exp-searching)
    (state EXP_IDLE)
  )
)


(defrule exp-passed-through-quadrant
  "We're driving slowly through a certain quadrant: reason enough to believe there's no machine here."
  (phase EXPLORATION)
  (exp-navigator-vmax ?max-velocity ?max-rotation)
  (MotorInterface (id "Robotino")
    (vx ?vx&:(< ?vx ?max-velocity)) (vy ?vy&:(< ?vy ?max-velocity)) (omega ?w&:(< ?w ?max-rotation))
  )
  (Position3DInterface (id "Pose") (translation $?trans) (time $?timestamp) (visibility_history ?vh&:(>= ?vh 10)))
  ?ze <- (zone-exploration
    (name ?zn&:(eq ?zn (get-zone 0.15 ?trans)))
    (machine UNKNOWN)
    (times-searched ?times-searched)
  )
=>
  (bind ?zone (get-zone 0.07 ?trans))
  (if ?zone then
    (synced-modify ?ze machine NONE times-searched (+ 1 ?times-searched))
  )
)


(defrule exp-found-line
  "Found a line that is within an unexplored zone."
  (phase EXPLORATION)
  (exp-searching)
  (LaserLineInterface (id ?id&~:(str-index "moving_avg" ?id))
    (visibility_history ?vh&:(>= ?vh 1))
    (time $?timestamp)
  )
  (MotorInterface (id "Robotino") (vx ?vx) (vy ?vy))
  (exp-zone-margin ?zone-margin)
  ?ze-f <- (zone-exploration
    (name ?zn&:(eq ?zn (get-zone ?zone-margin
      (compensate-movement
        ?*EXP-MOVEMENT-COMPENSATION*
        (create$ ?vx ?vy)
        (laser-line-get-center ?id ?timestamp)
        ?timestamp
      )
    )))
    (machine UNKNOWN)
    (line-visibility ?zn-vh&:(> ?vh ?zn-vh))
  )
=>
  (printout t "EXP found line: " ?zn " vh: " ?vh crlf)
  (modify ?ze-f (line-visibility ?vh))
)


;(defrule exp-log-line
;  (phase EXPLORATION)
;  (exp-searching)
;  (LaserLineInterface (id ?id&~:(str-index "moving_avg" ?id))
;    (visibility_history ?vh&:(>= ?vh 1))
;    (time $?timestamp)
;  )
;  (exp-zone-margin ?zone-margin)
;  (test (get-zone ?zone-margin (laser-line-get-center ?id ?timestamp)))
;=>
;  (bind ?zone (get-zone ?zone-margin (laser-line-get-center ?id ?timestamp)))
;  (do-for-all-facts ((?ze zone-exploration)) (eq ?ze:name ?zone)
;    (bind ?m ?ze:machine)
;    (bind ?lv ?ze:line-visibility)
;  )
;  (printout t "EXP log line: " ?zone " " ?m " " ?lv crlf)
;)


(defrule exp-stop-to-investigate-zone
  (phase EXPLORATION)
  (exp-searching)
  ?st-f <- (state EXP_GOTO_NEXT)
  (zone-exploration
    (name ?zn)
    (machine UNKNOWN)
    (line-visibility ?vh&:(> ?vh 0))
  )
  ; Use the zone with the highest line-visibility
  (not (zone-exploration (machine UNKNOWN) (line-visibility ?vh2&:(> ?vh2 ?vh))))
  (not (locked-resource (resource ?r&:(eq ?r ?zn))))
=>
  (delayed-do-for-all-facts ((?exp-f explore-zone-target)) TRUE (retract ?exp-f))
  (retract ?st-f)
  (assert
    (state EXP_STOPPING)
    (explore-zone-target (zone ?zn))
  )
  (skill-call relgoto x 0 y 0)
)


(defrule exp-stopped
  (phase EXPLORATION)
  ?srch-f <- (exp-searching)
  ?st-f <- (state EXP_STOPPING)
  (explore-zone-target (zone ?zn))
  (MotorInterface (id "Robotino")
    (vx ?vx&:(< ?vx 0.01)) (vy ?vy&:(< ?vy 0.01)) (omega ?w&:(< ?w 0.01))
  )
=>
  (retract ?st-f ?srch-f)
  (assert (state EXP_LOCK_REQUIRED)
    (lock (type GET) (agent ?*ROBOT-NAME*) (resource ?zn))
  )
)


(defrule exp-found-tag
  (phase EXPLORATION)
  ?srch-f <- (exp-searching)
  (tag-matching (tag-id ?tag) (machine ?machine) (side ?side))
  (not (found-tag (name ?machine)))
  (TagVisionInterface (id "/tag-vision/info")
    (tags_visible ?num-tags&:(> ?num-tags 0))
    (tag_id $?tag-ids&:(member$ ?tag ?tag-ids))
  )
  (Position3DInterface
    (id ?tag-if-id&:(eq ?tag-if-id (str-cat "/tag-vision/" (- (member$ ?tag ?tag-ids) 1))))
    (visibility_history ?vh&:(> ?vh 1))
    (translation $?trans) (rotation $?rot)
    (frame ?frame) (time $?timestamp)
  )
  (exp-zone-margin ?zone-margin)
  (zone-exploration
    (name ?zn&:(eq ?zn (get-zone ?zone-margin (transform-safe "map" ?frame ?timestamp ?trans ?rot))))
    (machine UNKNOWN)
  )
  (not (locked-resource (resource ?r&:(eq ?r ?zn))))
  ?st-f <- (state ?)
=>
  (skill-call relgoto x 0 y 0)
  (retract ?srch-f ?st-f)
  (assert (state EXP_LOCK_REQUIRED)
    (lock (type GET) (agent ?*ROBOT-NAME*) (resource ?zn))
    (explore-zone-target (zone ?zn))
  )
)


(defrule exp-skill-explore-zone
  (phase EXPLORATION)
  ?st-f <- (state EXP_LOCK_ACCEPTED)
  ?exp-f <- (explore-zone-target (zone ?zn))
  (navigator-default-vmax (velocity ?trans-vmax) (rotation ?rot-vmax))
=>
  (retract ?exp-f ?st-f)
  (assert (state EXP_EXPLORE_ZONE))
  (navigator-set-speed ?trans-vmax ?rot-vmax)
  (skill-call explore_zone zone (str-cat ?zn))
)


(defrule exp-skill-explore-zone-final
  (phase EXPLORATION)
  ?st-f <- (state EXP_EXPLORE_ZONE)
  ?skill-f <- (skill-done (name "explore_zone") (status FINAL))
  (ZoneInterface (id "/explore-zone/info") (zone ?zn-str)
    (orientation ?orientation) (tag_id ?tag-id) (search_state YES)
  )
  (Position3DInterface (id "/explore-zone/found-tag")
    (frame ?frame) (translation $?trans) (rotation $?rot)
  )
  ; We don't check the visibility_history here since that's already done in the explore_zone skill
  (tag-matching (tag-id ?tag-id) (machine ?machine) (side ?side) (team ?team-color))
  ?ze <- (zone-exploration (name ?zn2&:(eq ?zn2 (sym-cat ?zn-str))) (times-searched ?times-searched))
=>
  (retract ?st-f ?skill-f)
  (assert
    (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource (sym-cat ?zn-str)))
  )
  (if (any-factp ((?ft found-tag)) (eq ?ft:name ?machine)) then
    (printout error "BUG: Tag for " ?machine " already found. Locking glitch or agent bug!" crlf)
  else
    ; Order is important here: Update state before zone-exploration to avoid endless loop.
    (synced-modify ?ze machine ?machine times-searched (+ 1 ?times-searched))
    (synced-assert (str-cat "(found-tag (name " ?machine ") (side " ?side ")"
      "(frame \"map\") (trans " (implode$ ?trans) ") "
      "(rot " (implode$ ?rot) ") )")
    )
  )
  (assert
    (exp-searching)
    (state EXP_IDLE)
  )
)


(defrule exp-skill-explore-zone-failed
  (phase EXPLORATION)
  ?st-f <- (state EXP_EXPLORE_ZONE)
  ?skill-f <- (skill-done (name "explore_zone") (status ?status))
  (ZoneInterface (search_state ~YES))
=>
  (retract ?st-f ?skill-f)
  (if (eq ?status FINAL) then
    (printout error "BUG in explore_zone skill: Result is FINAL but no MPS was found.")
  )
  (assert
    (exp-searching)
    (state EXP_IDLE)
  )
)


(defrule exp-mirror-tag
  (found-tag (name ?machine) (side ?side) (frame ?frame) (trans $?trans) (rot $?rot))
  ; Assuming that ?frame is always "map". Otherwise things will break rather horribly...
  (tag-matching (tag-id ?tag) (machine ?machine) (side ?side))
  (zone-exploration (name ?zn) (machine ?machine) (times-searched ?times-searched))

  (tag-matching (tag-id ?tag2) (side ?side)
    (machine ?machine2&:(eq ?machine2 (mirror-name ?machine)))
  )
  (not (found-tag (name ?machine2&:(eq ?machine2 (mirror-name ?machine)))))
  ?ze2 <- (zone-exploration (name ?zn2&:(eq ?zn2 (mirror-name ?zn))))
=>
  (assert
    (found-tag (name ?machine2) (side ?side) (frame ?frame)
      (trans (mirror-trans ?trans)) (rot (mirror-rot ?rot))
    )
  )
  (modify ?ze2 (machine ?machine2) (times-searched ?times-searched))
)


(defrule exp-report-found-tag
  (or
    (NavGraphWithMPSGeneratorInterface (final TRUE))
    (NavGraphWithMPSGeneratorInterface (msgid 0))
    (not (NavGraphWithMPSGeneratorInterface))
  )
  (team-color ?team-color)

  (found-tag (name ?machine)
    (side ?side) (frame ?) (trans $?) (rot $?)
  )
  (tag-matching (tag-id ?tag) (team ?team-color)
    (machine ?machine) (side ?side)
  )
  (found-tag (name ?machine2&:(eq ?machine2 (mirror-name ?machine)))
    (side ?side) (frame ?) (trans $?) (rot $?)
  )
  (tag-matching (tag-id ?tag2) (team ?team-color2&~?team-color)
    (machine ?machine2) (side ?side)
  )
  (not (and (navgraph-added-for-mps (name ?machine)) (navgraph-added-for-mps (name ?machine2))))
=>
  (navgraph-add-all-new-tags)
)


(defrule exp-check-resource-locking
  "Handle a lock that was accepted by the master"
  (phase EXPLORATION)
  ?s <- (state EXP_LOCK_REQUIRED)
  ?l <- (lock (type ACCEPT) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?r))
  =>
  (printout t "Lock for " ?r " accepted." crlf)
  (retract ?s ?l)
  (assert (state EXP_LOCK_ACCEPTED))
)


