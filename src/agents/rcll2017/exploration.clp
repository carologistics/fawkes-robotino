
;---------------------------------------------------------------------------
;  exploration.clp - Robotino agent for exploration phase
;
;  Copyright  2017 Victor Mataré
;
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
  (phase EXPLORATION)
  ?st <- (exploration-start)
  (team-color ?team-color)
  (NavigatorInterface (id "Navigator") (max_velocity ?max-velocity) (max_rotation ?max-rotation))
=>
  (retract ?st)
  (assert (state EXP_IDLE)
          (timer (name send-machine-reports))
          (navigator-default-vmax (velocity ?max-velocity) (rotation ?max-rotation))
          (exp-repeated-search-limit 0)
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
  else
    ; Currently unused
    (assert (exp-do-clusters))
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
  (skill-call goto place ?next-node)
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


(defrule exp-goto-next-failed
  (phase EXPLORATION)
  ?s <- (state EXP_GOTO_NEXT)
  ?skill-f <- (skill-done (name "goto") (status FAILED))
  (exp-next-node (node ?node))
  (navgraph-node (name ?node) (pos $?node-trans))

  (Position3DInterface (id "Pose") (translation $?trans))
  (navigator-default-vmax (velocity ?max-velocity) (rotation ?max-rotation))
=>
  (if (< (distance-mf ?node-trans ?trans) 1.5) then
    (bind ?*EXP-ROUTE-IDX* (+ 1 ?*EXP-ROUTE-IDX*))
  )
  (retract ?s ?skill-f)
  (navigator-set-speed ?max-velocity ?max-rotation)
  (assert
    (exp-searching)
    (state EXP_IDLE)
  )
)


(defrule exp-goto-next-final
  (phase EXPLORATION)
  ?s <- (state EXP_GOTO_NEXT)
  ?skill-f <- (skill-done (name "goto") (status ?))
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
  (LaserLineInterface
    (visibility_history ?vh&:(>= ?vh 1))
    (time $?timestamp)
    (end_point_1 $?ep1)
    (end_point_2 $?ep2)
    (frame_id ?frame)
  )
  (MotorInterface (id "Robotino") (vx ?vx) (vy ?vy))
  (exp-zone-margin ?zone-margin)
  ?ze-f <- (zone-exploration
    (name ?zn&:(eq ?zn (get-zone ?zone-margin
      (compensate-movement
        ?*EXP-MOVEMENT-COMPENSATION*
        (create$ ?vx ?vy)
        (laser-line-center-map ?ep1 ?ep2 ?frame ?timestamp)
        ?timestamp
      )
    )))
    (machine UNKNOWN)
    (line-visibility ?zn-vh&:(< ?zn-vh 1))
  )
=>
  (synced-modify ?ze-f line-visibility ?vh)
  (printout warn "EXP found line: " ?zn " vh: " ?vh crlf)
)


(defrule exp-found-cluster
  "Found a cluster: Remember it for later when we run out of lines to explore."
  (phase EXPLORATION)
  (game-time $?game-time)
  (Position3DInterface (id ?id&:(eq (sub-string 1 19 ?id) "/laser-cluster/mps/"))
    (visibility_history ?vh&:(> ?vh 1))
    (translation $?trans) (rotation $?rot)
    (frame ?frame) (time $?timestamp)
  )
  (MotorInterface (id "Robotino") (vx ?vx) (vy ?vy))
  (exp-zone-margin ?zone-margin)
  ?ze-f <- (zone-exploration
    (name ?zn&:(eq ?zn (get-zone ?zone-margin
      (compensate-movement
        ?*EXP-MOVEMENT-COMPENSATION*
        (create$ ?vx ?vy)
        ?trans
        ?timestamp
      )
    )))
    (machine UNKNOWN)
    (cluster-visibility ?zn-vh)
    (last-cluster-time ?ctime&:(>= (nth$ 1 ?game-time) (+ ?ctime 2)))
  )
=>
  (printout t "EXP cluster ze-f: " ?ze-f crlf)
  (synced-modify ?ze-f
    cluster-visibility (+ ?zn-vh 1)
    last-cluster-time (nth$ 1 ?game-time)
  )
  (printout warn "EXP found cluster: " ?zn " vh: " (+ ?zn-vh 1) crlf)
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
  ?ze-f <- (zone-exploration
    (name ?zn&:(eq ?zn (get-zone ?zone-margin (transform-safe "map" ?frame ?timestamp ?trans ?rot))))
    (machine UNKNOWN)
    (line-visibility ?lv&:(< ?lv 2))
  )
  (not (locked-resource (resource ?r&:(eq ?r ?zn))))
  ?st-f <- (state ?)
=>
  (synced-modify ?ze-f line-visibility (+ ?lv 1))
)


(defrule exp-try-locking-line
  (phase EXPLORATION)
  (exp-searching)
  (state EXP_GOTO_NEXT|EXP_IDLE)
  ; Not currently locked/trying to lock anything
  (not (lock (type GET) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?)))
  (not (lock (type ACCEPT) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?)))

  ; An explorable zone for which no lock was refused yet
  (exp-repeated-search-limit ?search-limit)
  (zone-exploration
    (name ?zn)
    (machine UNKNOWN)
    (line-visibility ?vh&:(> ?vh 0))
    (times-searched ?ts&:(<= ?ts ?search-limit))
  )
  (not (lock (type REFUSE) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?zn)))

  ; Neither this zone nor the opposite zone is locked
  (not (locked-resource (resource ?r&:(eq ?r ?zn))))
  ;(not (locked-resource (resource ?r2&:(eq ?r2 (mirror-name ?zn)))))

  ; Locks for all closer zones with a line-visibility > 0 have been refused
  (Position3DInterface (id "Pose") (translation $?trans))
  (forall
    (zone-exploration (machine UNKNOWN) (line-visibility ?vh2&:(> ?vh2 0))
      (name ?zn2&:(< (distance-mf ?trans (zone-center ?zn2)) (distance-mf ?trans (zone-center ?zn))))
    )
    (lock (type REFUSE) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?zn2))
  )
=>
  (printout t "EXP trying to lock zone " ?zn crlf)
  (assert
    (lock (type GET) (agent ?*ROBOT-NAME*) (resource ?zn))
  )
)


(defrule exp-increase-search-limit
  (phase EXPLORATION)

  ; There is an explorable zone...
  (zone-exploration (machine UNKNOWN) (line-visibility ?vh&:(> ?vh 0)))

  ; ... but no zone may be searched according to the repeated-search-limit
  ?sl-f <- (exp-repeated-search-limit ?search-limit)
  (not (zone-exploration
    (machine UNKNOWN)
    (line-visibility ?vh-tmp&:(> ?vh-tmp 0))
    (times-searched ?ts&:(<= ?ts ?search-limit))
  ))
=>
  (modify ?sl-f (+ ?search-limit 1))
)


(defrule exp-tried-locking-all-zones
  "There is at least one unexplored zone with a line, but locks have been denied for
   ALL unexplored zones. So clear all REFUSEs and start requesting locks from the beginning."
  (zone-exploration (name ?) (machine UNKNOWN) (line-visibility ?tmp&:(> ?tmp 0)))
  (forall
    (zone-exploration (name ?zn) (machine UNKNOWN) (line-visibility ?vh&:(> ?vh 0)))
    (lock (type REFUSE) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?zn))
  )
=>
  (delayed-do-for-all-facts ((?l lock)) (and (eq ?l:type REFUSE) (eq ?l:agent ?*ROBOT-NAME*))
    (retract ?l)
  )
)


(defrule exp-stop-to-investigate-zone
  "Lock for an explorable zone was accepted"
  (phase EXPLORATION)
  (exp-searching)
  ?st-f <- (state EXP_GOTO_NEXT)

  (zone-exploration (name ?zn))
  (lock (type ACCEPT) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?zn))
=>
  (printout t "EXP exploring zone " ?zn crlf)
  (delayed-do-for-all-facts ((?exp-f explore-zone-target)) TRUE (retract ?exp-f))
  (retract ?st-f)
  (assert
    (state EXP_STOPPING)
    (explore-zone-target (zone ?zn))
  )
  (skill-call relgoto x 0 y 0)
)


(defrule exp-skill-explore-zone
  (phase EXPLORATION)
  ?srch-f <- (exp-searching)
  ?st-f <- (state EXP_STOPPING)
  (explore-zone-target (zone ?zn))
  ?skill-f <- (skill-done (name "relgoto"))
  (MotorInterface (id "Robotino")
    (vx ?vx&:(< ?vx 0.01)) (vy ?vy&:(< ?vy 0.01)) (omega ?w&:(< ?w 0.01))
  )
  (navigator-default-vmax (velocity ?trans-vmax) (rotation ?rot-vmax))
=>
  (retract ?st-f ?srch-f ?skill-f)
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
  (machine (name ?machine) (mtype ?mtype))
  ?exp-f <- (explore-zone-target (zone ?zn))
=>
  (retract ?st-f ?skill-f ?exp-f)
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
  ?exp-f <- (explore-zone-target (zone ?zn))
  (ZoneInterface (id "/explore-zone/info") (zone ?zn-str) (search_state ?s&:(neq ?s YES)))
  ?ze <- (zone-exploration (name ?zn2&:(eq ?zn2 (sym-cat ?zn-str))) (machine ?machine) (times-searched ?times-searched))
=>
  (retract ?st-f ?skill-f ?exp-f)
  (if (eq ?status FINAL) then
    (printout error "BUG in explore_zone skill: Result is FINAL but no MPS was found.")
  )
  (if (and (eq ?s NO) (eq ?machine UNKNOWN)) then
    (synced-modify ?ze machine NONE times-searched (+ ?times-searched 1))
  else
    (synced-modify ?ze line-visibility 0 times-searched (+ ?times-searched 1))
  )
  (assert
    (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource (sym-cat ?zn-str)))
    (exp-searching)
    (state EXP_IDLE)
  )
)


(defrule exp-report-to-refbox
  (phase EXPLORATION)
  (team-color ?color)
  (exploration-result (team ?color) (machine ?machine) (zone ?zone)
    (orientation ?orientation)
  )
  (time $?now)
  ?ws <- (timer (name send-machine-reports) (time $?t&:(timeout ?now ?t 1)) (seq ?seq))
  (game-time $?game-time)
  (confval (path "/clips-agent/rcll2016/exploration/latest-send-last-report-time")
    (value ?latest-report-time)
  )
  (team-color ?team-color&~nil)
  (peer-id private ?peer)
  (state ?s) ; TODO actually enter EXP_PREPARE_FOR_PRODUCTION_FINISHED state
=>
  (bind ?mr (pb-create "llsf_msgs.MachineReport"))
  (pb-set-field ?mr "team_color" ?team-color)
  (delayed-do-for-all-facts ((?er exploration-result)) (eq ?er:team ?team-color)
    (bind ?n-explored (length
      (find-all-facts ((?f zone-exploration))
        (and (eq ?f:team ?team-color) (neq ?f:machine UNKNOWN))
      )
    ))
    (bind ?n-zones (length
      (find-all-facts ((?f zone-exploration)) (eq ?f:team ?team-color))
    ))
    ; send report for last machine only if the exploration phase is going to end
    ; or we are prepared for production
    (if
      (or
        (< ?n-explored (- ?n-zones 1))
        (>= (nth$ 1 ?game-time) ?latest-report-time)
        (eq ?s EXP_PREPARE_FOR_PRODUCTION_FINISHED)
      )
    then
      (bind ?mre (pb-create "llsf_msgs.MachineReportEntry"))
      (pb-set-field ?mre "name" (str-cat ?er:machine))
      (pb-set-field ?mre "zone" (protobuf-name ?er:zone))
      (pb-set-field ?mre "rotation" ?er:orientation)
      (pb-add-list ?mr "machines" ?mre)
    )
  )
  (pb-broadcast ?peer ?mr)
  (modify ?ws (time ?now) (seq (+ ?seq 1)))
)


(defrule exp-mirror-tag
  (found-tag (name ?machine) (side ?side) (frame ?frame) (trans $?trans) (rot $?rot))
  ; Assuming that ?frame is always "map". Otherwise things will break rather horribly...

  (not (field-ground-truth (machine ?machine)))
  ; Do not trigger while updating zones/tags from Refbox PB msg after exploration

  (tag-matching (tag-id ?tag) (machine ?machine) (side ?side))
  (zone-exploration (name ?zn) (machine ?machine) (times-searched ?times-searched))

  (tag-matching (tag-id ?tag2) (side ?side)
    (machine ?machine2&:(eq ?machine2 (mirror-name ?machine)))
  )
  (not (found-tag (name ?machine2&:(eq ?machine2 (mirror-name ?machine)))))
  ?ze2 <- (zone-exploration (name ?zn2&:(eq ?zn2 (mirror-name ?zn))))
  (machine (name ?machine) (mtype ?mtype))
=>
  (bind ?m-rot (mirror-rot ?mtype ?zn ?rot))
  (if (neq ?rot ?m-rot) then
    (bind ?tag-yaw (tf-yaw-from-quat ?rot))
    (bind ?c-trans (translate-tag-x ?tag-yaw -0.17 ?trans))
    (bind ?m-trans
      (translate-tag-x
        (tf-yaw-from-quat ?m-rot)
        0.17
        (mirror-trans ?c-trans)
      )
    )
  else
    (bind ?m-trans (mirror-trans ?trans))
  )
  (assert
    (found-tag (name ?machine2) (side ?side) (frame ?frame)
      (trans ?m-trans) (rot ?m-rot)
    )
  )
  (synced-modify ?ze2 machine ?machine2 times-searched ?times-searched)
)


(defrule exp-add-tag-to-navgraph
  (not (requested-waiting-positions))
  (or
    (not (last-navgraph-compute-msg))
    (last-navgraph-compute-msg (final TRUE))
    (NavGraphWithMPSGeneratorInterface (msgid 0))
    (not (NavGraphWithMPSGeneratorInterface))
  )

  (forall (machine (name ?mps))
    (zone-exploration (machine ?found&:(eq ?mps ?found)))
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
  (assert (generating-navgraph))
  (navgraph-add-all-new-tags)
)


(defrule exp-navgraph-done
  ?gen-f <- (generating-navgraph)
  (last-navgraph-compute-msg (final TRUE))
=>
  (retract ?gen-f)
  (assert (navgraph-done))
)


(defrule exp-exploration-ends-cleanup
  "Clean up lock refusal facts when exploration ends"
  (phase EXPLORATION)
  (change-phase PRODUCTION)
=>
  (delayed-do-for-all-facts ((?l lock)) (eq ?l:type REFUSE)
    (retract ?l)
  )
)
