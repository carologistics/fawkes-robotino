
;---------------------------------------------------------------------------
;  exploration.clp - Robotino agent for exploration phase
;
;  Copyright  2017-2018 Victor Mataré, Daniel Habering
;
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defglobal
  ?*EXP-MOVEMENT-COMPENSATION* = 0.0
  ?*EXP-SEARCH-LIMIT* = 1
)

(defrule exp-sync-ground-truth
" When the RefBox sends ground-truth of a zone, update the corresponding
  domain fact. But only do so for ground-truth of the own team, this allows to
  deal with halve-fields, where only one set of MPS is present but ground truth
  for all machines is sent.
"
	(wm-fact (key game found-tag zone args? m ?mps) (value ?zone))
	?zc <- (domain-fact (name zone-content) (param-values ?zone ?o-mps&:(neq ?o-mps ?mps)))
	=>
	(modify ?zc (param-values ?zone ?mps))
)

(defrule exp-enable
" Exploration is needed as we received an mps-state already without knowing
  the zone."
	(or (and (wm-fact (key domain fact mps-state args? m ?name $?))
		       (not (wm-fact (key domain fact zone-content args? z ? m ?name))))
	    (wm-fact (key refbox phase) (value EXPLORATION)))
	(not (wm-fact (key exploration active) (type BOOL) (value TRUE)))
	=>
	(assert (wm-fact (key exploration active) (type BOOL) (value TRUE)))
)

(defrule exp-disable-goals
" Exploration is not needed anymore as all machines were found."
	?g <-(goal (class EXPLORE-ZONE|EXPLORATION-MOVE) (mode FORMULATED))
	(wm-fact (key exploration active) (type BOOL) (value FALSE))
	=>
	(retract ?g)
)

(defrule exp-fail-goals
" Exploration is not needed anymore as all machines were found."
	?g <-(goal (class EXPLORE-ZONE|EXPLORATION-MOVE) (mode DISPATCHED))
	(wm-fact (key exploration active) (type BOOL) (value FALSE))
	=>
	(modify ?g (mode FINISHED) (outcome FAILED))
)

(defrule exp-startup
" Asserts all needed wm-facts for the exploration phase
  line-vis:      value is greater 0 if there was a line detected in the zone
  tag-vis:       value is greater 0 if there was a tag detected in the zone
  time-searched: Number of times this zone was activly explored
  zone:          Which machine is in this zone. Initially every zone has an UNKNOWN machine.
                 If there is reason to believe, that the zone is empty, machine is NONE
                 If a machine was detected, this contains the name of the machine (eg C-CS1)
"
	(not (wm-fact (key domain fact zone-content args? z ?zn m ?machine)))
	(wm-fact (id "/config/rcll/exploration/zone-margin") (type FLOAT) (value ?zone-margin))
	(confval (path ?min-path&:(eq ?min-path (str-cat ?*NAVGRAPH_GENERATOR_MPS_CONFIG* "bounding-box/p1")))
	         (list-value ?x_min ?y_min))
	(confval (path ?max-path&:(eq ?max-path (str-cat ?*NAVGRAPH_GENERATOR_MPS_CONFIG* "bounding-box/p2")))
	         (list-value ?x_max ?y_max))
=>
	(bind ?zones (create$))
	(loop-for-count (?x ?x_min -1)
		(loop-for-count (?y (+ 1 ?y_min) ?y_max)
			(bind ?zones (append$ ?zones (translate-location-grid-to-map (abs ?x) ?y)))
		)
	)
	(loop-for-count (?x 1 ?x_max)
		(loop-for-count (?y (+ 1 ?y_min) ?y_max)
			(bind ?zones (append$ ?zones (translate-location-grid-to-map (abs ?x) ?y)))
		)
	)
	(assert (exp-zone-margin ?zone-margin))

   (foreach ?zone ?zones
     (assert (wm-fact (key exploration fact line-vis args? zone ?zone) (value 0) (type INT) (is-list FALSE) )
             (wm-fact (key exploration fact tag-vis args? zone ?zone) (value 0) (type INT) (is-list FALSE) )
             (wm-fact (key exploration fact time-searched args? zone ?zone) (value 0) (type INT) (is-list FALSE) )
             (domain-fact (name zone-content) (param-values ?zone UNKNOWN))
     )
   )
)


(defrule exp-conf-get-vmax
" Reads maximum values for rotating and velocity from the config and stores it as a fact
"
  (wm-fact (id "/config/rcll/exploration/low-velocity") (type FLOAT) (value ?low-velocity))
  (wm-fact (id "/config/rcll/exploration/low-rotation") (type FLOAT) (value ?low-rotation))
  (wm-fact (id "/config/rcll/exploration/max-velocity") (type FLOAT) (value ?max-velocity))
  (wm-fact (id "/config/rcll/exploration/max-rotation") (type FLOAT) (value ?max-rotation))
	(wm-fact (key central agent robot args? r ?robot))
	=>
	(assert (exp-navigator-vmax ?robot ?max-velocity ?max-rotation))
	(assert (exp-navigator-vlow ?robot ?low-velocity ?low-rotation))
)

(defrule exp-create-exploration-timer
" Initial goal creating
  Refer to fixed-squence.clp for the expandation of the goal and the creation of the EXPLORATION-PLAN
  The EXPLORATION-PLAN let the robot visit a number of configurable points. If a possible machine was detected, this plan is interrupted
"
  (wm-fact (key refbox phase) (value EXPLORATION))
  (wm-fact (key game state) (value RUNNING))
   =>
  (assert (timer (name send-machine-reports)))
)

(defrule exp-passed-through-quadrant
" If the robot drove through a zone slow enough and passed the middle of the zone with a certain margin
  we can conclude, that there is no machine in this zone
"
	(wm-fact (key central agent robot args? r ?r))
	(exp-navigator-vlow ?r ?max-velocity ?max-rotation)
	(MotorInterface (id ?motor-id &:(eq ?motor-id (remote-if-id ?r "Robotino")))
	  (vx ?vx&:(< ?vx ?max-velocity)) (vy ?vy&:(< ?vy ?max-velocity)) (omega ?w&:(< ?w ?max-rotation))
	)
	; The visibility history corresponds to the confidence of how accurate the
	; pose is
	(Position3DInterface (id ?pose-id&:(eq ?pose-id (remote-if-id ?r "Pose"))) (translation $?trans)
	                     (time $?timestamp) (visibility_history ?vh&:(>= ?vh 10)))
	?ze <- (wm-fact (key exploration fact time-searched args? zone ?zn&:(eq ?zn (get-zone 0.15 ?trans))) (value ?time-searched))
	?zm <- (domain-fact (name zone-content) (param-values ?zn UNKNOWN))
=>
	(bind ?zone (get-zone 0.07 ?trans))
	(if ?zone then
		(modify ?ze (value (+ 1 ?time-searched)))
		(modify ?zm (param-values ?zn NONE))
		(printout t "Passed through " ?zn crlf)
	)
)


(defrule exp-found-line
" If a laserline was found, that lies inside a zone with a certain margin,
  update the line-vis fact of this zone
"
	(wm-fact (key central agent robot args? r ?r))
	(LaserLineInterface
	  (id ?laser-id&:(str-index (str-cat ?r) ?laser-id))
	  (visibility_history ?vh&:(>= ?vh 1))
	  (time $?timestamp)
	  (end_point_1 ?e1 ?e2 $?)
	  (end_point_2 ?e3 ?e4 $?)
	  (frame_id ?frame)
	)
	(exp-zone-margin ?zone-margin)
	?ze-f <- (wm-fact (key exploration fact line-vis
	                  args? zone ?zn&:(eq ?zn (get-zone ?zone-margin
	                                          (get-2d-center ?e1 ?e2 ?e3 ?e4))))
	                                          (value ?zn-vh&:(< ?zn-vh 1) ))
	=>
	(modify ?ze-f (value 1 ))
	(printout warn "EXP found line: " ?zn " vh: " ?vh crlf)
)


(defrule exp-sync-mirrored-zone
" Every knowledge of a zone can be snyced to its counterpart on the other side of the field, since they are symmetrical
  Only syncs NONE machine, since syncing of a found machine is handled in another rule
"
  (wm-fact (key domain fact zone-content args? z ?zn m NONE))
  ?we <- (domain-fact (name zone-content) (param-values ?zn2&:(eq ?zn2 (mirror-name ?zn)) UNKNOWN))
  =>
  (modify ?we (param-values ?zn NONE))
  (printout t "Synced zone: " ?zn2 crlf)
)


(defrule exp-exclude-zones
" Mark a zone as empty if there can not be a machine according to the rules
"
  (exploration-result (zone ?zn) (machine ?machine) (orientation ?orientation) )
  ?wm <- (domain-fact (name zone-content) (param-values ?zn2 UNKNOWN))
  (test (eq TRUE (zone-is-blocked ?zn ?orientation ?zn2 ?machine)))
  =>
  (modify ?wm (param-values ?zn NONE))
  (printout t "There is a machine in " ?zn " with orientation " ?orientation  " so block " ?zn2 crlf)
)

(defrule exp-found-tag
" If a tag was found in a zone that we dont have any information of, update the corresponding tag-vis fact
"
	(wm-fact (key central agent robot args? r ?r))
  (domain-fact (name tag-matching) (param-values ?machine ?side ?col ?tag))
  (TagVisionInterface (id ?tag-vis-id &:(eq ?tag-vis-id (remote-if-id ?r "tag-vision/info")))
    (tags_visible ?num-tags&:(> ?num-tags 0))
    (tag_id $?tag-ids&:(member$ ?tag ?tag-ids))
  )
  (Position3DInterface
    (id ?tag-if-id&:(eq ?tag-if-id (str-cat (remote-if-id ?r (str-cat "tag-vision/" (- (member$ ?tag ?tag-ids) 1) "/to_map")))))
    (visibility_history ?vh&:(> ?vh 1))
    (translation ?x ?y $?)
  )
  (exp-zone-margin ?zone-margin)
  ?ze-f <- (wm-fact (key exploration fact tag-vis args? zone ?zn&:(eq ?zn (get-zone ?zone-margin ?x ?y))) (value ?tv&:(< ?tv 1) ))
  (wm-fact (key domain fact zone-content args? z ?zn m UNKNOWN))
=>
  (modify ?ze-f (value 1 ))
  (printout t "Found tag in " ?zn crlf)
)


(defrule exp-sync-tag-finding
" Sync finding of a tag to the other field size
"
  ?wm <- (wm-fact (key exploration fact tag-vis args? zone ?zn) (value ?tv))
  ?we <- (wm-fact (key exploration fact tag-vis args? zone ?zn2&:(eq ?zn2 (mirror-name ?zn))) (value ?tv2&: (< ?tv2 ?tv)))
  =>
  (modify ?we (value ?tv))
  (printout t "Synced tag-finding: " ?zn2 crlf)
)


(defrule exp-start-zone-exploring
" If there is a zone, where we suspect a machine, interrupt the EXPLORATION-PLAN and start exploring the zone
"
	(wm-fact (key central agent robot args? r ?r))
  (goal (id ?parent) (class EXPLORATION-ROOT))
  (Position3DInterface (id ?pos-id&:(eq ?pos-id (remote-if-id ?r "Pose"))) (translation $?trans))
  ?ze <- (wm-fact (key exploration fact time-searched args? zone ?zn) (value ?ts&:(<= ?ts ?*EXP-SEARCH-LIMIT*)))
  (wm-fact (key domain fact zone-content args? z ?zn m UNKNOWN))
  (wm-fact (key exploration fact line-vis args? zone ?zn) (value ?vh))
  (wm-fact (key exploration fact tag-vis args? zone ?zn) (value ?tv))

  ; Either a tag or a line was found in this zone"
  (test (or (> ?tv 0) (> ?vh 0)))

  ; Since the finding of a tag is a stronger indicator of a machine than a line, we prefer zones were a tag was found
  ; Either tag-vis is greater 0 or there is no zone with tag-vis greater than 0 and a line was found in this zone
  (or (test (> ?tv 0))
      (not (and (wm-fact (key exploration fact tag-vis args? zone ?zn2) (value ?tv2&:(> ?tv2 0)))
		            (wm-fact (key exploration fact time-searched args? zone ?zn2) (value ?ts2&:(<= ?ts2 ?*EXP-SEARCH-LIMIT*)))
		            (wm-fact (key domain fact zone-content args? z ?zn2 m UNKNOWN))
	              (not (exploration-result (zone ?zn2)))
	         )
      )
  )

  ; Check that there is no zone with the same indicator of a machine (tag or line) that is closer
  (not (and (wm-fact (key exploration fact line-vis args? zone ?zn3&:(< (distance-mf (zone-center ?zn3) ?trans) (distance-mf (zone-center ?zn) ?trans))) (value ?vh3& : (not (and (= ?vh3 0) (= ?tv 0)))))
	          (wm-fact (key exploration fact tag-vis args? zone ?zn3) (value ?tv3& : (not (and (> ?tv 0) (= ?tv3 0)))))
	          (wm-fact (key exploration fact time-searched  args? zone ?zn3) (value ?ts3&:(<= ?ts3 ?*EXP-SEARCH-LIMIT*)))
	          (wm-fact (key domain fact zone-content args? z ?zn3 m UNKNOWN))
            (not (exploration-result (zone ?zn3)))
	     )
  )

  (not (goal (class EXPLORE-ZONE) (params z ?zn) (outcome COMPLETED)))

  ; Only start interrupting the EXPLORATION-PLAN if the first move action was finished.
  ; This prohibits, that all bots start exploring zones right in front of the insertion zone
  (not (exploration-result (zone ?zn)))
	; TODO: set navigator speed to max when executing the plan
  ;(exp-navigator-vmax ?r ?vel ?rot)
  =>
  (bind ?new-ts (+ 1 ?ts))
  (modify ?ze (value ?new-ts))
  (printout t "Goal EXPLORE-ZONE  formulated in zone: " ?zn " with tag: " ?tv crlf)
  (assert (goal (id (sym-cat EXPLORE-ZONE- (gensym*))) (class EXPLORE-ZONE) (sub-type SIMPLE)
	              (params z ?zn) (meta-template goal-meta) (parent ?parent) (priority 10.) (mode FORMULATED)))
)

(defrule exp-explore-zone-executable
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?id) (class EXPLORE-ZONE) (params z ?zn) (mode FORMULATED)
	      (is-executable FALSE))
	(wm-fact (key domain fact zone-content args? z ?zn m UNKNOWN))
	(goal-meta (goal-id ?id) (assigned-to ?robot&~nil))
	(not (and (goal (id ?other-id) (class EXPLORE-ZONE) (params z ?zn) (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED))
	          (goal-meta (goal-id ?other-id) (assigned-to ?other-robot&:(neq ?other-robot ?robot))))
	)
	=>
	(modify ?g (is-executable TRUE))
)

(defrule exp-explore-zone-retract-not-executable
	(declare (salience ?*SALIENCE-GOAL-REJECT*))
	?g <- (goal (id ?id) (class EXPLORE-ZONE) (params z ?zn) (mode FORMULATED) (is-executable FALSE))
	?gm <- (goal-meta (goal-id ?id) (assigned-to ?robot&~nil))
	=>
	(retract ?g ?gm)
)

(defrule exp-increase-search-limit
" There are zones with tag or line findings, but the search limit is reached
  Then the search limit is incremented, to enable reexploration as a fallback solution
"
  (goal (class EXPLORATION) (mode DISPATCHED))

  (wm-fact (key exploration fact line-vis args? zone ?zn1) (value ?vh))
  (wm-fact (key exploration fact tag-vis args? zone ?zn1) (value ?tv))
  (wm-fact (key domain fact zone-content args? z ?zn1 m UNKNOWN))
  (test (or (> ?tv 0) (> ?vh 0)))

  (not (and
	       (wm-fact (key exploration fact line-vis args? zone ?zn2) (value ?vh-tmp))
	       (wm-fact (key exploration fact tag-vis args? zone ?zn2) (value ?tv-tmp))
         (wm-fact (key exploration fact time-searched args? zone ?zn2) (value ?ts&:(< ?ts ?*EXP-SEARCH-LIMIT*)))
	       (wm-fact (key domain fact zone-content args? z ?zn2 m UNKNOWN))
	       (test (or (> ?vh-tmp 0) (> ?tv-tmp 0)))
	     )
	)
=>
  (modify ?*EXP-SEARCH-LIMIT* (+ ?*EXP-SEARCH-LIMIT* 1))
)

(defrule exp-skill-explore-zone-apply-sensed-effects
" Exploration of a zone finished succesfully. Update zone wm-fact and assert exploration-result
"
  (goal (id ?goal-id) (class EXPLORE-ZONE) (mode DISPATCHED))
  (goal-meta (goal-id ?goal-id) (assigned-to ?r&~nil))
  (plan-action (action-name explore-zone) (state SENSED-EFFECTS-WAIT))
  (ZoneInterface (id ?zone-id&:(eq ?zone-id (remote-if-id ?r "explore-zone/info")))
	               (zone ?zn-str) (orientation ?orientation)
	               (tag_id ?tag-id) (search_state YES)
  )
  (TagVisionInterface (id ?tag-vis-id &:(eq ?tag-vis-id (remote-if-id ?r "tag-vision/info")))
    (tag_id $?tag-ids&:(member$ ?tag-id ?tag-ids))
  )
  (Position3DInterface
    (id ?tag-if-id&:(eq ?tag-if-id (str-cat (remote-if-id ?r (str-cat "tag-vision/" (- (member$ ?tag-id ?tag-ids) 1) "/to_map")))))
    (translation $?trans)
    (rotation $?rot)
  )
  (domain-fact (name tag-matching) (param-values ?machine ?side ?team-color ?tag-id))
  (wm-fact (key domain fact mps-type args? m ?machine t ?mtype))
  ?ze <- (wm-fact (key exploration fact time-searched args? zone ?zn2&:(eq ?zn2 (sym-cat ?zn-str))) (value ?times-searched))
  ?zm <- (domain-fact (name zone-content) (param-values ?zn2 ?))
  ; This is for a mirrored field
  ;?zm2 <- (domain-fact (name zone-content) (param-values ?zn3&:(eq (mirror-name ?zn2) ?zn3) ?))

  (not (exploration-result (machine ?machine) (zone ?zn2)))
  =>
  (modify ?ze (value (+ 1 ?times-searched)))
  (modify ?zm (param-values ?zn2 ?machine))
  ;(modify ?zm2 (param-values ?zn3 (mirror-name ?machine)))
  (assert
    (exploration-result
      (machine ?machine) (zone ?zn2)
      (orientation ?orientation)
      (team ?team-color)
      (trans ?trans)
      (rot ?rot)
      (tag-id ?tag-id)
    )
  )
  (printout t "EXP exploration fact zone successfull. Found " ?machine " in " ?zn2 crlf)
)

(defrule exp-report-to-refbox
" Regularly send all found machines to the refbox"
  (wm-fact (key refbox phase) (value EXPLORATION))
  (wm-fact (key refbox team-color) (value ?color))
  (exploration-result (team ?color) (machine ?machine) (zone ?zone)
    (orientation ?orientation)
  )
  (time $?now)
  ?ws <- (timer (name send-machine-reports) (time $?t&:(timeout ?now ?t 1)) (seq ?seq))
  (wm-fact (key refbox game-time) (values $?game-time))
  (wm-fact (id "/config/rcll/exploration/latest-send-last-report-time")
    (value ?latest-report-time)
  )
  (wm-fact (key refbox team-color) (value ?team-color&~nil))
  (wm-fact (key refbox comm peer-id public) (value ?peer))
=>
  (bind ?mr (pb-create "llsf_msgs.MachineReport"))
  (pb-set-field ?mr "team_color" ?team-color)
  (delayed-do-for-all-facts ((?er exploration-result)) (eq ?er:team ?team-color)
      (bind ?mre (pb-create "llsf_msgs.MachineReportEntry"))
      (pb-set-field ?mre "name" (str-cat ?er:machine))
      (pb-set-field ?mre "zone" (protobuf-name ?er:zone))
      (pb-set-field ?mre "rotation" ?er:orientation)
      (pb-add-list ?mr "machines" ?mre)
  )
  (pb-broadcast ?peer ?mr)
  (modify ?ws (time ?now) (seq (+ ?seq 1)))
)

(defrule exp-stop-when-all-found
	?exp-active <- (wm-fact (key exploration active) (type BOOL) (value TRUE))
	(not (and (wm-fact (key domain fact mps-type args? m ?target-mps $?))
	          (not (domain-fact (name zone-content)
	                            (param-values ?zz ?target-mps))
	)))
	(wm-fact (key refbox phase) (value PRODUCTION))
	=>
	(delayed-do-for-all-facts ((?exp wm-fact))
		(wm-key-prefix ?exp:key (create$ exploration fact))
		(retract ?exp)
	)
	(modify ?exp-active (value FALSE))
)
