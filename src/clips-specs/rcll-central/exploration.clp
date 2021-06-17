
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


(deftemplate exploration-result
" Template for storing a exploration result. Stores the machine name, zone, orientation and the team this machine belongs to"
  (slot machine (type SYMBOL) (allowed-symbols C-BS C-CS1 C-CS2 C-RS1 C-RS2 C-DS C-SS M-BS M-CS1 M-CS2 M-RS1 M-RS2 M-DS M-SS))
  (slot zone (type SYMBOL))
  (slot orientation (type INTEGER) (default -1))
  (slot team (type SYMBOL) (allowed-symbols CYAN MAGENTA))
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
    (not (wm-fact (key exploration zone ?zn args? machine ?machine team ?team)))
    (wm-fact (key refbox phase) (value EXPLORATION))
=>
    (bind $?Czones (create$
      C-Z18 C-Z28 C-Z38 C-Z48 C-Z58 C-Z68 C-Z78
      C-Z17 C-Z27 C-Z37 C-Z47 C-Z57 C-Z67 C-Z77
      C-Z16 C-Z26 C-Z36 C-Z46 C-Z56 C-Z66 C-Z76
      C-Z15 C-Z25 C-Z35 C-Z45 C-Z55 C-Z65 C-Z75
      C-Z14 C-Z24 C-Z34 C-Z44 C-Z54 C-Z64 C-Z74
      C-Z13 C-Z23 C-Z33 C-Z43 C-Z53 C-Z63 C-Z73
      C-Z12 C-Z22 C-Z32 C-Z42 C-Z52 C-Z62 C-Z72
      C-Z11 C-Z21 C-Z31 C-Z41))

    (bind $?Mzones (create$
      M-Z78 M-Z68 M-Z58 M-Z48 M-Z38 M-Z28 M-Z18
      M-Z77 M-Z67 M-Z57 M-Z47 M-Z37 M-Z27 M-Z17
      M-Z76 M-Z66 M-Z56 M-Z46 M-Z36 M-Z26 M-Z16
      M-Z75 M-Z65 M-Z55 M-Z45 M-Z35 M-Z25 M-Z15
      M-Z74 M-Z64 M-Z54 M-Z44 M-Z34 M-Z24 M-Z14
      M-Z73 M-Z63 M-Z53 M-Z43 M-Z33 M-Z23 M-Z13
      M-Z72 M-Z62 M-Z52 M-Z42 M-Z32 M-Z22 M-Z12
                        M-Z41 M-Z31 M-Z21 M-Z11))

     (foreach ?zone ?Czones
       (assert (wm-fact (key exploration fact line-vis args? zone ?zone) (value 0) (type INT) (is-list FALSE) )
               (wm-fact (key exploration fact tag-vis args? zone ?zone) (value 0) (type INT) (is-list FALSE) )
               (wm-fact (key exploration fact time-searched args? zone ?zone) (value 0) (type INT) (is-list FALSE) )
               (wm-fact (key exploration zone ?zone args? machine UNKNOWN team CYAN))
       )
     )
     (foreach ?zone ?Mzones
       (assert (wm-fact (key exploration fact line-vis args? zone ?zone) (value 0) (type INT) (is-list FALSE) )
               (wm-fact (key exploration fact tag-vis args? zone ?zone) (value 0) (type INT) (is-list FALSE) )
               (wm-fact (key exploration fact time-searched args? zone ?zone) (value 0) (type INT) (is-list FALSE) )
               (wm-fact (key exploration zone ?zone args? machine UNKNOWN team MAGENTA)))
     )

)


(defrule exp-conf-get-vmax
" Reads maximum values for rotating and velocity from the config and stores it as a fact
"
  (wm-fact (id "/config/rcll/exploration/low-velocity") (type FLOAT) (value ?low-velocity))
  (wm-fact (id "/config/rcll/exploration/low-rotation") (type FLOAT) (value ?low-rotation))
  (wm-fact (id "/config/rcll/exploration/max-velocity") (type FLOAT) (value ?max-velocity))
  (wm-fact (id "/config/rcll/exploration/max-rotation") (type FLOAT) (value ?max-rotation))
  =>
	(do-for-all-facts ((?robot-wm wm-fact)) (wm-key-prefix ?robot-wm:key (create$ central agent robot))
		(assert (exp-navigator-vmax (wm-key-arg ?robot-wm:key r) ?max-velocity ?max-rotation))
		(assert (exp-navigator-vlow (wm-key-arg ?robot-wm:key r) ?low-velocity ?low-rotation))
	)
)


(defrule exp-create-exploration-goal
" Initial goal creating
  Refer to fixed-squence.clp for the expandation of the goal and the creation of the EXPLORATION-PLAN
  The EXPLORATION-PLAN let the robot visit a number of configurable points. If a possible machine was detected, this plan is interrupted
"


  (wm-fact (key domain fact entered-field args? r ?r))
	(wm-fact (key central agent robot args? r ?r))
  (not (goal (id ?goal-id) (class EXPLORATION) (meta assigned-to ?r)))
  (wm-fact (key refbox phase) (value EXPLORATION))
  (wm-fact (key game state) (value RUNNING))

  ?cv <- (wm-fact (id "/config/rcll/exploration/zone-margin") (type FLOAT) (value ?zone-margin))
  (exp-navigator-vlow ?r ?vel ?rot)
  ; TODO: do not assign the robot while creating the goal
   =>
  (assert (exp-zone-margin ?zone-margin))
  (assert (timer (name send-machine-reports)))
  (assert (goal (id (sym-cat EXPLORATION- (gensym*))) (class EXPLORATION)
                (type ACHIEVE) (sub-type SIMPLE) (meta assigned-to ?r)))
)

(defrule exp-exploration-executable
" Initial goal creating
  Refer to fixed-squence.clp for the expandation of the goal and the creation of the EXPLORATION-PLAN
  The EXPLORATION-PLAN let the robot visit a number of configurable points. If a possible machine was detected, this plan is interrupted
"
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (class EXPLORATION)
	            (type ACHIEVE) (sub-type SIMPLE) (meta assigned-to ?r) (is-executable FALSE) (mode FORMULATED))
	(wm-fact (key domain fact entered-field args? r ?r))
	(wm-fact (key central agent robot args? r ?r))
	?cv <- (wm-fact (id "/config/rcll/exploration/zone-margin") (type FLOAT) (value ?zone-margin))
	(exp-navigator-vlow ?r ?vel ?rot)
	=>
	(modify ?g (is-executable TRUE))
	; TODO: this should only be set if the goal is executed
	"Lower the speed, for a more robust line and tag detection"
	(navigator-set-speed ?r ?vel ?rot)
)

(defrule exp-passed-through-quadrant
" If the robot drove through a zone slow enough and passed the middle of the zone with a certain margin
  we can conclude, that there is no machine in this zone
"
  (goal (class EXPLORATION) (mode DISPATCHED) (meta assigned-to ?r))
  (exp-navigator-vlow ?r ?max-velocity ?max-rotation)
  (MotorInterface (id ?motor-id &:(eq ?motor-id (remote-if-id ?r "Robotino")))
    (vx ?vx&:(< ?vx ?max-velocity)) (vy ?vy&:(< ?vy ?max-velocity)) (omega ?w&:(< ?w ?max-rotation))
  )
  (Position3DInterface (id ?pose-id&:(eq ?pose-id (remote-if-id ?r "Pose"))) (translation $?trans)
	                     (time $?timestamp) (visibility_history ?vh&:(>= ?vh 10)))
  ?ze <- (wm-fact (key exploration fact time-searched args? zone ?zn&:(eq ?zn (get-zone 0.15 ?trans))) (value ?time-searched))
  ?zm <- (wm-fact (key exploration zone ?zn args? machine UNKNOWN team ?team))
=>
  (bind ?zone (get-zone 0.07 ?trans))
  (if ?zone then
    (modify ?ze (key exploration fact time-searched args? zone ?zn) (value (+ 1 ?time-searched)))
    (modify ?zm (key exploration zone ?zn args? machine NONE team ?team))
    (printout t "Passed through " ?zn crlf)
  )
)


(defrule exp-found-line
" If a laserline was found, that lies inside a zone with a certain margin,
  update the line-vis fact of this zone
"
  (goal (class EXPLORATION) (mode DISPATCHED) (meta assigned-to ?r))
  (LaserLineInterface
		(id ?laser-id&:(str-index (str-cat ?r) ?laser-id))
    (visibility_history ?vh&:(>= ?vh 1))
    (time $?timestamp)
    (end_point_1 $?ep1)
    (end_point_2 $?ep2)
    (frame_id ?frame)
  )
  (MotorInterface (id ?motor-id &:(eq ?motor-id (remote-if-id ?r "Robotino")))
	                (vx ?vx) (vy ?vy))

  (exp-zone-margin ?zone-margin)
  ?ze-f <- (wm-fact (key exploration fact line-vis args? zone ?zn&:(eq ?zn (get-zone ?zone-margin
                                            (compensate-movement
                                              ?*EXP-MOVEMENT-COMPENSATION*
                                              (create$ ?vx ?vy)
                                              (laser-line-center-map ?ep1 ?ep2 ?frame ?timestamp)
                                              ?timestamp)))) (value ?zn-vh&:(< ?zn-vh 1) ))
=>
  (modify ?ze-f (key exploration fact line-vis args? zone ?zn) (value 1 ))
  (printout warn "EXP found line: " ?zn " vh: " ?vh crlf)
)


(defrule exp-sync-mirrored-zone
" Every knowledge of a zone can be snyced to its counterpart on the other side of the field, since they are symmetrical
  Only syncs NONE machine, since syncing of a found machine is handled in another rule
"
  ?wm <- (wm-fact (key exploration zone ?zn args? machine NONE team ?))
  ?we <- (wm-fact (key exploration zone ?zn2&:(eq ?zn2 (mirror-name ?zn)) args? machine UNKNOWN team ?team2))
  =>
  (modify ?we (key exploration zone ?zn2 args? machine NONE team ?team2))
  (printout t "Synced zone: " ?zn2 crlf)
)


(defrule exp-exclude-zones
" Mark a zone as empty if there can not be a machine according to the rules
"
  (exploration-result (zone ?zn) (machine ?machine) (orientation ?orientation) )
  ?wm <- (wm-fact (key exploration zone ?zn2 args? machine UNKNOWN team ?team))
  (test (eq TRUE (zone-is-blocked ?zn ?orientation ?zn2 ?machine)))
  =>
  (modify ?wm (key exploration zone ?zn2 args? machine NONE team ?team))
  (printout t "There is a machine in " ?zn " with orientation " ?orientation  " so block " ?zn2 crlf)
)


(defrule exp-found-tag
" If a tag was found in a zone that we dont have any information of, update the corresponding tag-vis fact
"
  (goal (class EXPLORATION) (mode DISPATCHED) (meta assigned-to ?r))
  (domain-fact (name tag-matching) (param-values ?machine ?side ?col ?tag))
  (TagVisionInterface (id ?tag-vis-id &:(eq ?tag-vis-id (remote-if-id ?r "/tag-vision/info")))
    (tags_visible ?num-tags&:(> ?num-tags 0))
    (tag_id $?tag-ids&:(member$ ?tag ?tag-ids))
  )
  (Position3DInterface
    (id ?tag-if-id&:(eq ?tag-if-id (str-cat (remote-if-id ?r (str-cat " tag-vision/" (- (member$ ?tag ?tag-ids) 1))))))
    (visibility_history ?vh&:(> ?vh 1))
    (translation $?trans) (rotation $?rot)
    (frame ?frame) (time $?timestamp)
  )
  (exp-zone-margin ?zone-margin)
  ?ze-f <- (wm-fact (key exploration fact tag-vis args? zone ?zn&:(eq ?zn (get-zone ?zone-margin
                                                      (transform-safe "map" ?frame ?timestamp ?trans ?rot)))) (value ?tv&:(< ?tv 1) ))
  (wm-fact (key exploration zone ?zn args? machine UNKNOWN team ?))
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
  (goal (id ?goal-id) (class EXPLORATION) (mode DISPATCHED) (meta assigned-to ?r))
  (Position3DInterface (id ?pos-id&:(eq ?pos-id (remote-if-id ?r "Pose"))) (translation $?trans))
  ?ze <- (wm-fact (key exploration fact time-searched args? zone ?zn) (value ?ts&:(<= ?ts ?*EXP-SEARCH-LIMIT*)))
  (wm-fact (key exploration zone ?zn args? machine UNKNOWN team ?team))
  (wm-fact (key exploration fact line-vis args? zone ?zn) (value ?vh))
  (wm-fact (key exploration fact tag-vis args? zone ?zn) (value ?tv))

  ; Either a tag or a line was found in this zone"
  (test (or (> ?tv 0) (> ?vh 0)))

  ; Since the finding of a tag is a stronger indicator of a machine than a line, we prefer zones were a tag was found
  ; Either tag-vis is greater 0 or there is no zone with tag-vis greater than 0 and a line was found in this zone
  (or (test (> ?tv 0))
      (not (and (wm-fact (key exploration fact tag-vis args? zone ?zn2) (value ?tv2&:(> ?tv2 0)))
		            (wm-fact (key exploration fact time-searched args? zone ?zn2) (value ?ts2&:(<= ?ts2 ?*EXP-SEARCH-LIMIT*)))
		            (wm-fact (key exploration zone ?zn2 args? machine UNKNOWN team ?team2))
	              (not (exploration-result (zone ?zn2)))
	         )
      )
  )

  ; Check that there is no zone with the same indicator of a machine (tag or line) that is closer
  (not (and (wm-fact (key exploration fact line-vis args? zone ?zn3&:(< (distance-mf (zone-center ?zn3) ?trans) (distance-mf (zone-center ?zn) ?trans))) (value ?vh3& : (not (and (= ?vh3 0) (= ?tv 0)))))
	          (wm-fact (key exploration fact tag-vis args? zone ?zn3) (value ?tv3& : (not (and (> ?tv 0) (= ?tv3 0)))))
	          (wm-fact (key exploration fact time-searched  args? zone ?zn3) (value ?ts3&:(<= ?ts3 ?*EXP-SEARCH-LIMIT*)))
	          (wm-fact (key exploration zone ?zn3 args? machine UNKNOWN team ?team3))
            (not (exploration-result (zone ?zn3)))
	     )
  )

  (plan (id ?plan-id&EXPLORATION-PLAN) (goal-id ?goal-id))
  (not (plan (id EXPLORE-ZONE)))

  (plan-action (id ?action-id) (action-name move-node) (plan-id ?plan-id) (state RUNNING))

  ; Only start interrupting the EXPLORATION-PLAN if the first move action was finished.
  ; This prohibits, that all bots start exploring zones right in front of the insertion zone
  (plan-action (id ?action-id2) (action-name ?action-name2) (plan-id ?plan-id) (state FINAL|FAILED))

  ?skill <- (skill (id ?skill-id) (name ?action-name) (status S_RUNNING))
  (not (exploration-result (zone ?zn)))

  (exp-navigator-vmax ?r ?vel ?rot)
  =>
  (navigator-set-speed ?r ?vel ?rot)
  (bind ?new-ts (+ 1 ?ts))
  (modify ?ze (value ?new-ts))
  (modify ?skill (status S_FAILED))
  (printout t "EXP formulating zone exploration plan " ?zn " with line: " ?vh " and tag: " ?tv crlf)
  (assert
    (plan (id EXPLORE-ZONE) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id EXPLORE-ZONE) (goal-id ?goal-id) (action-name explore-zone) (param-names r z) (param-values ?r ?zn))
    (plan-action (id 2) (plan-id EXPLORE-ZONE) (goal-id ?goal-id) (action-name evaluation))
  )
)


(defrule exp-increase-search-limit
" There are zones with tag or line findings, but the search limit is reached
  Then the search limit is incremented, to enable reexploration as a fallback solution
"
  (goal (class EXPLORATION) (mode DISPATCHED))

  (wm-fact (key exploration fact line-vis args? zone ?zn1) (value ?vh))
  (wm-fact (key exploration fact tag-vis args? zone ?zn1) (value ?tv))
  (wm-fact (key exploration zone ?zn1 args machine UNKNOWN team ?team))
  (test (or (> ?tv 0) (> ?vh 0)))

  (not (and
	       (wm-fact (key exploration fact line-vis args? zone ?zn2) (value ?vh-tmp))
	       (wm-fact (key exploration fact tag-vis args? zone ?zn2) (value ?tv-tmp))
         (wm-fact (key exploration fact time-searched args? zone ?zn2) (value ?ts&:(<= ?ts ?*EXP-SEARCH-LIMIT*)))
	       (wm-fact (key exploration zone ?zn2 args? machine UNKNOWN team ?team2))
	       (test (or (> ?vh-tmp 0) (> ?tv-tmp 0)))
	     )
	)
=>
  (modify ?*EXP-SEARCH-LIMIT* (+ ?*EXP-SEARCH-LIMIT* 1))
)


(defrule exp-skill-explore-zone-final
" Exploration of a zone finished succesfully. Update zone wm-fact and assert exploration-result
"
  (goal (id ?goal-id) (class EXPLORATION) (mode DISPATCHED) (meta assigned-to ?r))
  (plan-action (action-name explore-zone) (state FINAL))
  ?pa <- (plan-action (action-name evaluation) (goal-id ?goal-id)
                      (plan-id EXPLORE-ZONE) (state PENDING))
  (ZoneInterface (id ?zone-id&:(eq ?zone-id (remote-if-id ?r "explore-zone/info")))
	               (zone ?zn-str) (orientation ?orientation)
	               (tag_id ?tag-id) (search_state YES)
  )
  (domain-fact (name tag-matching) (param-values ?machine ?side ?team-color ?tag-id))
  (domain-fact (name mps-type) (param-values ?machine ?mtype))

  ?ze <- (wm-fact (key exploration fact time-searched args? zone ?zn2&:(eq ?zn2 (sym-cat ?zn-str))) (value ?times-searched))
  ?zm <- (wm-fact (key exploration zone ?zn2 args? machine ? team ?team))
  ?zm2 <- (wm-fact (key exploration zone ?zn3&:(eq (mirror-name ?zn2) ?zn3) args? machine ? team ?team2))

  (not (exploration-result (machine ?machine) (zone ?zn2)))
  (exp-navigator-vlow ?r ?vel ?rot)
  =>
  (navigator-set-speed ?r ?vel ?rot)
  (modify ?pa (state FINAL))
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
  (printout t "EXP exploration fact zone successfull. Found " ?machine " in " ?zn2 crlf)

)


(defrule exp-skill-explore-zone-failed
" Exploration of a zone failed. Simply set the evaluation action to final to continue"
  (plan-action (action-name explore-zone) (plan-id ?plan-id) (goal-id ?goal-id)
	             (state FAILED))
  ?p <- (plan-action (action-name evaluation) (state PENDING)
	                   (plan-id ?plan-id) (goal-id ?goal-id))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(goal (id ?goal-id) (mode DISPATCHED) (meta assigned-to ?r))
  (exp-navigator-vlow ?r ?vel ?rot)
  =>
  (navigator-set-speed ?r ?vel ?rot)
  (printout t "EXP exploration fact zone fail, nothing to do for evaluation" crlf)
  (modify ?p (state FINAL))
)


(defrule exp-report-to-refbox
" Regularly send all found machines to the refbox"
  (goal (class EXPLORATION) (mode DISPATCHED))
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


(defrule exp-exploration-ends-cleanup
" As soon as the game phase switches to production, set the exploration goal to finished
  and reset the velocity

"
  ?g <- (goal (class EXPLORATION) (mode DISPATCHED) (meta assigned-to ?r))
  (wm-fact (key refbox phase) (value PRODUCTION))
  (wm-fact (key game state) (value RUNNING))
  (wm-fact (id "/config/rcll/max-velocity") (type FLOAT) (value ?max-velocity))
  (wm-fact (id "/config/rcll/max-rotation") (type FLOAT) (value ?max-rotation))

=>
  (printout t "exploration phase ended, cleaning up" crlf)
  (modify ?g (mode FINISHED) (outcome COMPLETED))
  (navigator-set-speed ?r ?max-velocity ?max-rotation)
)


(defrule refbox-recv-ExploreInfo
" Listen to the reports of other bots to the refbox. Update own facts, if there is new information contained
  This is extremly useful if the syncing of wm-facts does not work. Thus, there is at least no zone explored twice
"
  (declare (salience 1000))
  ?pb-msg <- (protobuf-msg (type "llsf_msgs.MachineReport") (ptr ?p))
  (wm-fact (id "/refbox/team-color") (value ?team-name&:(neq ?team-name nil)))
  =>
  (foreach ?m (pb-field-list ?p "machines")

    (bind ?m-name (sym-cat (pb-field-value ?m "name")))
    (bind ?m-zone (sym-cat (pb-field-value ?m "zone")))
    (bind ?m-rotation (eval (sym-cat (pb-field-value ?m "rotation"))))
    (if (not (any-factp ((?er exploration-result)) (eq ?m-name ?er:machine) ))
      then
	(bind ?m-type (get-mps-type-from-name ?m-name))
        (assert (exploration-result (machine ?m-name) (zone ?m-zone) (orientation ?m-rotation) (team ?team-name )))
	(assert (exploration-result
			(machine (mirror-name ?m-name)) (zone (mirror-name ?m-zone))
			(orientation (mirror-orientation ?m-type ?m-zone ?m-rotation))
			(team (mirror-team ?team-name))))
   )
  )
)
