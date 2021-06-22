
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

(defrule exp-sync-ground-truth
" When the RefBox sends ground-truth of a zone, update the corresponding
  domain fact. But only do so for ground-truth of the own team, this allows to
  deal with halve-fields, where only one set of MPS is present but ground truth
  for all machines is sent.
"
	(wm-fact (key refbox team-color) (value ?team-color))
	(wm-fact (key refbox field-ground-truth zone args? m ?name&:(eq
	  (sub-string 1 1 ?name) (sub-string 1 1 ?team-color))) (value ?zone))
	?zc <- (wm-fact (key domain fact zone-content args? z ?zone m UNKNOWN))
	=>
	(modify ?zc (key domain fact zone-content args? z ?zone m ?name))
)

(defrule exp-enable
" Exploration is needed as we received an mps-state already without knowing
  the zone."
	(wm-fact (key domain fact mps-state args? m ?name $?))
	(not (wm-fact (key domain fact zone-content args? z ? m ?name)))
	(not (wm-fact (key exploration active) (type BOOL) (value TRUE)))
	=>
	(assert (wm-fact (key exploration active) (type BOOL) (value TRUE)))
)

(defrule exp-disable
" Exploration is not needed anymore as all machines were found."
	(wm-fact (key domain fact mps-state args? m ? $?))
	(forall
		(wm-fact (key domain fact mps-state args? m ?name $?))
		(wm-fact (key domain fact zone-content args? z ? m ?name))
	)
	?active <- (wm-fact (key exploration active) (type BOOL) (value TRUE))
	=>
	(modify ?active (value FALSE))
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
	;(wm-fact (key refbox phase) (value EXPLORATION))
	(wm-fact (key exploration active) (type BOOL) (value TRUE))
=>
	(assert (exp-zone-margin ?zone-margin))
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
             (domain-fact (name zone-content) (param-values ?zone UNKNOWN))
     )
   )
   (foreach ?zone ?Mzones
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
	(wm-fact (key exploration active) (type BOOL) (value TRUE))
	(wm-fact (key central agent robot args? r ?r))
  (exp-navigator-vlow ?r ?max-velocity ?max-rotation)
  (MotorInterface (id ?motor-id &:(eq ?motor-id (remote-if-id ?r "Robotino")))
    (vx ?vx&:(< ?vx ?max-velocity)) (vy ?vy&:(< ?vy ?max-velocity)) (omega ?w&:(< ?w ?max-rotation))
  )
  (Position3DInterface (id ?pose-id&:(eq ?pose-id (remote-if-id ?r "Pose"))) (translation $?trans)
	                     (time $?timestamp) (visibility_history ?vh&:(>= ?vh 10)))
  ?ze <- (wm-fact (key exploration fact time-searched args? zone ?zn&:(eq ?zn (get-zone 0.15 ?trans))) (value ?time-searched))
  ?zm <- (wm-fact (key domain fact zone-content args? z ?zn m UNKNOWN))
=>
  (bind ?zone (get-zone 0.07 ?trans))
  (if ?zone then
    (modify ?ze (key exploration fact time-searched args? zone ?zn) (value (+ 1 ?time-searched)))
    (modify ?zm (key domain fact zone-content args? z ?zn m NONE))
    (printout t "Passed through " ?zn crlf)
  )
)


(defrule exp-found-line
" If a laserline was found, that lies inside a zone with a certain margin,
  update the line-vis fact of this zone
"
	(wm-fact (key exploration active) (type BOOL) (value TRUE))
	(wm-fact (key central agent robot args? r ?r))
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
  ?wm <- (wm-fact (key domain fact zone-content args? z ?zn m NONE))
  ?we <- (wm-fact (key domain fact zone-content args? z ?zn2&:(eq ?zn2 (mirror-name ?zn)) m UNKNOWN))
  =>
  (modify ?we (key domain fact zone-content args? z ?zn m NONE))
  (printout t "Synced zone: " ?zn2 crlf)
)


(defrule exp-exclude-zones
" Mark a zone as empty if there can not be a machine according to the rules
"
  (exploration-result (zone ?zn) (machine ?machine) (orientation ?orientation) )
  ?wm <- (wm-fact (key domain fact zone-content args? z ?zn2 m UNKNOWN))
  (test (eq TRUE (zone-is-blocked ?zn ?orientation ?zn2 ?machine)))
  =>
  (modify ?wm (key domain fact zone-content args? z ?zn m NONE))
  (printout t "There is a machine in " ?zn " with orientation " ?orientation  " so block " ?zn2 crlf)
)


(defrule exp-found-tag
" If a tag was found in a zone that we dont have any information of, update the corresponding tag-vis fact
"
	(wm-fact (key exploration active) (type BOOL) (value TRUE))
	(wm-fact (key central agent robot args? r ?r))
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
	(wm-fact (key exploration active) (type BOOL) (value TRUE))
	(wm-fact (key central agent robot args? r ?r))
  (not (goal (class EXPLORE-ZONE)))
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

  (not (goal (class EXPLORE-ZONE) (params z ?zn)))

  ; Only start interrupting the EXPLORATION-PLAN if the first move action was finished.
  ; This prohibits, that all bots start exploring zones right in front of the insertion zone
  (not (exploration-result (zone ?zn)))
	; TODO: set navigator speed to max when executing the plan
  ;(exp-navigator-vmax ?r ?vel ?rot)
  =>
  (bind ?new-ts (+ 1 ?ts))
  (modify ?ze (value ?new-ts))
  (printout t "Goal EXPLORE-ZONE  formulated in zone: " ?zn " with line: " ?vh " and tag: " ?tv crlf)
  (assert (goal (id (sym-cat EXPLORE-ZONE- (gensym*))) (class EXPLORE-ZONE)
	              (params z ?zn)))
)


(defrule exp-increase-search-limit
" There are zones with tag or line findings, but the search limit is reached
  Then the search limit is incremented, to enable reexploration as a fallback solution
"
  (goal (class EXPLORATION) (mode DISPATCHED))

  (wm-fact (key exploration fact line-vis args? zone ?zn1) (value ?vh))
  (wm-fact (key exploration fact tag-vis args? zone ?zn1) (value ?tv))
  (wm-fact (key domain fact zone-content args? z ?zn1 UNKNOWN))
  (test (or (> ?tv 0) (> ?vh 0)))

  (not (and
	       (wm-fact (key exploration fact line-vis args? zone ?zn2) (value ?vh-tmp))
	       (wm-fact (key exploration fact tag-vis args? zone ?zn2) (value ?tv-tmp))
         (wm-fact (key exploration fact time-searched args? zone ?zn2) (value ?ts&:(<= ?ts ?*EXP-SEARCH-LIMIT*)))
	       (wm-fact (key domain fact zone-content args? z ?zn2 m UNKNOWN))
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
  (plan-action (action-name explore-zone) (state EXECUTION-SUCCEEDED))
  (ZoneInterface (id ?zone-id&:(eq ?zone-id (remote-if-id ?r "explore-zone/info")))
	               (zone ?zn-str) (orientation ?orientation)
	               (tag_id ?tag-id) (search_state YES)
  )
  (domain-fact (name tag-matching) (param-values ?machine ?side ?team-color ?tag-id))
  (domain-fact (name mps-type) (param-values ?machine ?mtype))

  ?ze <- (wm-fact (key exploration fact time-searched args? zone ?zn2&:(eq ?zn2 (sym-cat ?zn-str))) (value ?times-searched))
  ?zm <- (wm-fact (key domain fact zone-content args? z ?zn2 m ?))
  ?zm2 <- (wm-fact (key domain fact zone-content args? z ?zn3&:(eq (mirror-name ?zn2) ?zn3) m ?))

  (not (exploration-result (machine ?machine) (zone ?zn2)))
  =>
  (modify ?ze (value (+ 1 ?times-searched)))
  (modify ?zm (key domain fact zone-content args? z ?zn2 m ?machine))
  (modify ?zm2 (key domain fact zone-content args? z ?zn3 m (mirror-name ?machine)))
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
