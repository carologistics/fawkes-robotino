
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

(defrule exploration-set-speed-action-high
  ?pa <- (plan-action (action-name set-speed-high) (state PENDING))
  (exp-navigator-vmax ?vmax ?rmax)
  =>
  (navigator-set-speed ?vmax ?rmax)
  (modify ?pa (state EXECUTION-SUCCEEDED))
)

(defrule exploration-set-speed-action-low
  ?pa <- (plan-action (action-name set-speed-low) (state PENDING))
  (exp-navigator-vlow ?vlow ?rlow)
  =>
  (navigator-set-speed ?vlow ?rlow)
  (modify ?pa (state EXECUTION-SUCCEEDED))
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
    (wm-fact (key refbox phase) (value EXPLORATION|SETUP))
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

(defrule exp-conf-init-exploration
" Reads maximum values for rotating and velocity from the config and stores it as a fact
"
  (wm-fact (id "/config/rcll/exploration/low-velocity") (type FLOAT) (value ?low-velocity))
  (wm-fact (id "/config/rcll/exploration/low-rotation") (type FLOAT) (value ?low-rotation))
  (wm-fact (id "/config/rcll/exploration/max-velocity") (type FLOAT) (value ?max-velocity))
  (wm-fact (id "/config/rcll/exploration/max-rotation") (type FLOAT) (value ?max-rotation))

  (wm-fact (id "/config/rcll/exploration/zone-margin") (type FLOAT) (value ?zone-margin))
  =>
  (assert (exp-navigator-vmax ?max-velocity ?max-rotation))
  (assert (exp-navigator-vlow ?low-velocity ?low-rotation))

  (assert (exp-zone-margin ?zone-margin))
  (assert (timer (name send-machine-reports)))
)

(defrule exp-passed-through-quadrant
" If the robot drove through a zone slow enough and passed the middle of the zone with a certain margin
  we can conclude, that there is no machine in this zone
"
  (wm-fact (key refbox phase) (type UNKNOWN) (value EXPLORATION))
  (exp-navigator-vlow ?max-velocity ?max-rotation)
  (MotorInterface (id "Robotino")
    (vx ?vx&:(< ?vx ?max-velocity)) (vy ?vy&:(< ?vy ?max-velocity)) (omega ?w&:(< ?w ?max-rotation))
  )
  (Position3DInterface (id "Pose") (translation $?trans) (time $?timestamp) (visibility_history ?vh&:(>= ?vh 10)))
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
  (wm-fact (key refbox phase) (type UNKNOWN) (value EXPLORATION))
  (LaserLineInterface
    (visibility_history ?vh&:(>= ?vh 1))
    (time $?timestamp)
    (end_point_1 $?ep1)
    (end_point_2 $?ep2)
    (frame_id ?frame)
  )
  (MotorInterface (id "Robotino") (vx ?vx) (vy ?vy))
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
  (wm-fact (key refbox phase) (type UNKNOWN) (value EXPLORATION))
  (domain-fact (name tag-matching) (param-values ?machine ?side ?col ?tag))
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
  ?ze-f <- (wm-fact (key exploration fact tag-vis args? zone ?zn&:(eq ?zn (get-zone ?zone-margin
                                                      (transform-safe "map" ?frame ?timestamp ?trans ?rot)))) (value ?tv&:(< ?tv 1) ))
  (wm-fact (key exploration zone ?zn args? machine UNKNOWN team ?))
=>
  (modify ?ze-f (value 1 ))
  (printout t "Found tag in " ?zn crlf)
)

(defrule exp-increase-search-limit
" There are zones with tag or line findings, but the search limit is reached
  Then the search limit is incremented, to enable reexploration as a fallback solution
"
  (wm-fact (key refbox phase) (type UNKNOWN) (value EXPLORATION))

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

(defrule exp-report-to-refbox
" Regularly send all found machines to the refbox"
  (wm-fact (key refbox phase) (type UNKNOWN) (value EXPLORATION))
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
  ?g <- (goal (class EXPLORE-ZONE|MOVE-NODE) (mode DISPATCHED))
  (wm-fact (key refbox phase) (type UNKNOWN) (value PRODUCTION))
  (wm-fact (key game state) (type UNKNOWN) (value RUNNING))
  (wm-fact (id "/config/rcll/max-velocity") (type FLOAT) (value ?max-velocity))
  (wm-fact (id "/config/rcll/max-rotation") (type FLOAT) (value ?max-rotation))

=>
  (printout t "exploration phase ended, cleaning up" crlf)
  (modify ?g (mode FINISHED) (outcome FAILED))
  (navigator-set-speed ?max-velocity ?max-rotation)
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
