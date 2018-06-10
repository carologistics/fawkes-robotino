
;---------------------------------------------------------------------------
;  exploration.clp - Robotino agent for exploration phase
;
;  Copyright  2017 Victor Mataré
;
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

;priorities.clp and gloabal.clp
(defglobal
  ?*PRIORITY-WM*      =  200
  ?*EXP-ROUTE-IDX* = 1
  ?*EXP-MOVEMENT-COMPENSATION* = 0.0
  ?*EXP-SEARCH-LIMIT* = 1
)

; EXPLORATION
(deftemplate tried-lock
  (slot resource (type SYMBOL))
  (slot result (type SYMBOL) (allowed-symbols ACCEPT REJECT))
)

(deftemplate exploration-result
  (slot machine (type SYMBOL) (allowed-symbols C-BS C-CS1 C-CS2 C-RS1 C-RS2 C-DS C-SS M-BS M-CS1 M-CS2 M-RS1 M-RS2 M-DS M-SS))
  (slot zone (type SYMBOL)
    (allowed-symbols
      M-Z78 M-Z68 M-Z58 M-Z48 M-Z38 M-Z28 M-Z18
      M-Z77 M-Z67 M-Z57 M-Z47 M-Z37 M-Z27 M-Z17
      M-Z76 M-Z66 M-Z56 M-Z46 M-Z36 M-Z26 M-Z16
      M-Z75 M-Z65 M-Z55 M-Z45 M-Z35 M-Z25 M-Z15
      M-Z74 M-Z64 M-Z54 M-Z44 M-Z34 M-Z24 M-Z14
      M-Z73 M-Z63 M-Z53 M-Z43 M-Z33 M-Z23 M-Z13
      M-Z72 M-Z62 M-Z52 M-Z42 M-Z32 M-Z22 M-Z12
                        M-Z41 M-Z31 M-Z21 M-Z11

      C-Z18 C-Z28 C-Z38 C-Z48 C-Z58 C-Z68 C-Z78
      C-Z17 C-Z27 C-Z37 C-Z47 C-Z57 C-Z67 C-Z77
      C-Z16 C-Z26 C-Z36 C-Z46 C-Z56 C-Z66 C-Z76
      C-Z15 C-Z25 C-Z35 C-Z45 C-Z55 C-Z65 C-Z75
      C-Z14 C-Z24 C-Z34 C-Z44 C-Z54 C-Z64 C-Z74
      C-Z13 C-Z23 C-Z33 C-Z43 C-Z53 C-Z63 C-Z73
      C-Z12 C-Z22 C-Z32 C-Z42 C-Z52 C-Z62 C-Z72
      C-Z11 C-Z21 C-Z31 C-Z41
    )
  )
  (slot orientation (type INTEGER) (default -1))
  (slot team (type SYMBOL) (allowed-symbols CYAN MAGENTA))
)

(deftemplate found-tag
  (slot name (type SYMBOL))
  (slot side (type SYMBOL) (allowed-values INPUT OUTPUT))
  (slot frame (type STRING))
  (multislot trans (type FLOAT) (cardinality 3 3))
  (multislot rot (type FLOAT) (cardinality 4 4))
  (slot sync-id (type INTEGER) (default 0))
)

(defrule startup-exploration
    (not (wm-fact (key exploration zone ?zn args? machine ?machine team ?team)))
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

    (bind $?Mzones (create$ 	  M-Z78 M-Z68 M-Z58 M-Z48 M-Z38 M-Z28 M-Z18
      M-Z77 M-Z67 M-Z57 M-Z47 M-Z37 M-Z27 M-Z17
      M-Z76 M-Z66 M-Z56 M-Z46 M-Z36 M-Z26 M-Z16
      M-Z75 M-Z65 M-Z55 M-Z45 M-Z35 M-Z25 M-Z15
      M-Z74 M-Z64 M-Z54 M-Z44 M-Z34 M-Z24 M-Z14
      M-Z73 M-Z63 M-Z53 M-Z43 M-Z33 M-Z23 M-Z13
      M-Z72 M-Z62 M-Z52 M-Z42 M-Z32 M-Z22 M-Z12
                        M-Z41 M-Z31 M-Z21 M-Z11))

     (foreach ?zone ?Czones
	(assert (wm-fact (key exploration fact line-vis args? zone ?zone) (value 0) (is-list FALSE) )
       		(wm-fact (key exploration fact tag-vis args? zone ?zone) (value 0) (is-list FALSE) )
    		(wm-fact (key exploration fact time-searched args? zone ?zone) (value 0) (is-list FALSE) )
		(wm-fact (key exploration zone ?zone args? machine UNKNOWN team CYAN))
	)
     )
     (foreach ?zone ?Mzones
	(assert (wm-fact (key exploration fact line-vis args? zone ?zone) (value 0) (is-list FALSE) )
       		(wm-fact (key exploration fact tag-vis args? zone ?zone) (value 0) (is-list FALSE) )
    		(wm-fact (key exploration fact time-searched args? zone ?zone) (value 0) (is-list FALSE) )
		(wm-fact (key exploration zone ?zone args? machine UNKNOWN team MAGENTA)))
     )

)

(defrule conf-get-exp-vmax
  (wm-fact (id "/config/rcll/exploration/max-velocity") (type FLOAT) (value ?max-velocity))
  (wm-fact (id "/config/rcll/exploration/max-rotation") (type FLOAT) (value ?max-rotation))
  =>
  (assert (exp-navigator-vmax ?max-velocity ?max-rotation))
)

(defrule goal-reasoner-create-exploration-goal
  (not (goal (id ?goal-id) (class EXPLORATION)))
  (wm-fact (key domain fact entered-field args? r ?r))
  (wm-fact (key domain fact self args? r ?r))
  (wm-fact (key refbox phase) (type UNKNOWN) (value EXPLORATION))
  (wm-fact (key game state) (type UNKNOWN) (value RUNNING))
  ?cv <- (wm-fact (id "/config/rcll/exploration/zone-margin") (type FLOAT) (value ?zone-margin))
  =>
  (assert (exp-zone-margin ?zone-margin))
  (assert (timer (name send-machine-reports)))
  (assert (goal (id (sym-cat EXPLORATION- (gensym*))) (class EXPLORATION)
                (type ACHIEVE)))
)

(defrule exp-passed-through-quadrant
  "We're driving slowly through a certain quadrant: reason enough to believe there's no machine here."
  (goal (class EXPLORATION) (mode DISPATCHED))
  (exp-navigator-vmax ?max-velocity ?max-rotation)
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
  )
)

(defrule exp-found-line
  "Found a line that is within an unexplored zone."
  (goal (class EXPLORATION) (mode DISPATCHED))
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


(defrule exp-found-tag
  (goal (class EXPLORATION) (mode DISPATCHED))
  (domain-fact (name tag-matching) (param-values ?machine ?side ?col ?tag))
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
  ?ze-f <- (wm-fact (key exploration fact tag-vis args? zone ?zn&:(eq ?zn (get-zone ?zone-margin
                                                      (transform-safe "map" ?frame ?timestamp ?trans ?rot)))) (value ?tv&:(< ?tv 1) ))
=>
  (modify ?ze-f (value 1 ))
  (printout t "Found tag in " ?zn crlf)
)


(defrule exp-try-locking-line
  (goal (id ?goal-id) (class EXPLORATION) (mode DISPATCHED))
  (wm-fact (key domain fact self args? r ?r))
  (Position3DInterface (id "Pose") (translation $?trans))
  ?ze <- (wm-fact (key exploration fact time-searched args? zone ?zn) (value ?ts&:(<= ?ts ?*EXP-SEARCH-LIMIT*)))
  (wm-fact (key exploration zone ?zn args? machine UNKNOWN team ?team))
  (wm-fact (key exploration fact line-vis args? zone ?zn) (value ?vh))
  (wm-fact (key exploration fact tag-vis args? zone ?zn) (value ?tv))
  (test (or (> ?tv 0) (> ?vh 0)))

  (or (test (> ?tv 0))
      (not (and (wm-fact (key exploration fact tag-vis args? zone ?zn2) (value ?tv2&:(> ?tv2 0)))
		(wm-fact (key exploration fact time-searched args? zone ?zn2) (value ?ts2&:(<= ?ts2 ?*EXP-SEARCH-LIMIT*)))
		(wm-fact (key exploration zone ?zn2 args? machine UNKNOWN team ?team2))
	        (not (exploration-result (zone ?zn2)))
	   )
      )
  )
  (not (and (wm-fact (key exploration fact line-vis args? zone ?zn3&:(< (distance-mf (zone-center ?zn3) ?trans) (distance-mf (zone-center ?zn) ?trans))) (value ?vh3& : (not (and (= ?vh3 0) (= ?tv 0)))))
	    (wm-fact (key exploration fact tag-vis args? zone ?zn3) (value ?tv3& : (not (and (> ?tv 0) (= ?tv3 0)))))
	    (wm-fact (key exploration fact time-searched  args? zone ?zn3) (value ?ts3&:(<= ?ts3 ?*EXP-SEARCH-LIMIT*)))
	    (wm-fact (key exploration zone ?zn3 args? machine UNKNOWN team ?team3))
            (not (exploration-result (zone ?zn3)))
	)
  )

  (not (tried-lock (resource ?zn)))
  (plan (id ?plan-id&EXPLORATION-PLAN) (goal-id ?goal-id))
  (not (plan (id EXPLORE-ZONE)))

  (plan-action (id ?action-id) (action-name move-node) (plan-id ?plan-id) (status RUNNING))
  (plan-action (id ?action-id2) (action-name ?action-name2) (plan-id ?plan-id) (status FINAL|FAILED))

  ?skill <- (skill (id ?skill-id) (name ?action-name) (status S_RUNNING))
  (not (exploration-result (zone ?zn)))

  =>
  (bind ?new-ts (+ 1 ?ts))
  (assert (tried-lock (resource ?zn) (result REJECT)))
  (modify ?ze (value ?new-ts))
  (modify ?skill (status S_FAILED))
  (printout t "EXP formulating zone exploration plan " ?zn " with line: " ?vh " and tag: " ?tv crlf)
  (assert
    (plan (id EXPLORE-ZONE) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id EXPLORE-ZONE) (goal-id ?goal-id) (action-name one-time-lock) (param-names name) (param-values ?zn))
    (plan-action (id 2) (plan-id EXPLORE-ZONE) (goal-id ?goal-id) (action-name stop) (param-names r) (param-values ?r))
    (plan-action (id 3) (plan-id EXPLORE-ZONE) (goal-id ?goal-id) (action-name explore-zone) (param-names r z) (param-values ?r ?zn))
    (plan-action (id 4) (plan-id EXPLORE-ZONE) (goal-id ?goal-id) (action-name evaluation))
    (plan-action (id 5) (plan-id EXPLORE-ZONE) (goal-id ?goal-id) (action-name unlock) (param-names name) (param-values ?zn) (executable TRUE))
  )
)


(defrule exp-increase-search-limit
  (goal (class EXPLORATION) (mode DISPATCHED))

  ; There is an explorable zone...
  (wm-fact (key exploration fact line-vis args? zone ?zn1) (value ?vh))
  (wm-fact (key exploration fact tag-vis args? zone ?zn1) (value ?tv))
  (wm-fact (key exploration zone ?zn1 args machine UNKNOWN team ?team))
  (test (or (> ?tv 0) (> ?vh 0)))

  ; ... but no zone may be searched according to the repeated-search-limit
  (not (and
	(wm-fact (key exploration fact line-vis args? zone ?zn2) (value ?vh-tmp)) 
	(wm-fact (key exploration fact tag-vis args? zone ?zn2) (value ?tv-tmp))
        (wm-fact (key exploration fact time-searched args? zone ?zn2) (value ?ts&:(<= ?ts ?*EXP-SEARCH-LIMIT*)))
	(wm-fact (key exploration zone ?zn2 args? machine UNKNOWN team ?team2))
	(test (or (> ?vh-tmp 0) (> ?tv-tmp 0)))))
=>
  (modify ?*EXP-SEARCH-LIMIT* (+ ?*EXP-SEARCH-LIMIT* 1))
)

(defrule exp-skill-explore-zone-final
  (goal (id ?goal-id) (class EXPLORATION) (mode DISPATCHED))
  (plan-action (action-name explore-zone) (status FINAL))
  ?pa <- (plan-action (action-name evaluation) (goal-id ?goal-id)
                      (plan-id EXPLORE-ZONE) (status PENDING))
  (ZoneInterface (id "/explore-zone/info") (zone ?zn-str)
    (orientation ?orientation) (tag_id ?tag-id) (search_state YES)
  )
  (Position3DInterface (id "/explore-zone/found-tag")
    (frame ?frame) (translation $?trans) (rotation $?rot)
  )
  ?lock <- (tried-lock (resource ?zn-sym&:(eq ?zn-sym (sym-cat ?zn-str))))
  (domain-fact (name tag-matching) (param-values ?machine ?side ?team-color ?tag-id))

  ?ze <- (wm-fact (key exploration fact time-searched args? zone ?zn2&:(eq ?zn2 (sym-cat ?zn-str))) (value ?times-searched))
  ?zm <- (wm-fact (key exploration zone ?zn2 args? machine UNKNOWN team ?team2))
  (domain-fact (name mps-type) (param-values ?machine ?mtype))
  (not (exploration-result (machine ?machine) (zone ?zn2)))
  =>
  (modify ?lock (result ACCEPT))
  (modify ?pa (status FINAL))
  (if (any-factp ((?ft found-tag)) (eq ?ft:name ?machine)) then
    (printout error "BUG: Tag for " ?machine " already found. Locking glitch or agent bug!" crlf)
  else
    (modify ?ze (value (+ 1 ?times-searched)))
    (modify ?zm (key exploration zone ?zn2 args? machine ?machine team ?team2))
    (assert (found-tag (name ?machine) (side ?side) (frame "map") (trans ?trans) (rot ?rot)))
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
)

(defrule exp-skill-explore-zone-failed
  (plan-action (action-name explore-zone) (status FAILED))
  ?p <- (plan-action (action-name evaluation) (status PENDING))
  =>
  (printout t "EXP exploration fact zone fail, nothing to do for evaluation" crlf)
  (modify ?p (status FINAL))
)

(defrule exp-report-to-refbox
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
  "Clean up lock refusal facts when exploration ends"
  ?g <- (goal (class EXPLORATION) (mode DISPATCHED))
  (wm-fact (key refbox phase) (type UNKNOWN) (value PRODUCTION))
  (wm-fact (key game state) (type UNKNOWN) (value RUNNING))
=>
  (delayed-do-for-all-facts ((?l tried-lock)) (eq ?l:result REJECT)
    (retract ?l)
  )
  (printout t "exploration phase ended, cleaning up" crlf)
  (modify ?g (mode FINISHED) (outcome COMPLETED))
)

(deffunction get-mps-type-from-name (?mps)
  (bind ?type (sym-cat (sub-string 3 4 (str-cat ?mps))))
  (return ?type)
)

(defrule refbox-recv-ExploreInfo
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
