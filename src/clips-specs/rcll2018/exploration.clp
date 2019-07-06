
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
               (wm-fact (key exploration fact time-searched args? zone ?zone) (value 0) (type INT) (is-list FALSE) )
       )
     )
     (foreach ?zone ?Mzones
       (assert (wm-fact (key exploration fact line-vis args? zone ?zone) (value 0) (type INT) (is-list FALSE) )
               (wm-fact (key exploration fact time-searched args? zone ?zone) (value 0) (type INT) (is-list FALSE) ))
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
  (assert (exp-navigator-vmax ?max-velocity ?max-rotation))
  (assert (exp-navigator-vlow ?low-velocity ?low-rotation))
)


(defrule exp-create-exploration-goal
" Initial goal creating
  Refer to fixed-squence.clp for the expandation of the goal and the creation of the EXPLORATION-PLAN
  The EXPLORATION-PLAN let the robot visit a number of configurable points. If a possible machine was detected, this plan is interrupted
"
  (wm-fact (key refbox phase) (value EXPLORATION))
  (not (goal (id ?goal-id) (class EXPLORATION)))
  (wm-fact (key domain fact self args? r ?r))

  ?cv <- (wm-fact (id "/config/rcll/exploration/zone-margin") (type FLOAT) (value ?zone-margin))
  (exp-navigator-vlow ?vel ?rot)
  =>
  (assert (exp-zone-margin ?zone-margin))
  (assert (goal (id (sym-cat EXPLORATION- (gensym*))) (class EXPLORATION)
                (type ACHIEVE) (sub-type SIMPLE)))
  "Lower the speed, for a more robust line and tag detection"
  (navigator-set-speed ?vel ?rot)
)


(defrule exp-passed-through-quadrant
" If the robot drove through a zone slow enough and passed the middle of the zone with a certain margin
  we can conclude, that there is no machine in this zone
"
  (goal (class EXPLORATION) (mode DISPATCHED))
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

(defrule exp-start-zone-exploring
" If there is a zone, where we suspect a machine, interrupt the EXPLORATION-PLAN and start exploring the zone
"
  (goal (id ?goal-id) (class EXPLORATION) (mode DISPATCHED))
  (wm-fact (key domain fact self args? r ?r))
  (Position3DInterface (id "Pose") (translation $?trans))
  ?ze <- (wm-fact (key exploration fact time-searched args? zone ?zn) (value ?ts&:(<= ?ts ?*EXP-SEARCH-LIMIT*)))
  (wm-fact (key exploration fact line-vis args? zone ?zn) (value ?vh&:(> ?vh 0)))

  (not (wm-fact (key exploration fact line-vis args? zone ?zn3&:(< (distance-mf (zone-center ?zn3) ?trans) (distance-mf (zone-center ?zn) ?trans))) (value ?vh3&: (> ?vh3 0))))

  (plan (id ?plan-id&EXPLORATION-PLAN) (goal-id ?goal-id))
  (not (plan (id EXPLORE-ZONE)))

  (plan-action (id ?action-id) (action-name move-node) (plan-id ?plan-id) (state RUNNING))

  ?skill <- (skill (id ?skill-id) (name ?action-name) (status S_RUNNING))
  (not (exploration-result (zone ?zn)))

  (exp-navigator-vmax ?vel ?rot)
  =>
  (navigator-set-speed ?vel ?rot)
  (bind ?new-ts (+ 1 ?ts))
  (modify ?ze (value ?new-ts))
  (modify ?skill (status S_FAILED))
  (printout t "EXP formulating zone exploration plan " ?zn " with line: " ?vh crlf)
  (assert
    (plan (id EXPLORE-ZONE) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id EXPLORE-ZONE) (goal-id ?goal-id) (action-name explore-zone) (param-names r z) (param-values ?r ?zn))
    (plan-action (id 2) (plan-id EXPLORE-ZONE) (goal-id ?goal-id) (action-name evaluation) (param-names z) (param-values ?zn))
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
  (goal (id ?goal-id) (class EXPLORATION) (mode DISPATCHED))
  (plan-action (action-name explore-zone) (state FINAL))
  ?pa <- (plan-action (action-name evaluation) (goal-id ?goal-id)
                      (plan-id EXPLORE-ZONE) (state PENDING) (param-values ?zn-str))

  ?ze <- (wm-fact (key exploration fact time-searched args? zone ?zn2&:(eq ?zn2 (sym-cat ?zn-str))) (value ?times-searched))

  (not (exploration-result (zone ?zn2)))
  (exp-navigator-vlow ?vel ?rot)
  =>
  (navigator-set-speed ?vel ?rot)
  (modify ?pa (state FINAL))
  (modify ?ze (value (+ 1 ?times-searched)))
  (assert
    (exploration-result
      (zone ?zn2)
    )
  )
  (printout t "EXP exploration fact zone successfull" ?zn2 crlf)

)


(defrule exp-skill-explore-zone-failed
" Exploration of a zone failed. Simply set the evaluation action to final to continue"
  (plan-action (action-name explore-zone) (state FAILED))
  ?p <- (plan-action (action-name evaluation) (state PENDING))
  (exp-navigator-vlow ?vel ?rot)
  =>
  (navigator-set-speed ?vel ?rot)
  (printout t "EXP exploration fact zone fail, nothing to do for evaluation" crlf)
  (modify ?p (state FINAL))
)


(defrule exp-exploration-ends-cleanup
" As soon as the game phase switches to production, set the exploration goal to finished 
  and reset the velocity

"
  ?g <- (goal (class EXPLORATION) (mode DISPATCHED))
  (wm-fact (key refbox phase) (type UNKNOWN) (value PRODUCTION))
  (wm-fact (key game state) (type UNKNOWN) (value RUNNING))
  (wm-fact (id "/config/rcll/max-velocity") (type FLOAT) (value ?max-velocity))
  (wm-fact (id "/config/rcll/max-rotation") (type FLOAT) (value ?max-rotation))

=>
  (printout t "exploration phase ended, cleaning up" crlf)
  (modify ?g (mode FINISHED) (outcome COMPLETED))
  (navigator-set-speed ?max-velocity ?max-rotation)
)


