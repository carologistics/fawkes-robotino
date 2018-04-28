
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
  (slot result (type SYMBOL) (allowed-values ACCEPT REJECT))
)

; (deftemplate exploration-result
;   (slot machine (type SYMBOL) (allowed-values C-BS C-CS1 C-CS2 C-RS1 C-RS2 C-DS C-SS M-BS M-CS1 M-CS2 M-RS1 M-RS2 M-DS M-SS))
;   (slot zone (type SYMBOL)
;     (allowed-values
;       M-Z78 M-Z68 M-Z58 M-Z48 M-Z38 M-Z28 M-Z18
;       M-Z77 M-Z67 M-Z57 M-Z47 M-Z37 M-Z27 M-Z17
;       M-Z76 M-Z66 M-Z56 M-Z46 M-Z36 M-Z26 M-Z16
;       M-Z75 M-Z65 M-Z55 M-Z45 M-Z35 M-Z25 M-Z15
;       M-Z74 M-Z64 M-Z54 M-Z44 M-Z34 M-Z24 M-Z14
;       M-Z73 M-Z63 M-Z53 M-Z43 M-Z33 M-Z23 M-Z13
;       M-Z72 M-Z62 M-Z52 M-Z42 M-Z32 M-Z22 M-Z12
;                         M-Z41 M-Z31 M-Z21 M-Z11
;
;       C-Z18 C-Z28 C-Z38 C-Z48 C-Z58 C-Z68 C-Z78
;       C-Z17 C-Z27 C-Z37 C-Z47 C-Z57 C-Z67 C-Z77
;       C-Z16 C-Z26 C-Z36 C-Z46 C-Z56 C-Z66 C-Z76
;       C-Z15 C-Z25 C-Z35 C-Z45 C-Z55 C-Z65 C-Z75
;       C-Z14 C-Z24 C-Z34 C-Z44 C-Z54 C-Z64 C-Z74
;       C-Z13 C-Z23 C-Z33 C-Z43 C-Z53 C-Z63 C-Z73
;       C-Z12 C-Z22 C-Z32 C-Z42 C-Z52 C-Z62 C-Z72
;       C-Z11 C-Z21 C-Z31 C-Z41
;     )
;   )
;   (slot orientation (type INTEGER) (default -1))
;   (slot team (type SYMBOL) (allowed-values CYAN MAGENTA))
; )

(deftemplate found-tag
  (slot name (type SYMBOL))
  (slot side (type SYMBOL) (allowed-values INPUT OUTPUT))
  (slot frame (type STRING))
  (multislot trans (type FLOAT) (cardinality 3 3))
  (multislot rot (type FLOAT) (cardinality 4 4))
  (slot sync-id (type INTEGER) (default 0))
)

(defrule startup-exploration
    (not (wm-fact (key exploration zone args? zone ?zn args? ?vis machine ?machine team ?team ?ts)))
    ;(wm-fact (key cx identity) (value "R-1"))
    ;(wm-robmem-sync-conf (wm-fact-key-prefix explore-zone) (enabled TRUE))
=>
  (assert
    (wm-fact (key exploration zone args? zone C-Z11 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z21 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z31 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z41 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z12 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z22 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z32 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z42 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    ; Can't have machines in *-Z52 (see rulebook)...
    ;(wm-fact (key exploration zone args? zone C-Z51 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z62 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z72 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z13 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z23 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z33 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z43 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z53 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z63 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z73 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z62 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z73 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z14 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z24 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z34 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z44 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z54 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z64 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z74 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z64 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z74 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z15 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z25 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z35 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z45 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z55 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z65 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z75 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z65 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z75 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z16 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z26 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z36 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z46 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z56 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z66 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z76 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z66 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z76 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z17 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z27 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z37 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z47 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z57 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z67 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z77 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z67 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z77 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z18 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z28 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z38 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z48 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z58 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z68 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone C-Z78 machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))

    (wm-fact (key exploration zone args? zone M-Z11 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z21 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z31 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z41 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z12 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z22 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z32 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z42 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    ; Can't have machines in *-Z52 (see rulebook)...
    ;(wm-fact (key exploration zone args? zone M-Z51 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z62 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z72 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z13 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z23 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z33 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z43 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z53 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z63 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z73 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z62 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z73 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z14 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z24 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z34 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z44 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z54 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z64 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z74 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z64 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z74 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z15 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z25 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z35 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z45 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z55 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z65 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z75 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z65 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z75 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z16 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z26 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z36 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z46 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z56 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z66 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z76 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z66 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z76 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z17 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z27 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z37 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z47 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z57 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z67 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z77 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z67 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z77 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z18 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z28 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z38 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z48 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z58 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z68 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key exploration zone args? zone M-Z78 machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
  )
)

(defrule conf-get-exp-vmax
  (wm-fact (id "/config/rcll/exploration/max-velocity") (type FLOAT) (value ?max-velocity))
  (wm-fact (id "/config/rcll/exploration/max-rotation") (type FLOAT) (value ?max-rotation))
  =>
  (assert (exp-navigator-vmax ?max-velocity ?max-rotation))
)

(defrule goal-reasoner-create-exploration-goal
  (not (goal (id EXPLORATION)))
  (wm-fact (key domain fact entered-field args? r ?r))
  (wm-fact (key domain fact self args? r ?r))
  (wm-fact (key refbox phase) (type UNKNOWN) (value EXPLORATION))
  (wm-fact (key game state) (type UNKNOWN) (value RUNNING))
  ?cv <- (wm-fact (id "/config/rcll/exploration/zone-margin") (type FLOAT) (value ?zone-margin))
  =>
  (assert (exp-zone-margin ?zone-margin))
  (assert (timer (name send-machine-reports)))
  (assert (goal (id EXPLORATION) (type ACHIEVE)))
)

(defrule exp-passed-through-quadrant
  "We're driving slowly through a certain quadrant: reason enough to believe there's no machine here."
  (goal (id EXPLORATION) (mode DISPATCHED))
  (exp-navigator-vmax ?max-velocity ?max-rotation)
  (MotorInterface (id "Robotino")
    (vx ?vx&:(< ?vx ?max-velocity)) (vy ?vy&:(< ?vy ?max-velocity)) (omega ?w&:(< ?w ?max-rotation))
  )
  (Position3DInterface (id "Pose") (translation $?trans) (time $?timestamp) (visibility_history ?vh&:(>= ?vh 10)))
  ?ze <- (wm-fact (key exploration zone args? zone ?zn&:(eq ?zn (get-zone 0.15 ?trans)) machine UNKNOWN team ?team) (values ?vis_str ?time-searched_str))
=>
  (bind ?zone (get-zone 0.07 ?trans))
  (if ?zone then
    (modify ?ze (key exploration zone args? zone ?zn machine NONE team ?team) (values ?vis_str  (+ 1  ?time-searched_str)))
  )
)

(defrule exp-found-line
  "Found a line that is within an unexplored zone."
  (goal (id EXPLORATION) (mode DISPATCHED))
  (LaserLineInterface
    (visibility_history ?vh&:(>= ?vh 1))
    (time $?timestamp)
    (end_point_1 $?ep1)
    (end_point_2 $?ep2)
    (frame_id ?frame)
  )
  (MotorInterface (id "Robotino") (vx ?vx) (vy ?vy))
  (exp-zone-margin ?zone-margin)
  ?ze-f <- (wm-fact (key exploration zone args? zone ?zn&:(eq ?zn (get-zone ?zone-margin
                                            (compensate-movement
                                              ?*EXP-MOVEMENT-COMPENSATION*
                                              (create$ ?vx ?vy)
                                              (laser-line-center-map ?ep1 ?ep2 ?frame ?timestamp)
                                              ?timestamp)))
                              machine UNKNOWN team ?team) (values ?zn-vh_str&:(< ?zn-vh_str 1) ?ts_str))
=>
  (modify ?ze-f (key exploration zone args? zone ?zn machine UNKNOWN team ?team) (values ?vh ?ts_str))
  (printout warn "EXP found line: " ?zn " vh: " ?vh crlf)
)


(defrule exp-found-tag
  (goal (id EXPLORATION) (mode DISPATCHED))
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
  ?ze-f <- (wm-fact (key exploration zone args? zone ?zn&:(eq ?zn (get-zone ?zone-margin
                                                      (transform-safe "map" ?frame ?timestamp ?trans ?rot)))
                                                  machine UNKNOWN team ?team) (values ?lv_str&:(< ?lv_str 2) ?ts))
=>
  (modify ?ze-f (values (+ ?lv_str 1)  ?ts))
  (printout t "Found tag in " ?zn crlf)
)


(defrule exp-try-locking-line
  (goal (id EXPLORATION) (mode DISPATCHED))
  (wm-fact (key domain fact self args? r ?r))
  (Position3DInterface (id "Pose") (translation $?trans))
  ?ze <- (wm-fact (key exploration zone args? zone ?zn machine ?machine team ?team) (values ?vh&:(> ?vh 0) ?ts_str&:(<= ?ts_str ?*EXP-SEARCH-LIMIT*)))
  (not (wm-fact (key
		exploration zone  args? zone ?zn2&:(< (distance-mf (zone-center ?zn2) ?trans) (distance-mf (zone-center ?zn) ?trans))
		machine ?machine2 team ?team2)
		(values ?vh2&:(> ?vh2 0) ?ts2&:(<= ?ts2 ?*EXP-SEARCH-LIMIT*))))
  (not (tried-lock (resource ?zn)))
  (plan (id ?plan-id&EXPLORATION-PLAN) (goal-id EXPLORATION))
  (not (plan (id EXPLORE-ZONE)))

  (plan-action (id ?action-id) (action-name move-node) (plan-id ?plan-id) (status RUNNING))
  (plan-action (id ?action-id2) (action-name ?action-name2) (plan-id ?plan-id) (status FINAL|FAILED))

  ?skill <- (skill (id ?skill-id) (name ?action-name) (status S_RUNNING))
  (not (wm-fact (key exploration result args? zone ?zn $?args)))

  =>
  (assert (tried-lock (resource ?zn) (result REJECT)))
  (bind ?new-ts (+ 1 ?ts_str))
  (modify ?ze (values 0 ?new-ts))
  (modify ?skill (status S_FAILED))
  (printout t "EXP formulating zone exploration plan " ?zn crlf)
  (assert
    (plan (id EXPLORE-ZONE) (goal-id EXPLORATION))
    (plan-action (id 1) (plan-id EXPLORE-ZONE) (goal-id EXPLORATION) (action-name one-time-lock) (param-names (create$ name)) (param-values (create$ ?zn)))
    (plan-action (id 2) (plan-id EXPLORE-ZONE) (goal-id EXPLORATION) (action-name one-time-lock) (param-names (create$ name)) (param-values (create$ (mirror-name ?zn))))
    (plan-action (id 3) (plan-id EXPLORE-ZONE) (goal-id EXPLORATION) (action-name stop) (param-names r) (param-values ?r))
    (plan-action (id 4) (plan-id EXPLORE-ZONE) (goal-id EXPLORATION) (action-name explore-zone) (param-names r z) (param-values ?r ?zn))
    (plan-action (id 5) (plan-id EXPLORE-ZONE) (goal-id EXPLORATION) (action-name evaluation))
    (plan-action (id 6) (plan-id EXPLORE-ZONE) (goal-id EXPLORATION) (action-name unlock) (param-names (create$ name)) (param-values (create$ ?zn)))
    (plan-action (id 7) (plan-id EXPLORE-ZONE) (goal-id EXPLORATION) (action-name unlock) (param-names (create$ name)) (param-values (create$ (mirror-name ?zn))))
  )
)


(defrule exp-increase-search-limit
  (goal (id EXPLORATION) (mode DISPATCHED))

  ; There is an explorable zone...
  (wm-fact (key exploration zone args? zone ?zn1 machine UNKNOWN team ?team1) (values ?vh_str&:(> ?vh_str 0) ?ts1))

  ; ... but no zone may be searched according to the repeated-search-limit
  (not
  (wm-fact (key exploration zone args? zone ?zn2 machine UNKNOWN team ?team2) (values ?vh-tmp_str&:(> ?vh-tmp_str 0) ?ts_str&:(<= ?ts_str ?*EXP-SEARCH-LIMIT*))))
=>
  (modify ?*EXP-SEARCH-LIMIT* (+ ?*EXP-SEARCH-LIMIT* 1))
)


(defrule exp-tried-locking-all-zones
  "There is at least one unexplored zone with a line, but locks have been denied for
   ALL unexplored zones. So clear all REFUSEs and start requesting locks from the beginning."
  (forall (wm-fact (key exploration zone args? zone ?zn machine ?machine team ?team)) (tried-lock (resource ?zn)))
=>
  (delayed-do-for-all-facts ((?l tried-lock)) (eq ?l:result REJECT)
    (retract ?l)
  )
  (printout t "EXP Flushed tried-locks to restart on unsecure zones" crlf)
)


(defrule exp-skill-explore-zone-final
  (goal (id EXPLORATION) (mode DISPATCHED))
  (plan-action (action-name explore-zone) (status FINAL))
  ?pa <- (plan-action (action-name evaluation) (plan-id EXPLORE-ZONE) (status PENDING))
  (ZoneInterface (id "/explore-zone/info") (zone ?zn-str)
    (orientation ?orientation) (tag_id ?tag-id) (search_state YES)
  )
  (Position3DInterface (id "/explore-zone/found-tag")
    (frame ?frame) (translation $?trans) (rotation $?rot)
  )
  ?lock <- (tried-lock (resource ?zn-sym&:(eq ?zn-sym (sym-cat ?zn-str))))
  (domain-fact (name tag-matching) (param-values ?machine ?side ?team-color ?tag-id))

  ?ze <-
    (wm-fact (key exploration zone args? zone ?zn2&:(eq ?zn2 (sym-cat ?zn-str))  machine ?machine2 team ?team2) (values ?vis2_str ?times-searched_str))
  (domain-fact (name mps-type) (param-values ?machine ?mtype))
  (not (wm-fact (key exploration result args? zone ?zn2 machine ?machine)))
  =>
  (modify ?lock (result ACCEPT))
  (modify ?pa (status FINAL))
  (if (any-factp ((?ft found-tag)) (eq ?ft:name ?machine)) then
    (printout error "BUG: Tag for " ?machine " already found. Locking glitch or agent bug!" crlf)
  else
    (modify ?ze (values ?vis2_str (+ 1 ?times-searched_str)))
    (assert (found-tag (name ?machine) (side ?side) (frame "map") (trans ?trans) (rot ?rot)))
    (assert
      (wm-fact (key exploration result args? zone ?zn2 machine ?machine team ?team-color) (value ?orientation) (is-list FALSE) (type INT))

      (wm-fact (key exploration result args? zone (mirror-name ?zn2)  machine (mirror-name ?machine) team (mirror-team ?team-color)) (value (mirror-orientation ?mtype ?zn2 ?orientation)) (is-list FALSE) (type INT))
    )
    (printout t "EXP explore-zone successfull. Found " ?machine " in " ?zn2 crlf)
  )
)

(defrule exp-skill-explore-zone-failed
  (plan-action (action-name explore-zone) (status FAILED))
  ?p <- (plan-action (action-name evaluation) (status PENDING))
  =>
  (printout t "EXP explore-zone fail, nothing to do for evaluation" crlf)
  (modify ?p (status FINAL))
)

(defrule exp-report-to-refbox
  (goal (id EXPLORATION) (mode DISPATCHED))
  (wm-fact (key refbox team-color) (value ?color))
  (wm-fact (key exploration result args? zone ?zone machine ?machine team ?color) (value ?orientation) (is-list FALSE) (type INT))
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
  (do-for-all-facts ((?er wm-fact)) (and (eq (wm-key-arg ?er:key team) ?team-color) (wm-key-prefix ?er:key (create$ exploration result)))
      (printout error "Report: " (str-cat (wm-key-arg ?er:key machine)) " " (wm-key-arg ?er:key zone) " " ?er:value crlf)
      (bind ?mre (pb-create "llsf_msgs.MachineReportEntry"))
      (pb-set-field ?mre "name" (str-cat (wm-key-arg ?er:key machine)))
      (pb-set-field ?mre "zone" (protobuf-name (wm-key-arg ?er:key zone)))
      (pb-set-field ?mre "rotation" ?er:value)
      (pb-add-list ?mr "machines" ?mre)
  )
  (pb-broadcast ?peer ?mr)
  (modify ?ws (time ?now) (seq (+ ?seq 1)))
)

(defrule exp-exploration-ends-cleanup
  "Clean up lock refusal facts when exploration ends"
  ?g <- (goal (id EXPLORATION) (mode DISPATCHED))
  (wm-fact (key refbox phase) (type UNKNOWN) (value PRODUCTION))
  (wm-fact (key game state) (type UNKNOWN) (value RUNNING))
=>
  (delayed-do-for-all-facts ((?l tried-lock)) (eq ?l:result REJECT)
    (retract ?l)
  )
  (printout t "exploration phase ended, cleaning up" crlf)
  (modify ?g (mode FINISHED) (outcome COMPLETED))
)
