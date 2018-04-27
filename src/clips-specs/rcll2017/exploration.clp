
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

(deftemplate exploration-result
  (slot machine (type SYMBOL) (allowed-values C-BS C-CS1 C-CS2 C-RS1 C-RS2 C-DS C-SS M-BS M-CS1 M-CS2 M-RS1 M-RS2 M-DS M-SS))
  (slot zone (type SYMBOL)
    (allowed-values
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
  (slot team (type SYMBOL) (allowed-values CYAN MAGENTA))
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
    (not (wm-fact (key explore-zone ?zn args? ?vis machine ?machine team ?team ?ts)))
    (wm-fact (key cx identity) (value "R-1"))
    ;(wm-robmem-sync-conf (wm-fact-key-prefix explore-zone) (enabled TRUE))
=>
  (assert
    (wm-fact (key explore-zone C-Z11 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z21 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z31 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z41 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z12 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z22 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z32 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z42 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    ; Can't have machines in *-Z52 (see rulebook)...
    ;(wm-fact (key explore-zone C-Z51 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z62 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z72 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z13 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z23 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z33 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z43 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z53 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z63 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z73 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z62 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z73 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z14 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z24 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z34 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z44 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z54 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z64 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z74 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z64 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z74 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z15 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z25 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z35 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z45 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z55 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z65 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z75 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z65 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z75 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z16 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z26 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z36 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z46 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z56 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z66 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z76 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z66 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z76 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z17 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z27 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z37 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z47 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z57 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z67 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z77 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z67 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z77 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z18 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z28 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z38 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z48 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z58 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z68 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone C-Z78 args? machine UNKNOWN team CYAN) (values (create$ 0 0)) (is-list TRUE) (type INT))

    (wm-fact (key explore-zone M-Z11 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z21 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z31 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z41 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z12 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z22 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z32 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z42 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    ; Can't have machines in *-Z52 (see rulebook)...
    ;(wm-fact (key explore-zone M-Z51 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z62 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z72 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z13 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z23 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z33 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z43 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z53 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z63 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z73 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z62 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z73 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z14 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z24 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z34 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z44 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z54 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z64 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z74 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z64 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z74 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z15 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z25 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z35 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z45 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z55 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z65 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z75 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z65 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z75 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z16 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z26 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z36 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z46 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z56 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z66 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z76 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z66 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z76 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z17 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z27 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z37 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z47 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z57 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z67 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z77 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z67 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z77 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z18 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z28 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z38 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z48 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z58 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z68 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
    (wm-fact (key explore-zone M-Z78 args? machine UNKNOWN team MAGENTA) (values (create$ 0 0)) (is-list TRUE) (type INT))
  )
)

(defrule conf-get-exp-vmax
  (wm-fact (id "/config/rcll/exploration/max-velocity") (type FLOAT) (value ?max-velocity))
  (wm-fact (id "/config/rcll/exploration/max-rotation") (type FLOAT) (value ?max-rotation))
  =>
  (assert (exp-navigator-vmax ?max-velocity ?max-rotation))
)

(defrule conf-get-exp-zone-margin
  ?cv <- (wm-fact (id "/config/rcll/exploration/zone-margin") (type FLOAT) (value ?zone-margin))
  =>
  (assert (exp-zone-margin ?zone-margin))
  (retract ?cv)
)

(defrule exp-node-blocked
  (goal (id EXPLORATION) (mode DISPATCHED))
  (wm-fact (key domain fact self args? r ?r))
  ?s <- (state EXP_GOTO_NEXT)
  (navgraph-node (name ?next-node) (pos $?node-trans))
  (wm-fact (key explore-zone ?zn&:(eq ?zn (get-zone ?node-trans)) args? machine ?machine&~UNKNOWN&~NONE team ?))
  (Position3DInterface (id "Pose") (translation $?pose-trans))
  ?skill <- (skill (status S_RUNNING))
  (test (< 1.5 (distance
    (nth$ 1 ?node-trans)
    (nth$ 2 ?node-trans)
    (nth$ 1 ?pose-trans)
    (nth$ 2 ?pose-trans)
  )))
=>
  (modify ?skill (status S_FAILED))
  (printout t "Node " ?next-node " blocked by " ?machine
    " but we got close enough. Proceeding to next node." crlf)
  (retract ?s)
  (skill-call stop (create$ r) (create$ ?r))
  (bind ?*EXP-ROUTE-IDX* (+ 1 ?*EXP-ROUTE-IDX*))
  (assert (state EXP_IDLE))
)

(defrule goal-reasoner-create-exploration-goal
  (not (goal (id EXPLORATION)))
  (wm-fact (key domain fact entered-field args? r ?r))
  (wm-fact (key domain fact self args? r ?r))
  (wm-fact (key refbox phase) (type UNKNOWN) (value EXPLORATION))
  (wm-fact (key game state) (type UNKNOWN) (value RUNNING))
  =>
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
  ?ze <- (wm-fact (key explore-zone ?zn&:(eq ?zn (get-zone 0.15 ?trans)) args? machine UNKNOWN team ?team) (values ?vis_str ?time-searched_str))
=>
  (bind ?zone (get-zone 0.07 ?trans))
  (if ?zone then
    (modify ?ze (key explore-zone ?zn args? machine NONE team ?team) (values ?vis_str  (+ 1  ?time-searched_str)))
    ;(synced-modify ?ze machine NONE  (+ 1 ?times-searched)) TODO synced-modify
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
  ?ze-f <- (wm-fact (key explore-zone ?zn&:(eq ?zn (get-zone ?zone-margin
                                            (compensate-movement
                                              ?*EXP-MOVEMENT-COMPENSATION*
                                              (create$ ?vx ?vy)
                                              (laser-line-center-map ?ep1 ?ep2 ?frame ?timestamp)
                                              ?timestamp)))
                              args? machine UNKNOWN team ?team) (values ?zn-vh_str&:(< ?zn-vh_str 1) ?ts_str))
=>
  ;(synced-modify ?ze-f line-visibility ?vh) TODO synced-modify
  (modify ?ze-f (key explore-zone ?zn args? machine UNKNOWN team ?team) (values ?vh ?ts_str))
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
  ?ze-f <- (wm-fact (key explore-zone ?zn&:(eq ?zn (get-zone ?zone-margin
                                                      (transform-safe "map" ?frame ?timestamp ?trans ?rot)))
                                                  args? machine UNKNOWN team ?team) (values ?lv_str&:(< ?lv_str 2) ?ts))
  ;(not (locked-resource (resource ?r&:(eq ?r ?zn)))) TODO locked-resource
=>
  ;(synced-modify ?ze-f line-visibility (+ ?lv 1)) TODO synced-modify
  (modify ?ze-f (values (+ ?lv_str 1)  ?ts))
  (printout t "Found tag in " ?zn crlf)
)


(defrule exp-try-locking-line
  (goal (id EXPLORATION) (mode DISPATCHED))
  (wm-fact (key domain fact self args? r ?r))
  (Position3DInterface (id "Pose") (translation $?trans))
  ?ze <- (wm-fact (key explore-zone ?zn args? machine ?machine team ?team) (values ?vh_str&:(> ?vh_str 0)  ?ts_str&:(<= ?ts_str ?*EXP-SEARCH-LIMIT*)))
  (not (wm-fact (key explore-zone ?zn2&:(< (distance-mf (zone-center ?zn2) ?trans) (distance-mf (zone-center ?zn) ?trans)) args? machine ?machine2 team ?team2) (values ?vh2&:(> ?vh2 0) ?ts2&:(<= ?ts2 ?*EXP-SEARCH-LIMIT*)))) 
  (not (tried-lock (resource ?zn)))
  (plan (id ?plan-id&EXPLORATION-PLAN) (goal-id EXPLORATION))
  (not (plan (id EXPLORE-ZONE)))
  
  (plan-action (id ?action-id) (action-name move-node) (plan-id ?plan-id) (status RUNNING))
  (plan-action (id ?action-id2) (action-name ?action-name2) (plan-id ?plan-id) (status FINAL|FAILED))
  
  ?skill <- (skill (id ?skill-id) (name ?action-name) (status S_RUNNING))
  (not (exploration-result (zone ?zn)))
   
  =>
  (assert (tried-lock (resource ?zn) (result REJECT)))
  (bind ?new-ts (+ 1 ?ts_str))
  (modify ?ze (values 0 ?new-ts))
  (modify ?skill (status S_FAILED))
  (printout t "EXP formulating zone exploration plan " ?zn crlf)
  (assert
    (plan (id EXPLORE-ZONE) (goal-id EXPLORATION))
    (plan-action (id 1) (plan-id EXPLORE-ZONE) (goal-id EXPLORATION) (action-name one-time-lock) (param-names (create$ name)) (param-values (create$ ?zn)))
    (plan-action (id 2) (plan-id EXPLORE-ZONE) (goal-id EXPLORATION) (action-name stop) (param-names r) (param-values ?r))
    (plan-action (id 3) (plan-id EXPLORE-ZONE) (goal-id EXPLORATION) (action-name explore-zone) (param-names r z) (param-values ?r ?zn))
    (plan-action (id 4) (plan-id EXPLORE-ZONE) (goal-id EXPLORATION) (action-name evaluation))
    (plan-action (id 5) (plan-id EXPLORE-ZONE) (goal-id EXPLORATION) (action-name unlock) (param-names (create$ name)) (param-values (create$ ?zn)))
  )
)


(defrule exp-increase-search-limit
  (goal (id EXPLORATION) (mode DISPATCHED))

  ; There is an explorable zone...
  (wm-fact (key explore-zone ?zn1 args? machine UNKNOWN team ?team1) (values ?vh_str&:(> ?vh_str 0) ?ts1))

  ; ... but no zone may be searched according to the repeated-search-limit
  (not
  (wm-fact (key explore-zone ?zn2 args? machine UNKNOWN team ?team2) (values ?vh-tmp_str&:(> ?vh-tmp_str 0) ?ts_str&:(<= ?ts_str ?*EXP-SEARCH-LIMIT*))))
=>
  (modify ?*EXP-SEARCH-LIMIT* (+ ?*EXP-SEARCH-LIMIT* 1))
)


(defrule exp-tried-locking-all-zones
  "There is at least one unexplored zone with a line, but locks have been denied for
   ALL unexplored zones. So clear all REFUSEs and start requesting locks from the beginning."


  (forall (wm-fact (key explore-zone ?zn args? machine ?machine team ?team)) (tried-lock (resource ?zn)))
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
  ; We don't check the visibility_history here since that's already done in the explore_zone skill
  (domain-fact (name tag-matching) (param-values ?machine ?side ?team-color ?tag-id))

  ?ze <-
    (wm-fact (key explore-zone ?zn2&:(eq ?zn2 (sym-cat ?zn-str)) args? machine ?machine2 team ?team2) (values ?vis2_str ?times-searched_str))
  (domain-fact (name mps-type) (param-values ?machine ?mtype))
  (not (exploration-result
    (machine ?machine) (zone ?zn2)))
  =>
  (modify ?lock (result ACCEPT))
  (modify ?pa (status FINAL))
 ; (assert
 ;   (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource (sym-cat ?zn-str)))
 ; ) TODO lock RELEASE
  (if (any-factp ((?ft found-tag)) (eq ?ft:name ?machine)) then
    (printout error "BUG: Tag for " ?machine " already found. Locking glitch or agent bug!" crlf)
  else
    ; Order is important here: Update state before zone-exploration to avoid endless loop.
    ;(synced-modify ?ze machine ?machine  (+ 1 ?times-searched)) TODO synced-modify
    ;(synced-assert (str-cat "(found-tag (name " ?machine ") (side " ?side ")"
    ;  "(frame \"map\") (trans " (implode$ ?trans) ") "
    ;  "(rot " (implode$ ?rot) ") )")
    ;) TODO synced-assert

    (modify ?ze (values ?vis2_str (+ 1 ?times-searched_str)))
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

;(defrule exp-skill-explore-zone-failed
;  (goal (id EXPLORATION) (mode DISPATCHED))
;  ;?skill <- (skill (name explore-zone) (status ?status&S_FINAL|S_FAILED))
;  (plan (id EXPLORE-ZONE) (goal-id EXPLORATION))
;  ?pa <- (plan-action (action-name evaluation) (status PENDING))
;  (plan-action (action-name explore-zone) (plan-id EXPLORE-ZONE) (param-values ?r ?zn-str) (status FAILED))
;  (ZoneInterface (id "/explore-zone/info") (zone ?zn-str) (search_state ?s&:(neq ?s YES)))
;  ?ze <- (zone-exploration (name ?zn2&:(eq ?zn2 (sym-cat ?zn-str))) (machine ?machine) ( ?times-searched))
;=>
;  (modify ?pa (status FINAL))
;  (printout t "EXP Exploration of " ?zn-str " failed" crlf)
;  ;(retract ?st-f ?exp-f ?skill)
;  (if (and (eq ?s NO) (eq ?machine UNKNOWN)) then
;    (modify ?ze (machine NONE) ( (+ ? 1)))
;  ;(synced-modify ?ze machine NONE  (+ ? 1)) TODO synced-modify
;    else
;    (modify ?ze (line-visibility 0) ( (+ ? 1)))
;  ;(synced-modify ?ze line-visibility 0  (+ ? 1)) TODO synced-modify
;  )
;)

; TODO RELEASE LOCKS IF action of subgoal fails

(defrule exp-report-to-refbox
  (goal (id EXPLORATION) (mode DISPATCHED))
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
    (bind ?n-explored (length
      (find-all-facts ((?f wm-fact))
        (and (eq (wm-key-arg ?f:key team) ?team-color) (eq (wm-key-arg ?f:key machine) UNKNOWN))
      )
    ))
    (bind ?n-zones (length
      (find-all-facts ((?f wm-fact))
        (eq (wm-key-arg ?f:key team) ?team-color))
    ))
    ; send report for last machine only if the exploration phase is going to end
    ; or we are prepared for production
    (if
      (or
        (< ?n-explored (- ?n-zones 1))
        (>= (nth$ 1 ?game-time) ?latest-report-time)
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
