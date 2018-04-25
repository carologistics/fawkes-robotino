
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
(deftemplate lock
  (slot type (type SYMBOL) (allowed-values GET REFUSE ACCEPT RELEASE RELEASE_RVCD))
  (slot agent (type STRING))
  (slot resource (type SYMBOL))
  (slot priority (type INTEGER) (default 0))
)


(deftemplate zone-exploration
  (slot name (type SYMBOL)
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
  (slot cluster-visibility (type INTEGER) (default 0))
  (slot last-cluster-time (type INTEGER) (default 0))
  (slot line-visibility (type INTEGER) (default 0))
  (slot machine (type SYMBOL) (allowed-values NONE C-BS C-CS1 C-CS2 C-RS1 C-RS2 C-DS C-SS M-BS M-CS1 M-CS2 M-RS1 M-RS2 M-DS  M-SS UNKNOWN) (default UNKNOWN))
  (slot team (type SYMBOL) (allowed-symbols nil CYAN MAGENTA))

  ; for exploration-catch-up in produciton
  (multislot incoming (type SYMBOL) (default (create$)))
  (multislot incoming-agent (type SYMBOL) (default (create$)))
  (slot times-searched (type INTEGER) (default 0))
  (slot sync-id (type INTEGER) (default 0))
  (slot correct (type SYMBOL) (allowed-symbols TRUE UNKNOWN) (default UNKNOWN))
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
    (not (zone-exploration))
=>
  (assert
(zone-exploration (name C-Z11) (team CYAN))
    (zone-exploration (name C-Z21) (team CYAN))
    (zone-exploration (name C-Z31) (team CYAN))
    (zone-exploration (name C-Z41) (team CYAN))
    (zone-exploration (name C-Z12) (team CYAN))
    (zone-exploration (name C-Z22) (team CYAN))
    (zone-exploration (name C-Z32) (team CYAN))
    (zone-exploration (name C-Z42) (team CYAN))
; Can't have machines in *-Z52 (see rulebook)...
;   (zone-exploration (name C-Z52) (team CYAN))
    (zone-exploration (name C-Z62) (team CYAN))
    (zone-exploration (name C-Z72) (team CYAN))
    (zone-exploration (name C-Z13) (team CYAN))
    (zone-exploration (name C-Z23) (team CYAN))
    (zone-exploration (name C-Z33) (team CYAN))
    (zone-exploration (name C-Z43) (team CYAN))
    (zone-exploration (name C-Z53) (team CYAN))
    (zone-exploration (name C-Z63) (team CYAN))
    (zone-exploration (name C-Z73) (team CYAN))
    (zone-exploration (name C-Z14) (team CYAN))
    (zone-exploration (name C-Z24) (team CYAN))
    (zone-exploration (name C-Z34) (team CYAN))
    (zone-exploration (name C-Z44) (team CYAN))
    (zone-exploration (name C-Z54) (team CYAN))
    (zone-exploration (name C-Z64) (team CYAN))
    (zone-exploration (name C-Z74) (team CYAN))
    (zone-exploration (name C-Z15) (team CYAN))
    (zone-exploration (name C-Z25) (team CYAN))
    (zone-exploration (name C-Z35) (team CYAN))
    (zone-exploration (name C-Z45) (team CYAN))
    (zone-exploration (name C-Z55) (team CYAN))
    (zone-exploration (name C-Z65) (team CYAN))
    (zone-exploration (name C-Z75) (team CYAN))
    (zone-exploration (name C-Z16) (team CYAN))
    (zone-exploration (name C-Z26) (team CYAN))
    (zone-exploration (name C-Z36) (team CYAN))
    (zone-exploration (name C-Z46) (team CYAN))
    (zone-exploration (name C-Z56) (team CYAN))
    (zone-exploration (name C-Z66) (team CYAN))
    (zone-exploration (name C-Z76) (team CYAN))
    (zone-exploration (name C-Z17) (team CYAN))
    (zone-exploration (name C-Z27) (team CYAN))
    (zone-exploration (name C-Z37) (team CYAN))
    (zone-exploration (name C-Z47) (team CYAN))
    (zone-exploration (name C-Z57) (team CYAN))
    (zone-exploration (name C-Z67) (team CYAN))
    (zone-exploration (name C-Z77) (team CYAN))
    (zone-exploration (name C-Z18) (team CYAN))
    (zone-exploration (name C-Z28) (team CYAN))
    (zone-exploration (name C-Z38) (team CYAN))
    (zone-exploration (name C-Z48) (team CYAN))
    (zone-exploration (name C-Z58) (team CYAN))
    (zone-exploration (name C-Z68) (team CYAN))
    (zone-exploration (name C-Z78) (team CYAN))

    (zone-exploration (name M-Z11) (team MAGENTA))
    (zone-exploration (name M-Z21) (team MAGENTA))
    (zone-exploration (name M-Z31) (team MAGENTA))
    (zone-exploration (name M-Z41) (team MAGENTA))
    (zone-exploration (name M-Z12) (team MAGENTA))
    (zone-exploration (name M-Z22) (team MAGENTA))
    (zone-exploration (name M-Z32) (team MAGENTA))
    (zone-exploration (name M-Z42) (team MAGENTA))
; Can't have machines in *-Z52 (see rulebook)...
;   (zone-exploration (name M-Z52) (team MAGENTA))
    (zone-exploration (name M-Z62) (team MAGENTA))
    (zone-exploration (name M-Z72) (team MAGENTA))
    (zone-exploration (name M-Z13) (team MAGENTA))
    (zone-exploration (name M-Z23) (team MAGENTA))
    (zone-exploration (name M-Z33) (team MAGENTA))
    (zone-exploration (name M-Z43) (team MAGENTA))
    (zone-exploration (name M-Z53) (team MAGENTA))
    (zone-exploration (name M-Z63) (team MAGENTA))
    (zone-exploration (name M-Z73) (team MAGENTA))
    (zone-exploration (name M-Z14) (team MAGENTA))
    (zone-exploration (name M-Z24) (team MAGENTA))
    (zone-exploration (name M-Z34) (team MAGENTA))
    (zone-exploration (name M-Z44) (team MAGENTA))
    (zone-exploration (name M-Z54) (team MAGENTA))
    (zone-exploration (name M-Z64) (team MAGENTA))
    (zone-exploration (name M-Z74) (team MAGENTA))
    (zone-exploration (name M-Z15) (team MAGENTA))
    (zone-exploration (name M-Z25) (team MAGENTA))
    (zone-exploration (name M-Z35) (team MAGENTA))
    (zone-exploration (name M-Z45) (team MAGENTA))
    (zone-exploration (name M-Z55) (team MAGENTA))
    (zone-exploration (name M-Z65) (team MAGENTA))
    (zone-exploration (name M-Z75) (team MAGENTA))
    (zone-exploration (name M-Z16) (team MAGENTA))
    (zone-exploration (name M-Z26) (team MAGENTA))
    (zone-exploration (name M-Z36) (team MAGENTA))
    (zone-exploration (name M-Z46) (team MAGENTA))
    (zone-exploration (name M-Z56) (team MAGENTA))
    (zone-exploration (name M-Z66) (team MAGENTA))
    (zone-exploration (name M-Z76) (team MAGENTA))
    (zone-exploration (name M-Z17) (team MAGENTA))
    (zone-exploration (name M-Z27) (team MAGENTA))
    (zone-exploration (name M-Z37) (team MAGENTA))
    (zone-exploration (name M-Z47) (team MAGENTA))
    (zone-exploration (name M-Z57) (team MAGENTA))
    (zone-exploration (name M-Z67) (team MAGENTA))
    (zone-exploration (name M-Z77) (team MAGENTA))
    (zone-exploration (name M-Z18) (team MAGENTA))
    (zone-exploration (name M-Z28) (team MAGENTA))
    (zone-exploration (name M-Z38) (team MAGENTA))
    (zone-exploration (name M-Z48) (team MAGENTA))
    (zone-exploration (name M-Z58) (team MAGENTA))
    (zone-exploration (name M-Z68) (team MAGENTA))
    (zone-exploration (name M-Z78) (team MAGENTA))
  )
)


;utils
(deffunction distance (?x ?y ?x2 ?y2)
  "Returns the distance of two points in the x,y-plane."
  (return (float (sqrt (float(+ (* (- ?x ?x2) (- ?x ?x2)) (* (- ?y ?y2) (- ?y ?y2)))))))
)

(deffunction protobuf-name (?zone)
  (return
    (str-cat (sub-string 1 1 ?zone) "_" (sub-string 3 99 ?zone))
  )
)

(deffunction transform-safe (?to-frame ?from-frame ?timestamp ?trans ?rot)
  (if (tf-can-transform ?to-frame ?from-frame ?timestamp) then
    (bind ?rv (tf-transform-pose ?to-frame ?from-frame ?timestamp ?trans ?rot))
  else
    (if (tf-can-transform ?to-frame ?from-frame (create$ 0 0)) then
      (bind ?rv (tf-transform-pose ?to-frame ?from-frame (create$ 0 0) ?trans ?rot))
    else
      (return FALSE)
    )
  )
  (if (= (length$ ?rv) 7) then
    (return ?rv)
  else
    (return FALSE)
  )
)

(deffunction compensate-movement (?factor ?v-odom ?p ?timestamp)
  (if (eq ?p FALSE) then
    (return FALSE)
  )
  (if (= 2 (length$ ?v-odom)) then
    (bind ?v-odom (create$ (nth$ 1 ?v-odom) (nth$ 2 ?v-odom) 0))
  )
  (bind ?p-map (transform-safe "map" "base_link" ?timestamp (create$ 0 0 0) (create$ 0 0 0 1)))
  (if (<> 7 (length$ ?p-map)) then
    (return FALSE)
  )
  (bind ?pv-map (transform-safe "map" "base_link" ?timestamp ?v-odom (create$ 0 0 0 1)))
  (if (<> 7 (length$ ?pv-map)) then
    (return FALSE)
  )
  (bind ?v-map (create$
    (- (nth$ 1 ?pv-map) (nth$ 1 ?p-map))
    (- (nth$ 2 ?pv-map) (nth$ 2 ?p-map))
  ))
  (bind ?rv (create$
    (+ (nth$ 1 ?p) (* ?factor (nth$ 1 ?v-map)))
    (+ (nth$ 2 ?p) (* ?factor (nth$ 2 ?v-map)))
  ))
  (return ?rv)
)

(deffunction distance-mf (?p1 ?p2)
  (return (distance (nth$ 1 ?p1) (nth$ 2 ?p1) (nth$ 1 ?p2) (nth$ 2 ?p2)))
)

(deffunction round-down (?x)
  (bind ?round (round ?x))
  (if (< ?x ?round) then
    (return (- ?round 1))
  )
  (return ?round)
)

(deffunction round-up (?x)
  (bind ?round (round ?x))
  (if (> ?x ?round) then
    (return (+ ?round 1))
  )
  (return ?round)
)
(deffunction get-zone (?margin $?vector)
  "Return the zone name for a given map coordinate $?vector if its
   distance from the zone borders is greater or equal than ?margin."
  (if (eq ?vector FALSE) then
    (return FALSE)
  )
  (bind ?x (nth$ 1 ?vector))
  (bind ?y (nth$ 2 ?vector))
  (if (not (and (numberp ?x) (numberp ?y))) then
    (return FALSE)
  )

  (if (<= ?y 0) then
    ; y <= 0 is outside the playing field
    (return FALSE)
  else
    (bind ?yr (round-up ?y))
  )

  (if (or (< (- ?x ?margin) (round-down ?x))
          (> (+ ?x ?margin) (round-up ?x))
          (< (- ?y ?margin) (round-down ?y))
          (> (+ ?y ?margin) (round-up ?y))
      ) then
    (return FALSE)
  )

  (if (< ?x 0) then
    (bind ?rv M-Z)
    (bind ?x (* ?x -1))
  else
    (bind ?rv C-Z)
  )
  (bind ?xr (round-up ?x))

  (return (sym-cat ?rv ?xr ?yr))
)
(deffunction utils-get-2d-center (?x1 ?y1 ?x2 ?y2)
  (return (create$ (/ (+ ?x1 ?x2) 2) (/ (+ ?y1 ?y2) 2)))
)
(deffunction laser-line-center-map (?ep1 ?ep2 ?frame ?timestamp)
  (bind ?c (utils-get-2d-center (nth$ 1 ?ep1) (nth$ 2 ?ep1) (nth$ 1 ?ep2) (nth$ 2 ?ep2)))
  (bind ?c3 (nth$ 1 ?c) (nth$ 2 ?c) 0)
  (return (transform-safe "map" ?frame ?timestamp ?c3 (create$ 0 0 0 1)))
)

(deffunction mirror-name (?zn)
  (bind ?team (sub-string 1 1 ?zn))
  (bind ?zone (sub-string 3 99 ?zn))
  (if (eq ?team "M") then
    (return (sym-cat "C-" ?zone))
  else
    (return (sym-cat "M-" ?zone))
  )
)

(deffunction want-mirrored-rotation (?mtype ?zone)
"According to the RCLL2017 rulebook, this is when a machine is mirrored"
  (bind ?zn (str-cat ?zone))
  (bind ?x (eval (sub-string 4 4 ?zn)))
  (bind ?y (eval (sub-string 5 5 ?zn)))

  (return (or (member$ ?mtype (create$ BS DS SS))
              (not (or (eq ?x 7) ; left or right
                       (eq ?y 8) ; top wall
                       (eq ?y 1) ; bottom wall
                       (and (member$ ?x (create$ 5 6 7)); insertion
                            (eq ?y 2)
                       )
                   )
              )
  ))
)

(deffunction mirror-orientation (?mtype ?zone ?ori)
  (bind ?zn (str-cat ?zone))
  (bind ?t (sub-string 1 1 ?zn))
  (if (want-mirrored-rotation ?mtype ?zone)
   then
    (if (eq ?t "C")
     then
      (do-for-fact ((?mo domain-fact)) (and (eq (nth$ 1 ?mo:param-values) ?ori) (eq ?mo:name mirror-orientation))
        (bind ?m-ori (nth$ 2 ?mo:param-values))
      )
     else
      (do-for-fact ((?mo domain-fact)) (and (eq (nth$ 2 ?mo:param-values) ?ori) (eq ?mo:name mirror-orientation))
        (bind ?m-ori (nth$ 1 ?mo:param-values))
      )
    )
    (return ?m-ori)
   else
    (bind ?x (eval (sub-string 4 4 ?zn)))
    (bind ?y (eval (sub-string 5 5 ?zn)))

    (if (eq ?y 8) then
      (return 180)
    )
    (if (or (eq ?y 1) (eq ?y 2)) then
      (return 0)
    )
    (if (and (eq ?x 7) (eq ?t "M")) then  ; this is the other way around, because I compare with the team color of the originalting machine
      (return 90)
    )
    (if (and (eq ?x 7) (eq ?t "C")) then
      (return 270)
    )
    (printout error "error in rotation of machines, checked all possible cases, but nothing cateched" crlf)
    (return ?ori)
  )
)

(deffunction mirror-team (?team)
  (if (eq (sym-cat ?team) CYAN) then
    (return MAGENTA)
  else
    (return CYAN)
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
  (zone-exploration (name ?zn&:(eq ?zn (get-zone ?node-trans))) (machine ?machine&~UNKNOWN&~NONE))
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
  (wm-fact (key domain fact self args? r ?r))
  (wm-fact (key domain fact entered-field args? r ?r))
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
  ?ze <- (zone-exploration
    (name ?zn&:(eq ?zn (get-zone 0.15 ?trans)))
    (machine UNKNOWN)
    (times-searched ?times-searched)
  )
=>
  (bind ?zone (get-zone 0.07 ?trans))
  (if ?zone then
    ;(synced-modify ?ze machine NONE times-searched (+ 1 ?times-searched)) TODO synced-modify
   (modify ?ze (machine NONE) (times-searched (+ 1 ?times-searched)))
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
  ;(synced-modify ?ze-f line-visibility ?vh) TODO synced-modify
  (modify ?ze-f (line-visibility ?vh))
  (printout warn "EXP found line: " ?zn " vh: " ?vh crlf)
)


(defrule exp-found-cluster
  "Found a cluster: Remember it for later when we run out of lines to explore."
  (goal (id EXPLORATION) (mode DISPATCHED))

  (wm-fact (key refbox game-time) (values $?game-time))
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
  (modify ?ze-f (cluster-visibility (+ ?zn-vh 1)) (last-cluster-time (nth$ 1 ?game-time)))
  ;(synced-modify ?ze-f
  ;  cluster-visibility (+ ?zn-vh 1)
  ;  last-cluster-time (nth$ 1 ?game-time)
  ;) TODO synced-modify
  (printout warn "EXP found cluster: " ?zn " vh: " (+ ?zn-vh 1) crlf)
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
  ?ze-f <- (zone-exploration
    (name ?zn&:(eq ?zn (get-zone ?zone-margin (transform-safe "map" ?frame ?timestamp ?trans ?rot))))
    (machine UNKNOWN)
    (line-visibility ?lv&:(< ?lv 2))
  )
  ;(not (locked-resource (resource ?r&:(eq ?r ?zn)))) TODO locked-resource
=>
  ;(synced-modify ?ze-f line-visibility (+ ?lv 1)) TODO synced-modify
  (modify ?ze-f (line-visibility (+ ?lv 1)))
  (printout t "Found tag in " ?zn crlf)
)


(defrule exp-try-locking-line
  (goal (id EXPLORATION) (mode DISPATCHED))
  (wm-fact (key domain fact self args? r ?r))
  ; Not currently locked/trying to lock anything
  ;(not (lock (type GET) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?)))  TODO lock GET
  ;(not (lock (type ACCEPT) (resource ?)))
  ; An explorable zone for which no lock was refused yet
  (zone-exploration
    (name ?zn)
    (machine UNKNOWN)
    (line-visibility ?vh&:(> ?vh 0))
    (times-searched ?ts&:(<= ?ts ?*EXP-SEARCH-LIMIT*))
  )
  ;(not (lock (type REFUSE) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?zn)))  TODO lock REFUSE

  ; Neither this zone nor the opposite zone is locked
  ;(not (locked-resource (resource ?r&:(eq ?r ?zn))))  TODO locked-resource
  ;(not (locked-resource (resource ?r2&:(eq ?r2 (mirror-name ?zn)))))

  ; Locks for all closer zones with a line-visibility > 0 have been refused
  (Position3DInterface (id "Pose") (translation $?trans))
  ;(forall
   ; (zone-exploration (machine UNKNOWN) (line-visibility ?vh2&:(> ?vh2 0))
   ;   (name ?zn2&:(< (distance-mf ?trans (zone-center ?zn2)) (distance-mf ?trans (zone-center ?zn))))
   ; )
    ;(lock (type REFUSE) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?zn2))  TODO lock REFUSE
  ;)
  (plan (id ?plan-id&EXPLORATION-PLAN) (goal-id EXPLORATION))
  (plan-action (id ?action-id) (action-name ?action-name) (plan-id ?plan-id) (status RUNNING))
  (plan-action (id ?action-id2) (action-name ?action-name2) (plan-id ?plan-id) (status FINAL|FAILED))
  ?skill <- (skill (id ?skill-id) (name ?action-name) (status S_RUNNING))
  (zone-exploration (name ?zn))
  (not (explore-flag))
  =>
  (modify ?skill (status S_FAILED))
  (printout t "EXP formulating zone exploration plan " ?zn crlf)
  (assert
    (plan (id EXPLORE-ZONE) (goal-id EXPLORATION))
    ;(plan-action (id 1) (plan-id EXPLORE-ZONE) (goal-id EXPLORATION) (action-name lock-resource))
    (plan-action (id 2) (plan-id EXPLORE-ZONE) (goal-id EXPLORATION) (action-name stop) (param-names r) (param-values ?r))
    (plan-action (id 3) (plan-id EXPLORE-ZONE) (goal-id EXPLORATION) (action-name explore-zone) (param-names r z) (param-values ?r ?zn))
    ;(plan-action (id 4) (plan-id EXPLORE-ZONE) (goal-id EXPLORATION) (action-name release-resource))
  )
  (assert (explore-flag))
)


(defrule exp-increase-search-limit
  (goal (id EXPLORATION) (mode DISPATCHED))

  ; There is an explorable zone...
  (zone-exploration (machine UNKNOWN) (line-visibility ?vh&:(> ?vh 0)))

  ; ... but no zone may be searched according to the repeated-search-limit
  (not (zone-exploration
    (machine UNKNOWN)
    (line-visibility ?vh-tmp&:(> ?vh-tmp 0))
    (times-searched ?ts&:(<= ?ts ?*EXP-SEARCH-LIMIT*))
  ))
=>
  (modify ?*EXP-SEARCH-LIMIT* (+ ?*EXP-SEARCH-LIMIT* 1))
)

;
;(defrule exp-tried-locking-all-zones
;  "There is at least one unexplored zone with a line, but locks have been denied for
;   ALL unexplored zones. So clear all REFUSEs and start requesting locks from the beginning."
;  (zone-exploration (name ?) (machine UNKNOWN) (line-visibility ?tmp&:(> ?tmp 0)))
;  (not
;    (zone-exploration (name ?zn) (machine UNKNOWN) (line-visibility ?vh&:(> ?vh 0)))
;    ;(lock (type REFUSE) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?zn))  TODO lock REFUSE
;  )
;=>
; ; (delayed-do-for-all-facts ((?l lock)) (and (eq ?l:type REFUSE) (eq ?l:agent ?*ROBOT-NAME*))
; ;   (retract ?l)
; ; )
;  (printout warn "empty effect" crlf)
;)


(defrule exp-skill-explore-zone-final
  (goal (id EXPLORATION) (mode DISPATCHED))
  (plan-action (action-name explore-zone) (status FINAL))
  (ZoneInterface (id "/explore-zone/info") (zone ?zn-str)
    (orientation ?orientation) (tag_id ?tag-id) (search_state YES)
  )
  (Position3DInterface (id "/explore-zone/found-tag")
    (frame ?frame) (translation $?trans) (rotation $?rot)
  )
  ; We don't check the visibility_history here since that's already done in the explore_zone skill
  (domain-fact (name tag-matching) (param-values ?machine ?side ?team-color ?tag-id))
  ?ze <- (zone-exploration (name ?zn2&:(eq ?zn2 (sym-cat ?zn-str))) (times-searched ?times-searched))
  (domain-fact (name mps-type) (param-values ?machine ?mtype))
  (not (exploration-result
    (machine ?machine) (zone ?zn2)))
  ?ef <- (explore-flag)
  =>
 ; (assert
 ;   (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource (sym-cat ?zn-str)))
 ; ) TODO lock RELEASE
  (if (any-factp ((?ft found-tag)) (eq ?ft:name ?machine)) then
    (printout error "BUG: Tag for " ?machine " already found. Locking glitch or agent bug!" crlf)
  else
    ; Order is important here: Update state before zone-exploration to avoid endless loop.
    ;(synced-modify ?ze machine ?machine times-searched (+ 1 ?times-searched)) TODO synced-modify
    ;(synced-assert (str-cat "(found-tag (name " ?machine ") (side " ?side ")"
    ;  "(frame \"map\") (trans " (implode$ ?trans) ") "
    ;  "(rot " (implode$ ?rot) ") )")
    ;) TODO synced-assert

    (modify ?ze (machine ?machine) (times-searched (+ 1 ?times-searched)))
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
    (printout t "Exploration successfull. Found " ?machine " in " ?zn2 crlf)
    (retract ?ef)
  )
)
(defrule exp-skill-explore-zone-failed
  (goal (id EXPLORATION) (mode DISPATCHED))
  ;?skill <- (skill (name explore-zone) (status ?status&S_FINAL|S_FAILED))
  (plan (id EXPLORE-ZONE) (goal-id EXPLORATION))
  (plan-action (action-name explore-zone) (plan-id EXPLORE-ZONE) (param-values ?r ?zn-str) (status FAILED))
  (ZoneInterface (id "/explore-zone/info") (zone ?zn-str) (search_state ?s&:(neq ?s YES)))
  ?ze <- (zone-exploration (name ?zn2&:(eq ?zn2 (sym-cat ?zn-str))) (machine ?machine) (times-searched ?times-searched))
  ?ef <- (explore-flag)
=>
  (printout t "Exploration of " ?zn-str " failed" crlf)
  ;(retract ?st-f ?exp-f ?skill)
  (if (and (eq ?s NO) (eq ?machine UNKNOWN)) then
    (modify ?ze (machine NONE) (times-searched (+ ?times-searched 1)))
  ;(synced-modify ?ze machine NONE times-searched (+ ?times-searched 1)) TODO synced-modify
    else
    (modify ?ze (line-visibility 0) (times-searched (+ ?times-searched 1)))
  ;(synced-modify ?ze line-visibility 0 times-searched (+ ?times-searched 1)) TODO synced-modify
  )
  (retract ?ef)
  )

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
  (printout error "Reported mps : " ?mr crlf)
)

(defrule exp-exploration-ends-cleanup
  "Clean up lock refusal facts when exploration ends"
  ?g <- (goal (id EXPLORATION) (mode DISPATCHED))
  (wm-fact (key refbox phase) (type UNKNOWN) (value PRODUCTION))
  (wm-fact (key game state) (type UNKNOWN) (value RUNNING))

=>
  (delayed-do-for-all-facts ((?l lock)) (eq ?l:type REFUSE)
    (retract ?l)
  )
  (printout t "exploration phase ended, cleaning up" crlf)
  (modify ?g (mode FINISHED) (outcome COMPLETED))
)
