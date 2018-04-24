
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
)


; This deftemplate is only needed because do-for-all-facts doesn't
; work for implied deftemplates
;facts
(deftemplate exp-next-node
  (slot node (type STRING))
)

(deftemplate explore-zone-target
  (slot zone (type SYMBOL))
)

; EXPLORATION

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

(deftemplate field-ground-truth
  (slot machine (type SYMBOL))
  (slot yaw (type FLOAT))
  (slot orientation (type INTEGER))
  (slot zone (type SYMBOL))
  (slot mtype (type SYMBOL))
)

(deftemplate navgraph-added-for-mps
  (slot name (type SYMBOL))
)

(deftemplate last-navgraph-compute-msg
  (slot id (type INTEGER))
  (slot final (type SYMBOL) (allowed-symbols TRUE FALSE) (default FALSE))
)

(deftemplate navigator-default-vmax
  (slot velocity (type FLOAT))
  (slot rotation (type FLOAT))
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

(deffunction navigator-set-speed (?max-velocity ?max-rotation)
  (bind ?msg (blackboard-create-msg "NavigatorInterface::Navigator" "SetMaxVelocityMessage"))
  (blackboard-set-msg-field ?msg "max_velocity" ?max-velocity)
  (blackboard-send-msg ?msg)
  (bind ?msg (blackboard-create-msg "NavigatorInterface::Navigator" "SetMaxRotationMessage"))
  (blackboard-set-msg-field ?msg "max_rotation" ?max-rotation)
  (blackboard-send-msg ?msg)
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
(deffunction mirror-trans ($?trans)
  (return (create$
    (- 0 (nth$ 1 ?trans))
    (nth$ 2 ?trans)
    (nth$ 3 ?trans)
  ))
)

(deffunction translate-tag-x (?tag-yaw ?dx $?trans)
  (return (create$
    (+ (nth$ 1 ?trans) (* (cos ?tag-yaw) ?dx))
    (+ (nth$ 2 ?trans) (* (sin ?tag-yaw) ?dx))
    (nth$ 3 ?trans)
  ))
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


(deffunction mirror-rot (?mtype ?zone $?rot)
"Mirror rotation according to rules, $?rot is a quaternion"
  (if (want-mirrored-rotation ?mtype ?zone) then
    (bind ?yaw (tf-yaw-from-quat ?rot))
    (bind ?yaw-mirror (+ (- 0 (- ?yaw ?*PI-HALF*)) ?*PI-HALF*))
    (if (> ?yaw-mirror ?*PI*) then
      (bind ?yaw-mirror (- ?yaw-mirror ?*2PI*)))
    (if (< ?yaw-mirror (- 0 ?*PI*)) then
      (bind ?yaw-mirror (+ ?yaw-mirror ?*2PI*)))
    (return (tf-quat-from-yaw ?yaw-mirror))
  else
    (bind ?zn (str-cat ?zone))
    (bind ?t (sub-string 1 1 ?zn))
    (bind ?x (eval (sub-string 4 4 ?zn)))
    (bind ?y (eval (sub-string 5 5 ?zn)))

    (if (eq ?y 8) then
      (return (tf-quat-from-yaw ?*PI*))
    )
    (if (or (eq ?y 1) (eq ?y 2)) then
      (return (tf-quat-from-yaw 0.0))
    )
    (if (and (eq ?x 7) (eq ?t "M")) then  ; this is the other way around, because I compare with the team color of the originalting machine
      (return (tf-quat-from-yaw ?*PI-HALF*))
    )
    (if (and (eq ?x 7) (eq ?t "C")) then
      (return (tf-quat-from-yaw (- 0 ?*PI-HALF*)))
    )
    (printout error "error in rotation of machines, checked all possible cases, but nothing cateched" crlf)
    (return ?rot)
  )
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


(defrule start-exploration-goal
  (not (goal (id EXPLORATION)))
  (wm-fact (key refbox phase) (type UNKNOWN) (value EXPLORATION))
  (wm-fact (key game state) (type UNKNOWN) (value RUNNING))
  =>
  (assert (goal (id EXPLORATION) (type MAINTAIN)))
)

(defrule expand-exploration-goal
  ?g <- (goal (id EXPLORATION) (mode SELECTED))
  (not (plan (goal-id EXPLORATION)))
  =>
  (assert (plan (id EXPLORATION-PLAN) (goal-id EXPLORATION)))
  (modify ?g (mode EXPANDED))
)

;(defrule launch-exploration-blackboards



;)

;Read exploration rows from config
(defrule exp-cfg-get-row
  "Read configuration for exploration row order of machines"
  (declare (salience ?*PRIORITY-WM*))
  (goal (id EXPLORATION) (mode DISPATCHED))
  (wm-fact (key domain fact self args? r ?robot-name))
  ;(robot-name ?robot-name)

  (wm-fact (key refbox team-color) (value ?team))
  ;(team-color ?team)
  (wm-fact
    (id ?id&:(eq ?id (str-cat "/config/rcll/route/" ?team "/" ?robot-name)))
    (values $?route)
  )
=>
  (assert (exp-route ?route)
  )
 ; (assert (exp-route "exp-12"))
  (printout t "Exploration route: " ?route " " ?team " " ?robot-name crlf)
)

(defrule exp-start
  (goal (id EXPLORATION) (mode DISPATCHED))
  ;?st <- (exploration-start)
  (not (timer (name send-machine-reports)))
  (wm-fact (key refbox team-color) (value ?team-color))
  ;(team-color ?team-color)
  (NavigatorInterface (id "Navigator") (max_velocity ?max-velocity) (max_rotation ?max-rotation))
=>
  ;(retract ?st)
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
  (goal (id EXPLORATION) (mode DISPATCHED))
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
  (goal (id EXPLORATION) (mode DISPATCHED))
  (plan (id ?plan-id) (goal-id EXPLORATION))
  (wm-fact (key domain fact self args? r ?r))
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
  (assert (plan-action (id 1) (action-name move-node) (param-values ?r ?next-node) (plan-id ?plan-id) (goal-id EXPLORATION) (status PENDING)))
  ;(skill-call goto place ?next-node) TODO make this a plan action
)


(defrule exp-node-blocked
  (goal (id EXPLORATION) (mode DISPATCHED))
  (plan (id ?plan-id) (goal-id EXPLORATION))
  (wm-fact (key domain fact self args? r ?r))

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
  (assert (plan-action (id 1) (action-name stop) (param-values ?r) (status PENDING)))  ;(skill-call relgoto x 0 y 0) TODO Make this a plan action
  (bind ?*EXP-ROUTE-IDX* (+ 1 ?*EXP-ROUTE-IDX*))
  (assert (state EXP_IDLE))
)


(defrule exp-goto-next-failed
  (goal (id EXPLORATION) (mode DISPATCHED))
  ?s <- (state EXP_GOTO_NEXT)
  ?pa <- (plan-action (action-name move-node) (status FAILED))
  ;?skill-f <- (skill-done (name "goto") (status FAILED)) TODO this should be a failed action
  (exp-next-node (node ?node))
  (navgraph-node (name ?node) (pos $?node-trans))

  (Position3DInterface (id "Pose") (translation $?trans))
  (navigator-default-vmax (velocity ?max-velocity) (rotation ?max-rotation))
=>
  (if (< (distance-mf ?node-trans ?trans) 1.5) then
    (bind ?*EXP-ROUTE-IDX* (+ 1 ?*EXP-ROUTE-IDX*))
  )
  (retract ?s ?pa);?skill-f)
  (navigator-set-speed ?max-velocity ?max-rotation)
  (assert
    (exp-searching)
    (state EXP_IDLE)
  )
)


(defrule exp-goto-next-final
  (goal (id EXPLORATION) (mode DISPATCHED))
  ?s <- (state EXP_GOTO_NEXT)
  ?pa <- (plan-action (action-name move-node) (status ?status))
  ;?skill-f <- (skill-done (name "goto") (status ?)) TODO this schould be a plan-action
  (navigator-default-vmax (velocity ?max-velocity) (rotation ?max-rotation))
=>
  (retract ?s ?pa) ;?skill-f)
  (navigator-set-speed ?max-velocity ?max-rotation)
  (bind ?*EXP-ROUTE-IDX* (+ 1 ?*EXP-ROUTE-IDX*))
  (assert
    (exp-searching)
    (state EXP_IDLE)
  )
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
;  (if ?zone then
    ;(synced-modify ?ze machine NONE times-searched (+ 1 ?times-searched)) TODO now wm-facts
;  )
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
  ;(synced-modify ?ze-f line-visibility ?vh) TODO now wm-fact
  (printout warn "EXP found line: " ?zn " vh: " ?vh crlf)
)


(defrule exp-found-cluster
  "Found a cluster: Remember it for later when we run out of lines to explore."
  (goal (id EXPLORATION) (mode DISPATCHED))
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
  ;(synced-modify ?ze-f
  ;  cluster-visibility (+ ?zn-vh 1)
  ;  last-cluster-time (nth$ 1 ?game-time)
  ;) TODO now wm-fact
  (printout warn "EXP found cluster: " ?zn " vh: " (+ ?zn-vh 1) crlf)
)


(defrule exp-found-tag
  (goal (id EXPLORATION) (mode DISPATCHED))
  ?srch-f <- (exp-searching)
  (domain-fact (name tag-matching) (param-values ?machine ?side ?col ?tag))
;(tag-id ?tag) (machine ?machine) (side ?side))
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
  ;(not (locked-resource (resource ?r&:(eq ?r ?zn)))) TODO what about locks
  ?st-f <- (state ?)
=>
  ;(synced-modify ?ze-f line-visibility (+ ?lv 1)) TODO now wm-fact
)


(defrule exp-try-locking-line
  (goal (id EXPLORATION) (mode DISPATCHED))
  (exp-searching)
  (state EXP_GOTO_NEXT|EXP_IDLE)
  ; Not currently locked/trying to lock anything
  ;(not (lock (type GET) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?)))  TODO what about locks
  ;(not (lock (type ACCEPT) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?)))  TODO what about locks

  ; An explorable zone for which no lock was refused yet
  (exp-repeated-search-limit ?search-limit)
  (zone-exploration
    (name ?zn)
    (machine UNKNOWN)
    (line-visibility ?vh&:(> ?vh 0))
    (times-searched ?ts&:(<= ?ts ?search-limit))
  )
  ;(not (lock (type REFUSE) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?zn)))  TODO what about locks

  ; Neither this zone nor the opposite zone is locked
  ;(not (locked-resource (resource ?r&:(eq ?r ?zn))))  TODO what about locked-resource
  ;(not (locked-resource (resource ?r2&:(eq ?r2 (mirror-name ?zn)))))

  ; Locks for all closer zones with a line-visibility > 0 have been refused
  (Position3DInterface (id "Pose") (translation $?trans))
  (not
    (zone-exploration (machine UNKNOWN) (line-visibility ?vh2&:(> ?vh2 0))
      (name ?zn2&:(< (distance-mf ?trans (zone-center ?zn2)) (distance-mf ?trans (zone-center ?zn))))
    )
    ;(lock (type REFUSE) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?zn2))  TODO what about locks
  )
=>
  (printout t "EXP trying to lock zone " ?zn crlf)
  ;(assert
    ;(lock (type GET) (agent ?*ROBOT-NAME*) (resource ?zn))  TODO what about locks
  ;)
)


(defrule exp-increase-search-limit
  (goal (id EXPLORATION) (mode DISPATCHED))

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
  (not
    (zone-exploration (name ?zn) (machine UNKNOWN) (line-visibility ?vh&:(> ?vh 0)))
    ;(lock (type REFUSE) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?zn))  TODO what about locks
  )
=>
  ;(delayed-do-for-all-facts ((?l lock)) (and (eq ?l:type REFUSE) (eq ?l:agent ?*ROBOT-NAME*))  TODO what about locks
  ;  (retract ?l)
  ;)
  (printout warn "empty effect" crlf)
)


(defrule exp-stop-to-investigate-zone
  "Lock for an explorable zone was accepted"
  (goal (id EXPLORATION) (mode DISPATCHED))
  (plan (id ?plan-id) (goal-id EXPLORATION))
  (wm-fact (key domain fact self args? r ?r))
  (exp-searching)
  ?st-f <- (state EXP_GOTO_NEXT)

  (zone-exploration (name ?zn))
  ;(lock (type ACCEPT) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?zn))  TODO what about locks
=>
  (printout t "EXP exploring zone " ?zn crlf)
  (delayed-do-for-all-facts ((?exp-f explore-zone-target)) TRUE (retract ?exp-f))
  (retract ?st-f)
  (assert
    (state EXP_STOPPING)
    (explore-zone-target (zone ?zn))
  )
  (assert (plan-action (action-name stop) (param-values ?r) (plan-id ?plan-id) (goal-id EXPLORATION) (status PENDING)))
  ;(skill-call relgoto x y 0) TODO make this a plan action
)


(defrule exp-skill-explore-zone
  (goal (id EXPLORATION) (mode DISPATCHED))
  (plan (id ?plan-id) (goal-id EXPLORATION))
  ?srch-f <- (exp-searching)
  ?st-f <- (state EXP_STOPPING)
  (explore-zone-target (zone ?zn))
  ?pa <- (plan-action (action-name stop) (status ?s))
  ;?skill-f <- (skill-done (name "relgoto")) TODO this should be a plan action
  (MotorInterface (id "Robotino")
    (vx ?vx&:(< ?vx 0.01)) (vy ?vy&:(< ?vy 0.01)) (omega ?w&:(< ?w 0.01))
  )
  (navigator-default-vmax (velocity ?trans-vmax) (rotation ?rot-vmax))
  (wm-fact (key domain fact self args? r ?r))
=>
  (retract ?st-f ?srch-f ?pa) ;?skill-f)
  (assert (state EXP_EXPLORE_ZONE))
  (navigator-set-speed ?trans-vmax ?rot-vmax)
  (assert (plan-action (id 1) (plan-id ?plan-id) (goal-id EXPLORATION) (action-name explore-zone) (param-values ?r ?zn) (status PENDING)))
  ;(skill-call explore_zone zone (str-cat ?zn)) TODO make this a plan action
)


(defrule exp-skill-explore-zone-final
  (goal (id EXPLORATION) (mode DISPATCHED))
  ?st-f <- (state EXP_EXPLORE_ZONE)
  ?pa <- (plan-action (action-name explore-zone) (status FINAL))
  ;?skill-f <- (skill-done (name "explore_zone") (status FINAL)) TODO this should be a plan-action
  (ZoneInterface (id "/explore-zone/info") (zone ?zn-str)
    (orientation ?orientation) (tag_id ?tag-id) (search_state YES)
  )
  (Position3DInterface (id "/explore-zone/found-tag")
    (frame ?frame) (translation $?trans) (rotation $?rot)
  )
  ; We don't check the visibility_history here since that's already done in the explore_zone skill
  (domain-fact (name tag-matching) (param-values ?machine ?side ?team-color ?tag-id))
  ;(tag-matching (tag-id ?tag-id) (machine ?machine) (side ?side) (team ?team-color))
  ?ze <- (zone-exploration (name ?zn2&:(eq ?zn2 (sym-cat ?zn-str))) (times-searched ?times-searched))
  ;(machine (name ?machine) (mtype ?mtype))
  (domain-fact (name mps-type) (param-values ?machine ?mtype))
  ?exp-f <- (explore-zone-target (zone ?zn))
=>
  (retract ?st-f ?exp-f ?pa); ?skill-f )
  ;(assert
  ;  (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource (sym-cat ?zn-str)))  TODO what about locks
  ;)
  (if (any-factp ((?ft found-tag)) (eq ?ft:name ?machine)) then
    (printout error "BUG: Tag for " ?machine " already found. Locking glitch or agent bug!" crlf)
  else
    ; Order is important here: Update state before zone-exploration to avoid endless loop.
    ;(synced-modify ?ze machine ?machine times-searched (+ 1 ?times-searched)) TODO now wm-facts
    ;(synced-assert (str-cat "(found-tag (name " ?machine ") (side " ?side ")"
    ;  "(frame \"map\") (trans " (implode$ ?trans) ") "
    ;  "(rot " (implode$ ?rot) ") )")
    ;) TODO synced-assert
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
  (goal (id EXPLORATION) (mode DISPATCHED))
  ?st-f <- (state EXP_EXPLORE_ZONE)
  ?pa <- (plan-action (action-name explore-zone) (status ?status))
  ;?skill-f <- (skill-done (name "explore_zone") (status ?status)) TODO this should be a skill
  ?exp-f <- (explore-zone-target (zone ?zn))
  (ZoneInterface (id "/explore-zone/info") (zone ?zn-str) (search_state ?s&:(neq ?s YES)))
  ?ze <- (zone-exploration (name ?zn2&:(eq ?zn2 (sym-cat ?zn-str))) (machine ?machine) (times-searched ?times-searched))
=>
  (retract ?st-f ?exp-f ?pa) ;?skill-f)
  (if (eq ?status FINAL) then
    (printout error "BUG in explore_zone skill: Result is FINAL but no MPS was found.")
  )
  (if (and (eq ?s NO) (eq ?machine UNKNOWN)) then
    ;(synced-modify ?ze machine NONE times-searched (+ ?times-searched 1)) TODO now wm-facts
  else
    ;(synced-modify ?ze line-visibility 0 times-searched (+ ?times-searched 1)) TODO now wm-facts
  )
  (assert
    ;(lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource (sym-cat ?zn-str)))  TODO what about locks
    (exp-searching)
    (state EXP_IDLE)
  )
)


(defrule exp-report-to-refbox
  (goal (id EXPLORATION) (mode DISPATCHED))
  (team-color ?color)
  (exploration-result (team ?color) (machine ?machine) (zone ?zone)
    (orientation ?orientation)
  )
  (time $?now)
  ?ws <- (timer (name send-machine-reports) (time $?t&:(timeout ?now ?t 1)) (seq ?seq))
  (game-time $?game-time)
  (confval (path "/clips-executive/specs/rcll2017/parameters/rcll/latest-send-last-report-time")
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

  (domain-fact (name tag-matching) (param-values ?machine ?side ?team-color ?tag))
;  (tag-matching (tag-id ?tag) (machine ?machine) (side ?side))
  (zone-exploration (name ?zn) (machine ?machine) (times-searched ?times-searched))

  (domain-fact (name tag-matching) (param-values ?machine2&:(eq ?machine2 (mirror-name ?machine)) ?side ?col ?tag2))
 ; (tag-matching (tag-id ?tag2) (side ?side)
  ;  (machine ?machine2&:(eq ?machine2 (mirror-name ?machine)))
  ;)

  (found-tag (name ?machine2&:(eq ?machine2 (mirror-name ?machine))))
  ?ze2 <- (zone-exploration (name ?zn2&:(eq ?zn2 (mirror-name ?zn))))
  ;(machine (name ?machine) (mtype ?mtype))
  (domain-fact (name mps-type) (param-values ?machine ?mtype))
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
  ;(synced-modify ?ze2 machine ?machine2 times-searched ?times-searched) TODO now wm-facts
)


(defrule exp-add-tag-to-navgraph
  (not (requested-waiting-positions))
  (or
    (not (last-navgraph-compute-msg))
    (last-navgraph-compute-msg (final TRUE))
    (NavGraphWithMPSGeneratorInterface (msgid 0))
    (not (NavGraphWithMPSGeneratorInterface))
  )
  ;(forall (machine (name ?mps))
  (forall (domain-fact (name mps-type) (param-values ?mps ?mps-type))
    (zone-exploration (machine ?found&:(eq ?mps ?found)))
  )

  (wm-fact (key refbox team-color) (value ?team-color))
  ;(team-color ?team-color)

  (found-tag (name ?machine)
    (side ?side) (frame ?) (trans $?) (rot $?)
  )
  (domain-fact (name tag-matching) (param-values ?machine ?side ?team-color ?tag))
 ; (tag-matching (tag-id ?tag) (team ?team-color)
 ;   (machine ?machine) (side ?side)
 ;; )
  (found-tag (name ?machine2&:(eq ?machine2 (mirror-name ?machine)))
    (side ?side) (frame ?) (trans $?) (rot $?)
  )
  (domain-fact (name tag-matching) (param-values ?machine2 ?side ?team-color2&:(neq ?team-color ?team-color2) ?tag2))
;  (tag-matching (tag-id ?tag2) (team ?team-color2&~?team-color)
;    (machine ?machine2) (side ?side)
;  )
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
  ?g <- (goal (id EXPLORATION) (mode DISPATCHED))
  (change-phase PRODUCTION)
=>
  ;(delayed-do-for-all-facts ((?l lock)) (eq ?l:type REFUSE)  TODO what about locks
  ;  (retract ?l)
  ;)
  (printout warn "empty effect" crlf)
  (modify ?g (mode FINISHED) (outcome COMPLETED))
)
