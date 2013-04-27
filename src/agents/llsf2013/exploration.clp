
;---------------------------------------------------------------------------
;  exploration.clp - Robotino agent for exploration phase
;
;  Created: Fri Apr 26 18:38:18 2013 (Magdeburg)
;  Copyright  2013  Frederik Zwilling
;             2013  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------


;should the agent go clockwise or anticlockwise?
;machine name, coordinates, next machine in exploration cycle
(defrule exp-config-path-clockwise
  (confval (path "/clips-agent/llsf2013/exploration-agent-cycle-clockwise") (value true))
  (confval (path "/clips-agent/llsf2013/double-exploration") (value ?double-exploration))
  =>
  (printout t "Driving clockwise" crlf)
  (if (eq ?double-exploration false)
    then
    (assert 
      (machine-exploration (name M10) (x 2.18) (y 4.74) (next M7) (look-pos M10))
      (machine-exploration (name M9) (x 1.38) (y 3.42) (next M10) (look-pos M9))
      (machine-exploration (name M8) (x 1.38) (y 2.18) (next M9) (look-pos M8))
      (machine-exploration (name M7) (x 2.5) (y 4.5) (next M6) (look-pos M7))
      (machine-exploration (name M6) (x 3.1) (y 4.42) (next M2) (look-pos M6))
      (machine-exploration (name M5) (x 2.3) (y 3.1) (next M4) (look-pos M5))
      (machine-exploration (name M4) (x 3.1) (y 2.13) (next M1) (look-pos M4))
      (machine-exploration (name M3) (x 3.1) (y 1.06) (next M8) (look-pos M3))
      (machine-exploration (name M2) (x 4.42) (y 3.62) (next M5) (look-pos M2))
      (machine-exploration (name M1) (x 3.62) (y 1.18) (next M3) (look-pos M1))
    )
    else
;double
    (assert
      (machine-exploration (name M10) (x 2.18) (y 4.74) (next M9) (look-pos M10))
      (machine-exploration (name M9) (x 1.38) (y 3.42) (next R2) (look-pos M9))
      (machine-exploration (name M7) (x 2.5) (y 4.5) (next M6) (look-pos M7))
      (machine-exploration (name M6) (x 3.1) (y 4.42) (next M10) (look-pos M6))
      (machine-exploration (name M2) (x 4.42) (y 3.62) (next M7) (look-pos M2))
      (machine-exploration (name R2) (x 3.62) (y 1.18) (next R2) (look-pos R2))
      (second-robotino)
      (stille-ecke R2)
    )
  )
  (assert
    (first-exploration-machine M2)
  )
)

(defrule exp-config-path-counter-clockwise
  (confval (path "/clips-agent/llsf2013/exploration-agent-cycle-clockwise") (value false))
  (confval (path "/clips-agent/llsf2013/double-exploration") (value ?double-exploration))
  =>
  (printout t "Driving anti-clockwise" crlf)
  (if (eq ?double-exploration false)
    then
    (assert 
      (machine-exploration (name M10) (x 2.18) (y 4.74) (next M9) (look-pos M10))
      (machine-exploration (name M9) (x 1.38) (y 3.42) (next M8) (look-pos M9))
      (machine-exploration (name M8) (x 1.38) (y 2.18) (next M3) (look-pos M8))
      (machine-exploration (name M7) (x 2.5) (y 4.5) (next M10) (look-pos M7))
      (machine-exploration (name M6) (x 3.1) (y 4.42) (next M7) (look-pos M6))
      (machine-exploration (name M5) (x 2.3) (y 3.1) (next M2) (look-pos M5))
      (machine-exploration (name M4) (x 3.1) (y 2.13) (next M5) (look-pos M4))
      (machine-exploration (name M3) (x 3.1) (y 1.06) (next M1) (look-pos M3))
      (machine-exploration (name M2) (x 4.42) (y 3.62) (next M6) (look-pos M2))
      (machine-exploration (name M1) (x 3.62) (y 1.18) (next M4) (look-pos M1))
    )
    else
    (assert
      (machine-exploration (name M10) (x 2.18) (y 4.74) (next M9) (look-pos M10))
      (machine-exploration (name M9) (x 1.38) (y 3.42) (next M8) (look-pos M9))
      (machine-exploration (name M8) (x 1.38) (y 2.18) (next M3) (look-pos M8))
      (machine-exploration (name M7) (x 2.5) (y 4.5) (next M10) (look-pos M7))
      (machine-exploration (name M6) (x 3.1) (y 4.42) (next M7) (look-pos M6))
      (machine-exploration (name M5) (x 2.3) (y 3.1) (next M2) (look-pos M5))
      (machine-exploration (name M4) (x 3.1) (y 2.13) (next M5) (look-pos M4))
      (machine-exploration (name M3) (x 3.1) (y 1.06) (next M1) (look-pos M3))
      (machine-exploration (name M2) (x 4.42) (y 3.62) (next M6) (look-pos M2))
      (machine-exploration (name M1) (x 3.62) (y 1.18) (next M4) (look-pos M1))
      (first-robotino)
    )
  )
  (assert
    (first-exploration-machine M8)
  )
)


;Set up the status
;there are two rounds. In the first the robotino drives to each machine in a defined cycle. After the first round the robotino drives to unrecognized machines again.
(defrule exp-start
  (phase EXPLORATION)
  ?st  <- (exploration-start)
  (time $?now)
  =>
  (retract ?st)
  (assert (status "start"))
  (assert (status "firstround"))
  (assert (status "explorationRunning"))
  (assert (signal (type send-machine-reports)))
  (assert (signal (type print-unrecognized-lights)))
  (assert (signal (type in-die-stille-ecke-command)))
  (printout t "Yippi ka yeah. I am in the exploration-phase." crlf)
)

;(defrule test
;  (RobotinoLightInterface (id "Light_State") (red ?red) (yellow ?yellow) (green ?green) (ready ?))
;  (time $?now)
;  =>
;  (printout t "GOT PLUGIN LIGHT INTERFACE" crlf)
;)

;Robotino drives to the first machine to start the first round
(defrule exp-goto-first
  (phase EXPLORATION)
  ?s <- (status "start")
  (status "firstround")
  ?first-machine <- (first-exploration-machine ?v)
  (machine-exploration (name ?v) (x ?) (y ?) (next ?) (look-pos ?lp))
  =>
  (printout t "First machine:" ?v crlf)
  (skill-call ppgoto place (str-cat ?lp))
  (retract ?s ?first-machine)
  (assert (status "drivingToMachine"))
  (assert (goalmachine ?v))
  (assert (startmachine ?v))
)

;arriving at a machine in first or second round. Preparing recognition of the light signals
(defrule exp-arrived-at-machine
  (phase EXPLORATION)
  ?final <- (skill (name "ppgoto") (status FINAL) (skill-string ?skill)) 
  ?s <- (status "drivingToMachine")
  (time $?now)
  =>
  (printout t "arrived. skill string was: " ?skill crlf)
  (printout t "Read light now" crlf)
  (retract ?s ?final)
  (assert (status "waitingAtMachine"))
  (assert (signal (type waiting-since) (time ?now) (seq 1)))

  ;;;;;;; this  should be the waiting skill ;;;;;
  ;(skill-call relgoto rel_x 0 rel_y 0)
)

;Recognizing succseded => memorize light-signals for further rules and prepare to drive to the next machine
(defrule exp-read-light-at-machine
  (phase EXPLORATION)
  ;;;;;;; change relgoto to waiting skill ;;;;;
  ;?final <- (skill (name "relgoto") (status FINAL) (skill-string ?skill)) 
  (time $?now)
  ?ws <- (signal (type waiting-since))
  ?s <- (status "waitingAtMachine")
  ?g <- (goalmachine ?old)
  (machine-exploration (name ?old) (x ?) (y ?) (next ?nextMachine))
  ?rli <- (RobotinoLightInterface (id "Light_State") (red ?red) (yellow ?yellow) (green ?green) (visibility_history ?vh&:(> ?vh 20)) (ready TRUE))

  =>

  ;(printout warn "Read light ready: " ?ready " red: " ?red " yellow: " ?yellow " green: " ?green crlf)
  (printout t "Read light red: " ?red " yellow: " ?yellow " green: " ?green crlf)
  (retract ?s)  
  (retract ?g)  
  ;(retract ?final)
  (retract ?rli)
  (retract ?ws)
  (assert (status "idle"))
  (assert (nextInCycle ?nextMachine))
  (assert (machine-light (name ?old) (red ?red) (yellow ?yellow) (green ?green)))
  (assert (blocked ?old ?now))
  (printout t "asserted block of " ?old crlf)
)

(defrule exp-test-light
  (phase EXPLORATION)
  (status "waitingAtMachine")
  ?rli <- (RobotinoLightInterface (id "Light_State") (red ?red) (yellow ?yellow) (green ?green) (ready ?ready))
  =>
  (printout t "WAITING AT MACHINE" crlf)
  (printout t "See light red: " ?red " yellow: " ?yellow " green: " ?green " ready: " ?ready crlf)
)

;Matching of recognized lights to the machine types
(defrule exp-match-light-types
  (phase EXPLORATION)
  ?ml <- (machine-light (name ?name) (red ?red) (yellow ?yellow) (green ?green))
  (matching-type-light (type ?type) (red ?red) (yellow ?yellow) (green ?green))
  =>
  (assert (machine-type (name ?name) (type ?type)))
  (assert (machineRecognized ?name))
  (printout t "Identified machine" crlf)
  (retract ?ml)
)

;Sending all results to the refbox every second
(defrule exp-send-recognized-machines "Tell the refbox"
  (phase EXPLORATION)
  (time $?now)
  ?ws <- (signal (type send-machine-reports) (time $?t&:(timeout ?now ?t 0.5)) (seq ?seq))
  
  =>
  (bind ?mr (pb-create "llsf_msgs.MachineReport"))

  (printout t "Ich sende jetzt folgende Maschinen:" crlf)
  (do-for-all-facts ((?machine machine-type)) TRUE
    (bind ?mre (pb-create "llsf_msgs.MachineReportEntry"))
    (pb-set-field ?mre "name" (str-cat ?machine:name))
    (pb-set-field ?mre "type" (str-cat ?machine:type))
    (pb-add-list ?mr "machines" ?mre)
    (printout t "Maschine " ?machine:name ", Typ " ?machine:type crlf)
  )

  (pb-broadcast ?mr)
  (modify ?ws (time ?now) (seq (+ ?seq 1)))
)

(defrule exp-print-read-but-not-recognized-lights
  (phase EXPLORATION)
  (time $?now)  
  ?ws <- (signal (type print-unrecognized-lights) (time $?t&:(timeout ?now ?t 1.0)) (seq ?seq))
  =>
  (do-for-all-facts ((?read machine-light)) TRUE
    (printout t "Read light with no type matching: " ?read:name ", red " ?read:red ", yellow " ?read:yellow ", green " ?read:green crlf)
  )
  (modify ?ws (time ?now) (seq (+ ?seq 1)))
)

(defrule exp-all-matchings-are-there
  (phase EXPLORATION)
  (matching-type-light (type T1) (red ?) (yellow ?) (green ?)) 
  (matching-type-light (type T2) (red ?) (yellow ?) (green ?))
  (matching-type-light (type T3) (red ?) (yellow ?) (green ?))
  (matching-type-light (type T4) (red ?) (yellow ?) (green ?))
  (matching-type-light (type T5) (red ?) (yellow ?) (green ?))
  =>
  (assert (have-all-matchings))
)

;Recieve light-pattern-to-type matchig and save it in a fact
(defrule exp-recieve-type-light-pattern-matching
  (phase EXPLORATION)
  ?pbm <- (protobuf-msg (type "llsf_msgs.ExplorationInfo") (ptr ?p) (rcvd-via BROADCAST))
  (not (have-all-matchings))

  =>
  
  ;(printout t "GOT MESSAGE FROM REFBOX (EXPLORATION-INFO)" crlf)
  (retract ?pbm)
  (foreach ?sig (pb-field-list ?p "signals")
    (bind ?type (pb-field-value ?sig "type"))
    (progn$ (?light (pb-field-list ?sig "lights"))
      (bind ?light-color (pb-field-value ?light "color"))
      (bind ?light-state (pb-field-value ?light "state"))
      ;assert the read type color and state to compose it together in compose-type-light-pattern-matching 
      (assert (type-spec-pre ?type ?light-color ?light-state))
    )
  )
)

;the refbox sends BLINK but in the interface BLINKING is used. This rule converts BLINK to BLINKING
(defrule exp-convert-blink-to-blinking
  (declare (salience 10)) ; this rule has to fire before compose-type-light-pattern-matching
  (phase EXPLORATION)
  ?tsp <- (type-spec-pre ?type ?light-color BLINK)
  =>
  ;(retract ?tsp)
  (assert (type-spec-pre ?type ?light-color BLINKING))
)

;Compose information sent by the refbox as one
(defrule exp-compose-type-light-pattern-matching
  (declare (salience 0));this rule has to fire after convert-blink-to-blinking
  (phase EXPLORATION)
  ?r <- (type-spec-pre ?type RED ?red-state)
  ?y <- (type-spec-pre ?type YELLOW ?yellow-state)
  ?g <- (type-spec-pre ?type GREEN ?green-state)
  =>
  (assert (matching-type-light (type ?type) (red ?red-state) (yellow ?yellow-state) (green ?green-state)))
  (retract ?r ?y ?g)
)

;Recognizing of lights failed => drive to next mashine or retry (depending on the round)
(defrule exp-recognized-machine-failed
  ;?final <- (skill (name "relgoto") (status FAILED) (skill-string ?skill))
  (phase EXPLORATION)
  (time $?now)
  ?ws <- (signal (type waiting-since) (time $?t&:(timeout ?now ?t 5.0)))
  ?s <- (status "waitingAtMachine")
  ?g <- (goalmachine ?old)
  (machine-exploration (name ?old) (x ?) (y ?) (next ?nextMachine))

  =>
 
  (printout t "Reading light at " ?old " failed." crlf)
  (printout t "Waited 5 seconds on RobotinoLightInterface with ready = TRUE" crlf)
  (retract ?s)  
  (retract ?g)
  (retract ?ws)  
  ;(retract ?final)
  (assert (status "idle"))
  (assert (nextInCycle ?nextMachine))
  (assert (blocked ?old ?now))
  (printout t "asserted block of " ?old crlf)
)

;Find next machine and assert drinve command in the first round
(defrule exp-goto-next-machine-first-round
  (phase EXPLORATION)
  (status "firstround")
  ?s <- (status "idle")
  ?n <- (nextInCycle ?nextMachine)
  (not (machineRecognized ?nextMachine))
  (machine-exploration (name ?nextMachine) (x ?) (y ?) (next ?) (look-pos ?lp))

  =>

  (printout t "Going to next machine" crlf)
  (retract ?s)
  (retract ?n)
  (assert (status "drivingToMachine"))
  (assert (goalmachine ?nextMachine))

  (skill-call ppgoto place (str-cat ?lp))
)

;finish the first round and begin retry round
(defrule exp-finish-first-round
  (phase EXPLORATION)
  ?r <- (status "firstround")
  ?s <- (status "idle")
  ?n <- (nextInCycle ?nextMachine)
  (machineRecognized ?nextMachine)
  =>
  (printout t "Finished first round" crlf)
  (retract ?s)
  (retract ?r) 
  (retract ?n) 
  (assert (status "retryRound"))
  (assert (status "idle"))
)

;should be unnecessary but hack for safety
(defrule exp-finish-first-round-and-flee
  ?sr <- (status "retryRound")
  ?si <- (status "idle")
  (second-robotino)
  (stille-ecke ?l)
  =>
  (skill-call ppgoto place (str-cat ?l))
  (retract ?sr ?si)
  (assert (end-status "flee"))  
)

(deffunction machine-is-closer (?x ?y ?x2 ?y2 $?pos)
  (if (<= (+ (* (- ?x (nth$ 1 ?pos)) (- ?x (nth$ 1 ?pos))) (* (- ?y (nth$ 2 ?pos)) (- ?y (nth$ 2 ?pos)))) (+ (* (- ?x2 (nth$ 1 ?pos)) (- ?x2 (nth$ 1 ?pos))) (* (- ?y2 (nth$ 2 ?pos)) (- ?y2 (nth$ 2 ?pos)))))
    then
      TRUE
    else
      FALSE
  )
)

;Drive to the nearest unrecognized machine in the retry round
(defrule exp-retry-nearest-unrecognized
  (phase EXPLORATION)
  (status "retryRound")
  ?s <- (status "idle")
  (machine-exploration (name ?m) (x ?x) (y ?y) (next ?) (look-pos ?lp))
  (not (machineRecognized ?m))
  (not (blocked ?m $?))
  (machine-exploration (name ?m2&~?m))
  (not (machineRecognized ?m2))
  (Position3DInterface (id "Pose") (translation $?pos))
  (machine-exploration (name ?m2) (x ?x2) (y ?y2&~:(machine-is-closer ?x2 ?y2 ?x ?y ?pos)))

  =>

  (printout t "Retry machine" crlf)
  (retract ?s)
  (assert (status "drivingToMachine"))
  (assert (goalmachine ?m))

  (skill-call ppgoto place (str-cat ?lp))
)

;Do not retry the recently failed machine, free the block here
(defrule exp-free-blocked
  (phase EXPLORATION)
  (time $?now)
  ?b <- (blocked ?m $?t&:(timeout ?now ?t 0.2))
  =>
  (retract ?b)
  (printout t "Removed blocking of " ?m crlf)
)


;Finish exploration phase if all machines are recognized
(defrule exp-finish-exploration
  (phase EXPLORATION)
  (forall (machine-exploration (name ?m) (x ?) (y ?) (next ?))
    (machineRecognized ?m))
  ?s <- (status "retryRound")
  
  =>

  (printout t "Finished Exploration :-)" crlf)
  (retract ?s)
  (assert (status "finishedExploration"))
)

(defrule exp-move-second-agent-away-finished
  (declare (salience ?*PRIORITY-HIGH*))
  (status "finishedExploration")
  (second-robotino)
  =>
  (assert (end-status "flee"))
)
(defrule exp-move-second-agent-away-prod-started
  (declare (salience ?*PRIORITY-HIGH*))
  (phase PRODUCTION)
  (second-robotino)
  =>
  (assert (end-status "flee"))
)

(defrule exp-in-die-stille-ecke
  (declare (salience ?*PRIORITY-HIGH*))
  (second-robotino)
  (end-status "flee")
  (stille-ecke ?l)
  =>
  (skill-call ppgoto place (str-cat ?l))
)
(defrule exp-remove-phases
  (declare (salience ?*PRIORITY-HIGH*))
  (second-robotino)
  (end-status "flee")
  ?p <- (phase ?)
  (time $?)
  =>
  (retract ?p)
)
(defrule exp-remove-status
  (declare (salience ?*PRIORITY-HIGH*))
  (second-robotino)
  (end-status "flee")
  ?s <- (status ?)
  (time $?)
  =>
  (retract ?s)
)

