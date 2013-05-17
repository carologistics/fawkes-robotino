
;---------------------------------------------------------------------------
;  exploration.clp - Robotino agent for exploration phase
;
;  Created: Fri Apr 26 18:38:18 2013 (Magdeburg)
;  Copyright  2013  Frederik Zwilling
;             2013  Alexander von Wirth 
;             2013  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------


;Determine the first machine in dependency of the role
(defrule exp-determine-first-machine
  (phase EXPLORATION)
  (role ?role)
  (confval (path "/clips-agent/llsf2013/start-machine-exploration-only") (value ?first-machine-exp-only))
  (confval (path "/clips-agent/llsf2013/start-machine-exploration-production") (value ?first-machine-exp-prod))
  =>
  (if (eq ?role EXPLORATION_ONLY)
    then
      (assert (first-exploration-machine (sym-cat ?first-machine-exp-only)))
    else
      (if (eq ?role EXPLORATION_PRODUCTION)
				then
					(assert (first-exploration-machine (sym-cat ?first-machine-exp-prod)))
				else
					(printout t "UNKNOWN ROLE! UNKNOWN ROLE! UNKNOWN ROLE! UNKNOWN ROLE! ")
      )
  )
)

;Set up the state
;there are two rounds. In the first the robotino drives to each machine in a defined cycle. After the first round the robotino drives to unrecognized machines again.
(defrule exp-start
  (phase EXPLORATION)
  ?st <- (exploration-start)
  =>
  (retract ?st)
  (assert (state EXP_START)
          (round FIRST)
          (signal (type send-machine-reports))
          (signal (type print-unrecognized-lights))
  )
  (printout t "Yippi ka yeah. I am in the exploration-phase." crlf)
)

;(defrule test
;  =>
;)

;Robotino drives to the first machine to start the first round
(defrule exp-goto-first
  (phase EXPLORATION)
  ?s <- (state EXP_START)
  (round FIRST)
  ?first-machine <- (first-exploration-machine ?v)
  (machine-exploration (name ?v) (x ?) (y ?) (next ?) (look-pos ?lp))
  =>
  (printout t "First machine: " ?v crlf)
  (skill-call ppgoto place (str-cat ?lp))
  (retract ?s)
  (assert (state EXP_DRIVING_TO_MACHINE)
          (goalmachine ?v))
)

;arriving at a machine in first or second round. Preparing recognition of the light signals
(defrule exp-arrived-at-machine
  (phase EXPLORATION)
  ?final <- (skill (name "ppgoto") (status FINAL) (skill-string ?skill)) 
  ?s <- (state EXP_DRIVING_TO_MACHINE)
  (time $?now)
  =>
  (printout t "Arrived. Skill string was: " ?skill crlf)
  (printout t "Read light now." crlf)
  (retract ?s ?final)
  (assert (state EXP_WAITING_AT_MACHINE)
          (signal (type waiting-since) (time ?now) (seq 1))
  )
)

;Recognizing succeeded => memorize light-signals for further rules and prepare to drive to the next machine
(defrule exp-read-light-at-machine
  (phase EXPLORATION)
  (time $?now)
  ?ws <- (signal (type waiting-since))
  ?s <- (state EXP_WAITING_AT_MACHINE)
  ?g <- (goalmachine ?old)
  (machine-exploration (name ?old) (x ?) (y ?) (next ?nextMachine))
  ?rli <- (RobotinoLightInterface (id "Light_State") (red ?red) (yellow ?yellow) (green ?green) (visibility_history ?vh&:(> ?vh 20)) (ready TRUE))
  =>
  (printout t "Read light: red: " ?red " yellow: " ?yellow " green: " ?green crlf)
  (retract ?s ?g ?rli ?ws)
  (assert (state EXP_IDLE)
          (nextInCycle ?nextMachine)
          (machine-light (name ?old) (red ?red) (yellow ?yellow) (green ?green))
  )
)

;Matching of recognized lights to the machine types
(defrule exp-match-light-types
  (phase EXPLORATION)
  ?ml <- (machine-light (name ?name) (red ?red) (yellow ?yellow) (green ?green))
  (matching-type-light (type ?type) (red ?red) (yellow ?yellow) (green ?green))
  =>
  (printout t "Identified machine" crlf)
  (retract ?ml)
  (assert (machine-type (name ?name) (type ?type))
  )
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

;Receive light-pattern-to-type matchig and save it in a fact
(defrule exp-receive-type-light-pattern-matching
  (phase EXPLORATION)
  ?pbm <- (protobuf-msg (type "llsf_msgs.ExplorationInfo") (ptr ?p) (rcvd-via BROADCAST))
  (not (have-all-matchings))
  =>
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

(defrule exp-receive-recognized-machines
  (phase EXPLORATION)
  ?pbm <- (protobuf-msg (type "llsf_msgs.MachineReportInfo") (ptr ?p))
  =>
  (retract ?pbm)
  (foreach ?machine (pb-field-list ?p "reported_machines")
    (assert (machineRecognized ?machine))
    (printout t "Ich habe folgende Maschine bereits erkannt: " ?machine crlf)
  )
)

;the refbox sends BLINK but in the interface BLINKING is used. This rule converts BLINK to BLINKING
(defrule exp-convert-blink-to-blinking
  (declare (salience 10)) ; this rule has to fire before compose-type-light-pattern-matching
  (phase EXPLORATION)
  (type-spec-pre ?type ?light-color BLINK)
  =>
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
  (retract ?r ?y ?g)
  (assert (matching-type-light (type ?type) (red ?red-state) (yellow ?yellow-state) (green ?green-state)))
)

;Recognizing of lights failed => drive to next mashine or retry (depending on the round)
(defrule exp-recognized-machine-failed
  (phase EXPLORATION)
  (time $?now)
  ?ws <- (signal (type waiting-since) (time $?t&:(timeout ?now ?t 5.0)))
  ?s <- (state EXP_WAITING_AT_MACHINE)
  ?g <- (goalmachine ?old)
  (machine-exploration (name ?old) (x ?) (y ?) (next ?nextMachine))
  =>
  (printout t "Reading light at " ?old " failed." crlf)
  (printout t "Waited 5 seconds on RobotinoLightInterface with ready = TRUE." crlf)
  (retract ?s ?g ?ws)
  (assert (state EXP_IDLE)
          (nextInCycle ?nextMachine)
  )
)

;Find next machine and assert drinve command in the first round
(defrule exp-goto-next-machine-first-round
  (phase EXPLORATION)
  (round FIRST)
  ?s <- (state EXP_IDLE)
  ?n <- (nextInCycle ?nextMachine)
  (not (machineRecognized ?nextMachine))
  (machine-exploration (name ?nextMachine) (x ?) (y ?) (next ?) (look-pos ?lp))
  =>
  (printout t "Going to next machine" crlf)
  (retract ?s ?n)
  (assert (state EXP_DRIVING_TO_MACHINE)
          (goalmachine ?nextMachine)
  )
  (skill-call ppgoto place (str-cat ?lp))
)

;finish the first round and begin retry round
(defrule exp-finish-first-round
  (phase EXPLORATION)
  ?r <- (round FIRST)
  ?s <- (state EXP_IDLE)
  ?n <- (nextInCycle ?nextMachine)
  (machineRecognized ?nextMachine)
  =>
  (printout t "Finished first round" crlf)
  (retract ?s ?r ?n) 
  (assert (round RETRY)
          (state EXP_IDLE)
  )
)

;should be unnecessary but hack for safety
(defrule exp-finish-first-round-and-flee
  ?sr <- (round RETRY)
  ?si <- (state EXP_IDLE)
  (second-robotino)
  (stille-ecke ?l)
  =>
  (skill-call ppgoto place (str-cat ?l))
  (retract ?sr ?si)
  (assert (end-state FLEE))  
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
  (round RETRY)
  ?s <- (state EXP_IDLE)
  (machine-exploration (name ?m) (x ?x) (y ?y) (next ?) (look-pos ?lp))
  (not (machineRecognized ?m))
  (machine-exploration (name ?m2&~?m))
  (not (machineRecognized ?m2))
  (Position3DInterface (id "Pose") (translation $?pos))
  (machine-exploration (name ?m2) (x ?x2) (y ?y2&~:(machine-is-closer ?x2 ?y2 ?x ?y ?pos)))
  =>
  (printout t "Retry machine" crlf)
  (retract ?s)
  (assert (state EXP_DRIVING_TO_MACHINE)
          (goalmachine ?m)
  )
   (skill-call ppgoto place (str-cat ?lp))
)


;Finish exploration phase if all machines are recognized
(defrule exp-finish-exploration
  (phase EXPLORATION)
  (forall (machine-exploration (name ?m) (x ?) (y ?) (next ?))
    (machineRecognized ?m))
  ?s <- (round RETRY)
  =>
  (printout t "Finished Exploration :-)" crlf)
  (retract ?s)
  (assert (state EXP_FINISHED_EXPLORATION))
)

(defrule exp-goto-input-after-exploration
  (phase EXPLORATION)
  (state EXP_FINISHED_EXPLORATION)
  (role EXPLORATION_PRODUCTION)
  (not (driven-to-insertion))
  (confval (path "/clips-agent/llsf2013/waiting-for-production-point") (value ?waiting-for-prod-point))
  =>
  (printout t "Finished Exploration - Driving to waiting-point" crlf)
  (assert (driven-to-insertion))
  (skill-call ppgoto place (str-cat ?waiting-for-prod-point))
)

(defrule exp-move-second-agent-away-finished
  (declare (salience ?*PRIORITY-HIGH*))
  (state EXP_FINISHED_EXPLORATION)
  (role EXPLORATION_ONLY)
  =>
  (assert (end-state FLEE))
)
(defrule exp-move-second-agent-away-prod-started
  (declare (salience ?*PRIORITY-HIGH*))
  (phase PRODUCTION)
  (role EXPLORATION_ONLY)
  =>
  (assert (end-state FLEE))
)

(defrule exp-in-die-stille-ecke
  (declare (salience ?*PRIORITY-HIGH*))
  (role EXPLORATION_ONLY)
  (end-state FLEE)
  (stille-ecke ?l)
  =>
  (skill-call ppgoto place (str-cat ?l))
)
(defrule exp-remove-phases
  (declare (salience ?*PRIORITY-HIGH*))
  (role EXPLORATION_ONLY)
  (end-state FLEE)
  ?p <- (phase ?)
  (time $?)
  =>
  (retract ?p)
)
(defrule exp-remove-state
  (declare (salience ?*PRIORITY-HIGH*))
  (role EXPLORATION_ONLY)
  (end-state FLEE)
  ?s <- (state ?)
  (time $?)
  =>
  (retract ?s)
)

