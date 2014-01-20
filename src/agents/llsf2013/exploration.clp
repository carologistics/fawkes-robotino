
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
  (confval (path "/clips-agent/llsf2013/start-machine-exploration-p1p2") (value ?first-machine-exp-only))
  (confval (path "/clips-agent/llsf2013/start-machine-exploration-p3") (value ?first-machine-exp-prod))
  =>
  (if (eq ?role EXPLORATION_P1P2)
    then
      (assert (first-exploration-machine (sym-cat ?first-machine-exp-only)))
    else
      (if (eq ?role EXPLORATION_P3)
				then
					(assert (first-exploration-machine (sym-cat ?first-machine-exp-prod)))
				else
					(printout t "UNKNOWN ROLE! UNKNOWN ROLE! UNKNOWN ROLE! UNKNOWN ROLE! " crlf)
      )
  )
)

;Set up the state
;There are two rounds. In the first the robotino drives to each machine in a defined cycle. After the first round the robotino drives to unrecognized machines again.
(defrule exp-start
  (phase EXPLORATION)
  ?st <- (exploration-start)
  =>
  (retract ?st)
  (assert (state EXP_START)
          (round FIRST)
          (timer (name send-machine-reports))
          (timer (name print-unrecognized-lights))
  )
  (printout t "Yippi ka yeah. I am in the exploration-phase." crlf)
)

;Robotino drives to the first machine to start the first round
(defrule exp-goto-first
  (phase EXPLORATION)
  ?s <- (state EXP_START)
  (round FIRST)
  (first-exploration-machine ?v)
  (machine-exploration (name ?v) (x ?) (y ?) (next ?) (look-pos ?lp))
  (not (driven-to-waiting-point))
  =>
  (printout t "First machine: " ?v crlf)
  ;(skill-call ppgoto place (str-cat ?lp))
  (retract ?s)
  ;(assert (state EXP_DRIVING_TO_MACHINE)
  ;        (goalmachine ?v))
	(assert (state EXP_LOCK_REQUIRED)
					(lock (type GET) (agent ?*ROBOT-NAME*) (resource ?v))
					(nextInCycle ?v)
	)
)

;arriving at a machine in first or second round. Preparing recognition of the light signals
(defrule exp-ppgoto-arrived-at-machine
  (phase EXPLORATION)
  ?final <- (skill (name "ppgoto") (status FINAL)) 
  ?s <- (state EXP_DRIVING_TO_MACHINE)
  (goalmachine ?name)
  (machine-exploration (name ?name) (look-pos ?goal))
  (not (driven-to-waiting-point))
  =>
  (printout t "PPGoto Arrived. Calling global_motor_move." crlf)
  (retract ?s ?final)
  (assert (state EXP_DRIVING_TO_MACHINE_GLOBAL))
  (skill-call global_motor_move place (str-cat ?goal))
)

;redo movement with global motor move
(defrule exp-global-motor-move-finished
  (phase EXPLORATION)
  ?final <- (skill (name "global_motor_move") (status FINAL|FAILED)) 
  ?s <- (state EXP_DRIVING_TO_MACHINE_GLOBAL)
  (time $?now)
  =>
  (printout t "Arrived." crlf)
  (printout t "Read light now." crlf)
  (retract ?s ?final)
  (assert (state EXP_WAITING_AT_MACHINE)
          (timer (name waiting-since) (time ?now) (seq 1))
  )
)

;Recognizing succeeded => memorize light-signals for further rules and prepare to drive to the next machine
(defrule exp-read-light-at-machine
  (phase EXPLORATION)
  (time $?now)
  ?ws <- (timer (name waiting-since))
  ?s <- (state EXP_WAITING_AT_MACHINE)
  ?g <- (goalmachine ?old)
  (machine-exploration (name ?old) (x ?) (y ?) (next ?nextMachine))
  ?rli <- (RobotinoLightInterface (id "Light_State") (red ?red) (yellow ?yellow) (green ?green) (visibility_history ?vh&:(> ?vh 20)) (ready TRUE))
  (matching-type-light (type ?type) (red ?red) (yellow ?yellow) (green ?green))
  =>
  (printout t "Identified machine" crlf)
  (printout t "Read light: red: " ?red " yellow: " ?yellow " green: " ?green crlf)
  (retract ?s ?g ?rli ?ws)
  (assert (state EXP_IDLE)
    (nextInCycle ?nextMachine)
    (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?old))
    (machine-type (name ?old) (type ?type))
  )
)

;Recognizing of lights failed => ty again with a slightly other position
(defrule exp-recognized-machine-failed-once
  (phase EXPLORATION)
  (time $?now)
  ?ws <- (timer (name waiting-since) (time $?t&:(timeout ?now ?t 5.0)))
  ?s <- (state EXP_WAITING_AT_MACHINE)
  ?g <- (goalmachine ?old)
  (machine-exploration (name ?old) (x ?) (y ?) (next ?nextMachine))
  (not (second-recognize-try))
  =>
  (printout t "Reading light at " ?old " failed first time." crlf)
  (printout t "Try again in from an other position." crlf)
  (assert (second-recognize-try)
  )
  (modify ?ws (time ?now))
  (skill-call motor_move x 0 y 0.15 vel_trans 0.1)
)

;Recognizing of lights failed => drive to next mashine or retry (depending on the round)
(defrule exp-recognized-machine-failed-twice
  (phase EXPLORATION)
  (time $?now)
  ?ws <- (timer (name waiting-since) (time $?t&:(timeout ?now ?t 5.0)))
  ?s <- (state EXP_WAITING_AT_MACHINE)
  ?g <- (goalmachine ?old)
  (machine-exploration (name ?old) (x ?) (y ?) (next ?nextMachine))
  ?srt <- (second-recognize-try)
  =>
  (printout t "Reading light at " ?old " failed." crlf)
  (printout t "Waited 5 seconds on RobotinoLightInterface with ready = TRUE." crlf)
  (retract ?s ?g ?ws ?srt)
  (assert (state EXP_IDLE)
          (nextInCycle ?nextMachine)
					(lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?old))
  )
)

;delete second-recognize-try fact
(defrule exp-second-retry-fact-retract
  (state EXP_DRIVING_TO_MACHINE)
  ?srt <- (second-recognize-try)
  =>
  (retract ?srt)
)

;Find next machine and assert drinve command in the first round
(defrule exp-find-next-machine-first-round
  (phase EXPLORATION)
  (round FIRST)
  ?s <- (state EXP_IDLE)
  (nextInCycle ?nextMachine)
  (not (machineRecognized ?nextMachine))
  =>
  (retract ?s)
  (assert (state EXP_FOUND_NEXT_MACHINE))
)

;If next machine is already recognized, find next unrecognized machine in list
(defrule exp-find-next-machine-if-recognized-first-round
  (phase EXPLORATION)
  (round FIRST)
  (state EXP_IDLE)
  ?n <- (nextInCycle ?nextMachine)
  (machineRecognized ?nextMachine)
  (machine-exploration (name ?nextMachine) (x ?) (y ?) (next ?nextnext) (look-pos ?lp))
  (not (first-exploration-machine ?nextMachine))
  =>
  (retract ?n)
  (assert (nextInCycle ?nextnext))
)

;Require resource-locking
(defrule exp-require-resource-locking
  (phase EXPLORATION)
  (round FIRST)
  ?s <- (state EXP_FOUND_NEXT_MACHINE)
  (nextInCycle ?nextMachine)
  =>
  (printout t "Require lock for " ?nextMachine crlf)
  (retract ?s)
  (assert (state EXP_LOCK_REQUIRED)
	  (lock (type GET) (agent ?*ROBOT-NAME*) (resource ?nextMachine)))
)

;Wait for answer from MASTER
(defrule exp-check-resource-locking
  (phase EXPLORATION)
  (round FIRST)
  ?s <- (state EXP_LOCK_REQUIRED)
  (nextInCycle ?nextMachine)
  ?l <- (lock (type ACCEPT) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?nextMachine))
  =>
  (printout t "Lock accepted." crlf)
  (retract ?s ?l)
  (assert (state EXP_LOCK_ACCEPTED))
)

;Pass slow robotino
(defrule exp-pass-slow-robotino
  (phase EXPLORATION)
  (round FIRST)
  (state EXP_LOCK_REQUIRED)
  ?n <- (nextInCycle ?next)
  (machine-exploration (name ?next) (next ?second-next))
  (machine-exploration (name ?second-next) (next ?third-next))
  (not (machineRecognized ?next))
  (not (machineRecognized ?second-next))
  (not (machineRecognized ?third-next))
  ?lg <- (lock (type GET) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?nextMachine))
  ?lr <- (lock (type REFUSE) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?nextMachine))
  =>
  (printout warn "Passing slow robotino. Going for " ?third-next crlf)
  (retract ?n ?lg ?lr)
  (assert (nextInCycle ?third-next)
	  (lock (type GET) (agent ?*ROBOT-NAME*) (resource ?third-next))
  )
)

;Drive to next machine
(defrule exp-go-to-next-machine-first-round
  (phase EXPLORATION)
  (round FIRST)
  ?s <- (state EXP_LOCK_ACCEPTED)
  ?n <- (nextInCycle ?nextMachine)
  (machine-exploration (name ?nextMachine) (x ?) (y ?) (next ?) (look-pos ?lp))
  (not (driven-to-waiting-point))
  =>
  (printout t "Going to next machine." crlf)
  (retract ?s ?n)
  (assert (state EXP_DRIVING_TO_MACHINE)
          (goalmachine ?nextMachine))
  (skill-call ppgoto place (str-cat ?lp))
)

;Finish first round
(defrule exp-finish-first-round
  (phase EXPLORATION)
  ?r <- (round FIRST)
  (state EXP_IDLE)
  ?n <- (nextInCycle ?nextMachine)
  (first-exploration-machine ?nextMachine)
  =>
  (printout t "Finished first round." crlf)
  (retract ?r ?n) 
  (assert (round FIRST_FINISHED)
  )
)

;Start retry round if EXPLORATION_P1P2
(defrule exp-start-retry-round
  (phase EXPLORATION)
  ?r <- (round FIRST_FINISHED)
  (role EXPLORATION_P1P2)
  =>
  (printout t "Starting retry round." crlf)
	(retract ?r)
  (assert (round RETRY))
)

;Move EXPLORATION_P3 away after first round
(defrule exp-move-away-after-finished
  (declare (salience ?*PRIORITY-HIGH*))
  ?sr <- (round FIRST_FINISHED)
  ?si <- (state EXP_IDLE)
  (role EXPLORATION_P3)
  (not (driven-to-waiting-point))
  (confval (path "/clips-agent/llsf2013/waiting-for-p3-production-point") (value ?waiting-for-prod-point))
  =>
  (skill-call ppgoto place (str-cat ?waiting-for-prod-point))
  (printout t "Driving away..." crlf)
  (retract ?sr ?si)
  (assert (driven-to-waiting-point))  
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
(defrule exp-retry-unrecognized
  (phase EXPLORATION)
  (round RETRY)
  ?s <- (state EXP_IDLE)
  (machine-exploration (name ?m))
  (not (machineRecognized ?m))
  =>
  (assert (want-to-retry ?m))
)
(defrule retry-nearer-unrecognized
  (declare (salience ?*PRIORITY-HIGH*))
  (phase EXPLORATION)
  (round RETRY)
  ?r <- (want-to-retry ?m)  
  (Position3DInterface (id "Pose") (translation $?pos))
  (machine-exploration (name ?m) (x ?x) (y ?y) (next ?) (look-pos ?lp))
  (machine-exploration (name ?m2) (x ?x2) (y ?y2&~:(machine-is-closer ?x ?y ?x2 ?y2 ?pos)))
  (not (machineRecognized ?m2))
  =>
  (retract ?r)
  (assert (want-to-retry ?m2))
)
(defrule execute-retry
  (phase EXPLORATION)
  (round RETRY)
  ?s <- (state EXP_IDLE)
  ?r <- (want-to-retry ?m)
  (machine-exploration (name ?m) (x ?x) (y ?y) (next ?) (look-pos ?lp))
  =>
  (printout t "Retry machine" crlf)
  (retract ?s ?r)
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
  (role EXPLORATION_P1P2)
  ?s <- (round RETRY)
  (confval (path "/clips-agent/llsf2013/waiting-for-p1p2-prouction-point") (value ?work-finished-point))
  =>
  (printout t "Finished Exploration :-)" crlf)
  (retract ?s)
  (skill-call ppgoto place (str-cat ?work-finished-point))
  (printout t "Driving away." crlf)
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

;Sending all results to the refbox every second
(defrule exp-send-recognized-machines
  (phase EXPLORATION)
  (time $?now)
  ?ws <- (timer (name send-machine-reports) (time $?t&:(timeout ?now ?t 0.5)) (seq ?seq))
  =>
  (bind ?mr (pb-create "llsf_msgs.MachineReport"))
  (do-for-all-facts ((?machine machine-type)) TRUE
    (bind ?mre (pb-create "llsf_msgs.MachineReportEntry"))
    (pb-set-field ?mre "name" (str-cat ?machine:name))
    (pb-set-field ?mre "type" (str-cat ?machine:type))
    (pb-add-list ?mr "machines" ?mre)
  )
  (pb-broadcast ?mr)
  (modify ?ws (time ?now) (seq (+ ?seq 1)))
)

(defrule exp-receive-recognized-machines
  (phase EXPLORATION)
  ?pbm <- (protobuf-msg (type "llsf_msgs.MachineReportInfo") (ptr ?p))
  =>
  (retract ?pbm)
  (foreach ?machine (pb-field-list ?p "reported_machines")
    (assert (machineRecognized (sym-cat ?machine)))
    ;(printout t "Ich habe folgende Maschine bereits erkannt: " ?machine crlf)
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
