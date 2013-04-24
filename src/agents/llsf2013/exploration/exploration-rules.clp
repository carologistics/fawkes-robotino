;Clips program for exploration phase

(defrule init
  (init)
  (protobuf-available)
  =>
  (pb-peer-enable "172.16.35.255" 4445 4444)
)

;Set up the status
;there are two rounds. In the first the robotino drives to each machine in a defined cycle. After the first round the robotino drives to unrecognized machines again.
(defrule start
  ?st  <- (start)
  (time $?now)
  =>
  (retract ?st)
  (assert (status "start"))
  (assert (status "firstround"))
  (assert (signal (name "refboxSendLoop") (time ?now) (seq 2)))
  (printout t "Yippi ka yeah. I am in the exploration-phase." crlf)
)

(defrule test
  (Position3DInterface (id "Pose") (translation $?pos))
  (time $?now)
  =>
  (printout t "HAVE pos" crlf)
)

;Robotino drives to the nearest machine to start the first round
(defrule goto-nearest-machine
  ?s <- (status "start")
  (status "firstround")
  (machine-exploration (name ?v) (x ?x) (y ?y) (next ?))
  (forall (machine-exploration (name ?) (x ?x1) (y ?y1) (next ?))
    (Position3DInterface (id "Pose") (translation $?pos&:(<= (+ (* (- ?x (nth$ 1 ?pos)) (- ?x (nth$ 1 ?pos))) (* (- ?y (nth$ 2 ?pos)) (- ?y (nth$ 2 ?pos)))) (+ (* (- ?x1 (nth$ 1 ?pos)) (- ?x1 (nth$ 1 ?pos))) (* (- ?y1 (nth$ 2 ?pos)) (- ?y1 (nth$ 2 ?pos)))))))
  )
  =>
  (printout t "nearest machine:" ?v crlf)
  (skill-call ppgoto place (str-cat ?v))
  (retract ?s)
  (assert (status "drivingToMachine"))
  (assert (goalmachine ?v))
  (assert (startmachine ?v))
)

;arriving at a machine in first or second round. Preparing recognition of the light signals
(defrule arrived-at-machine
  (declare (salience 0))
  ?final <- (skill (name "ppgoto") (status FINAL) (skill-string ?skill)) 
  ?s <- (status "drivingToMachine")
  (time $?now)
  =>
  (printout t "arrived. skill string was: " ?skill crlf)
  (printout t "Read light now" crlf)
  (retract ?s ?final)
  (assert (status "waitingAtMachine"))
  (assert (signal (name "waitingSince") (time ?now) (seq 1)))

  ;;;;;;; this  should be the waiting skill ;;;;;
  ;(skill-call relgoto rel_x 0 rel_y 0)
)

;Recognizing succseded => memorize light-signals for further rules and prepare to drive to the next machine
(defrule recognized-machine
  ;;;;;;; change relgoto to waiting skill ;;;;;
  ;?final <- (skill (name "relgoto") (status FINAL) (skill-string ?skill)) 
  ?ws <- (signal (name "waitingSince") (time $?) (seq ?))
  ?s <- (status "waitingAtMachine")
  ?g <- (goalmachine ?old)
  (machine-exploration (name ?old) (x ?) (y ?) (next ?nextMachine))
  ?rli <- (RobotinoLightInterface (id "Light_State") (red ?red) (yellow ?yellow) (green ?green) (ready TRUE))

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
)

;Matching of recognized lights to the machine types
(defrule match-light-types
  ?ml <- (machine-light (name ?name) (red ?red) (yellow ?yellow) (green ?green))
  (matching-type-light (type ?type) (red ?red) (yellow ?yellow) (green ?green))
  =>
  (assert (machine-type (name ?name) (type ?type)))
  (assert (machineRecognized ?name))
  (printout t "Identified machine" crlf)
  (retract ?ml)
)

;Sending all results to the refbox every second
(defrule send-recognized-machines
  ;;;;;Tell the refbox;;;;;;;;;;;;;;;
  (time $?now)  
  ?ws <- (signal (name "refboxSendLoop") (time $?t&:(timeout ?now ?t 1.0)) (seq 2))
  
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
  (modify ?ws (name "refboxSendLoop") (time ?now) (seq 2))
)

;Recieve light-pattern-to-type matchig and save it in a fact
(defrule recieve-type-light-pattern-matching
  ?pbm <- (protobuf-msg (type "llsf_msgs.ExplorationInfo") (ptr ?p) (rcvd-via BROADCAST))
;  (or (not (matching-type-light (type T1) (red ?) (yellow ?) (green ?))) 
;      (not (matching-type-light (type T2) (red ?) (yellow ?) (green ?)))
;      (not (matching-type-light (type T3) (red ?) (yellow ?) (green ?)))
;      (not (matching-type-light (type T4) (red ?) (yellow ?) (green ?)))
;      (not (matching-type-light (type T5) (red ?) (yellow ?) (green ?))))

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

;the refbox sends BLINK but in the interface BLINKING is used. This rule converts BLINK to BLINKING
(defrule convert-blink-to-blinking
  (declare (salience 10));this rule has to fire before compose-type-light-pattern-matching
  ?tsp <- (type-spec-pre ?type ?light-color BLINK)
  =>
  ;(retract ?tsp)
  (assert (type-spec-pre ?type ?light-color BLINKING))
)

;Compose information sent by the refbox as one
(defrule compose-type-light-pattern-matching
  (declare (salience 0));this rule has to fire after convert-blink-to-blinking
  ?r <- (type-spec-pre ?type RED ?red-state)
  ?y <- (type-spec-pre ?type YELLOW ?yellow-state)
  ?g <- (type-spec-pre ?type GREEN ?green-state)
  =>
  (assert (matching-type-light (type ?type) (red ?red-state) (yellow ?yellow-state) (green ?green-state)))
  (retract ?r ?y ?g)
)

;Recognizing of lights failed => drive to next mashine or retry (depending on the round)
(defrule recognized-machine-failed
  ;?final <- (skill (name "relgoto") (status FAILED) (skill-string ?skill))
  (time $?now)
  ?ws <- (signal (name "waitingSince") (time $?t&:(timeout ?now ?t 3.0)) (seq 1))
  ?s <- (status "waitingAtMachine")
  ?g <- (goalmachine ?old)
  (machine-exploration (name ?old) (x ?) (y ?) (next ?nextMachine))

  =>
 
  (printout t "Reading light at " ?old " failed." crlf)
  (retract ?s)  
  (retract ?g)
  (retract ?ws)  
  ;(retract ?final)
  (assert (status "idle"))
  (assert (nextInCycle ?nextMachine))

;  (if (not (matching-type-light (type ?) (red ?) (yellow ?) (green ?)))
;    then
;    (printout t "DO NOT HAVE A MATCHING FACT! Exploration Info?" crlf)
;    )
)

;Find next machine and assert drinve command in the first round
(defrule goto-next-machine-first-round
  (status "firstround")
  ?s <- (status "idle")
  ?n <- (nextInCycle ?nextMachine)
  (not (machineRecognized ?nextMachine))

  =>

  (printout t "Going to next machine" crlf)
  (retract ?s)
  (retract ?n)
  (assert (status "drivingToMachine"))
  (assert (goalmachine ?nextMachine))

  (skill-call ppgoto place (str-cat ?nextMachine))
)

;finish the first round and begin retry round
(defrule finish-first-round
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

;Drive to the nearest unrecognized machine in the retry round
(defrule retry-nearest-unrecognized
  (status "retryRound")
  ?s <- (status "idle")
  (machine-exploration (name ?m) (x ?x) (y ?y) (next ?))
  (not (machineRecognized ?m))
  (forall (machine-exploration (name ?m2) (x ?x1) (y ?y1) (next ?))
    (or (machineRecognized ?m2)
        (Position3DInterface (id "Pose") (translation $?pos&:(<= (+ (* (- ?x (nth$ 1 ?pos)) (- ?x (nth$ 1 ?pos))) (* (- ?y (nth$ 2 ?pos)) (- ?y (nth$ 2 ?pos)))) (+ (* (- ?x1 (nth$ 1 ?pos)) (- ?x1 (nth$ 1 ?pos))) (* (- ?y1 (nth$ 2 ?pos)) (- ?y1 (nth$ 2 ?pos)))))))
    )
  )

  =>

  (printout t "Retry machine" crlf)
  (retract ?s)
  (assert (status "drivingToMachine"))
  (assert (goalmachine ?m))

  (skill-call ppgoto place (str-cat ?m))
)


;Finish exploration phase if all machines are recognized
(defrule finish-exploration
  (forall (machine-exploration (name ?m) (x ?) (y ?) (next ?))
    (machineRecognized ?m))
  ?s <- (status "retryRound")
  
  =>

  (printout t "Finished Exploration :-)" crlf)
  (retract ?s)
  (assert (status "finishedExploration"))
)
