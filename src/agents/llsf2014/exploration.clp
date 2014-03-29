
;---------------------------------------------------------------------------
;  exploration.clp - Robotino agent for exploration phase
;
;  Created: Fri Apr 26 18:38:18 2013 (Magdeburg)
;  Copyright  2013  Frederik Zwilling
;             2013  Alexander von Wirth 
;             2013  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

;Read exploration rows from config
(defrule exp-cfg-get-row
  (declare (salience ?*PRIORITY-WM*))
  (phase EXPLORATION)
  (confval (path "/clips-agent/llsf2014/exploration/row-high") (list-value $?row-high))
  (confval (path "/clips-agent/llsf2014/exploration/row-mid") (list-value $?row-mid))
  (confval (path "/clips-agent/llsf2014/exploration/row-low") (list-value $?row-low))
  (confval (path "/clips-agent/llsf2014/exploration/row") (value ?agent-row))
  =>
  (bind $?row-high-sym (create$))
  (progn$ (?m ?row-high)
    (bind $?row-high-sym (append$ ?row-high-sym (sym-cat ?m)))
  )
  (bind $?row-mid-sym (create$))
  (progn$ (?m ?row-mid)
    (bind $?row-mid-sym (append$ ?row-mid-sym (sym-cat ?m)))
  )
  (bind $?row-low-sym (create$))
  (progn$ (?m ?row-low)
    (bind $?row-low-sym (append$ ?row-low-sym (sym-cat ?m)))
  )
  (bind ?row (sym-cat ?agent-row))
  (switch ?row
    (case HIGH then (assert (exp-row (name HIGH) (row ?row-high-sym))))
    (case MID then (assert (exp-row (name MID) (row ?row-mid-sym))))
    (case LOW then (assert (exp-row (name LOW) (row ?row-low-sym))))
    (default (printout warn "NO EXPLORATION ROW DEFINED! UNABLE TO DO EXPLORATION!" crlf))
  )
  (printout t "At first, I am exploring the " ?row " row" crlf)
)

;turn over line if we start on the left field
(defrule exp-turn-line-over
  (declare (salience ?*PRIORITY-WM*))
  (phase EXPLORATION)
  (team-color MAGENTA)
  ?er <- (exp-row (row $?row))
  (not (exp-line-already-turned))
  =>
  (bind $?turned-row (create$))
  (progn$ (?m ?row)
    (bind ?turned-row (insert$ ?turned-row 1 ?m))
  )
  (modify ?er (row ?turned-row))
  (assert (exp-line-already-turned))
)

;Determine the first machine in dependency of the role
(defrule exp-determine-first-machine
  (phase EXPLORATION)
  (exp-row (row $?row))
  (team-color ?team-color)
  =>
  ;find first machine in row
  (bind ?first NONE)
  (progn$ (?m ?row)
    (do-for-fact ((?me machine-exploration)) (and (eq ?m ?me:name) (eq ?me:team ?team-color))
      (bind ?first ?me:name)
    )
    (if (neq ?first NONE)
      then
      (break)
    )
  )
  (assert (first-exploration-machine ?first))
)

(defrule exp-handle-no-own-machine-in-row
  (declare (salience ?*PRIORITY-WM*))
  (first-exploration-machine NONE)
  ?s <- (state EXP_START)
  ?et <- (exp-tactic LINE)
  (exp-row (row $?row))
  =>
  ;go directly in the nearest-mode
  (retract ?s ?et)
  (assert (state EXP_IDLE)
	  (exp-tactic NEAREST)
	  (goalmachine (nth$ (length$ ?row) ?row)) ;getting nearest from last in row
  )
)

;Set up the state
;There are two rounds. In the first the robotino drives to each machine in a defined cycle. After the first round the robotino drives to unrecognized machines again.
(defrule exp-start
  (phase EXPLORATION)
  ?st <- (exploration-start)
  (team-color ?team-color)
  =>
  (retract ?st)
  (assert (state EXP_START)
          (exp-tactic LINE)
          (timer (name send-machine-reports))
          (timer (name print-unrecognized-lights))
  )
  (if (eq ?team-color nil) then
    (printout error "Ouch, starting exploration but I don't know my team color" crlf)
  )
  (printout t "Yippi ka yeah. I am in the exploration-phase." crlf)
)

;Robotino drives to the first machine to start the first round
(defrule exp-goto-first
  (phase EXPLORATION)
  ?s <- (state EXP_START)
  (exp-tactic LINE)
  (first-exploration-machine ?v)
  (machine-exploration (name ?v) (x ?) (y ?) (look-pos ?lp))
  (not (driven-to-waiting-point))
  =>
  (printout t "First machine: " ?v crlf)
  (retract ?s)
  (assert (state EXP_LOCK_REQUIRED)
    (lock (type GET) (agent ?*ROBOT-NAME*) (resource ?v))
    (exp-next-machine ?v)
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
  (machine-exploration (name ?old) (x ?) (y ?))
  (confval (path "/clips-agent/llsf2014/exploration/needed-visibility-history") (value ?needed-vh))
  ?rli <- (RobotinoLightInterface (id "Light_State") (red ?red) (yellow ?yellow) (green ?green) (visibility_history ?vh&:(> ?vh ?needed-vh)) (ready TRUE))
  (matching-type-light (type ?type) (red ?red) (yellow ?yellow) (green ?green))
  =>
  (printout t "Identified machine" crlf)
  (printout t "Read light: red: " ?red " yellow: " ?yellow " green: " ?green crlf)
  (retract ?s ?rli ?ws)
  (assert (state EXP_IDLE)
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
  (machine-exploration (name ?old) (x ?) (y ?))
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
  (retract ?s ?ws ?srt)
  (assert (state EXP_IDLE)
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

;Find next machine when driving along the line
(defrule exp-find-next-machine-line
  (phase EXPLORATION)
  ?t <- (exp-tactic LINE)
  ?s <- (state EXP_IDLE)
  ?g <- (goalmachine ?old)
  (exp-row (row $?row))
  (team-color ?team-color&~nil)
  =>
  ;find next machine in the line
  (bind ?ind (member$ ?old ?row))
  (bind ?ind (+ ?ind 1))
  (while (<= ?ind (length$ ?row)) do
    ; can I go to this machine next?
    (if (and (any-factp ((?me machine-exploration)) (and (eq ?me:name (nth$ ?ind ?row))
							 (eq ?me:team ?team-color)
							 (not ?me:recognized)))
	     (not (any-factp ((?lock locked-resource)) (eq ?lock:resource (nth$ ?ind ?row)))))
      then
      (break)
    )
    (bind ?ind (+ ?ind 1))
  )
  (if (<= ?ind (length$ ?row))
    then
    ;we found the next machine on the line
    ;(printout t "Found next: " (nth$ ?ind ?row) crlf)
    (retract ?s ?g)
    (assert (state EXP_FOUND_NEXT_MACHINE)
	    (exp-next-machine (nth$ ?ind ?row)))
    
    else
    ;there is not machine on the line left
    ;(printout warn "No next machine on the line" crlf)
    ;change strategy
    (printout t "Changing exploration strategy: driving to nearest machine starting at end of line" crlf)
    (retract ?t ?g)
    (assert (exp-tactic NEAREST)
	    (goalmachine (nth$ (length$ ?row) ?row))) ;getting nearest from last in row
  )
)

;Find next machine when driving along the line
(defrule exp-find-next-machine-nearest
  (phase EXPLORATION)
  (exp-tactic NEAREST)
  ?s <- (state EXP_IDLE)
  ?g <- (goalmachine ?old)
  (team-color ?team-color)
  (machine-exploration (name ?old) (x ?x) (y ?y) (team ?team))
  =>
  ;find next machine nearest to last machine
  (bind ?nearest NONE)
  (bind ?min-dist 1000.0)
  (do-for-all-facts ((?me machine-exploration))
    (and (eq ?me:team ?team-color) (not ?me:recognized)
	 (not (any-factp ((?mt machine-type)) (eq ?mt:name ?me:name))))

    ;check if the machine is nearer and unlocked
    (bind ?dist (distance ?x ?y ?me:x ?me:y))
    (if (and (not (any-factp ((?lock locked-resource)) (eq ?lock:resource ?me:name)))
	     (< ?dist ?min-dist))
      then
      (bind ?min-dist ?dist)
      (bind ?nearest ?me:name)
    )
  )
  (if (neq ?nearest NONE)
    then
    ;we found the next machine
    (printout warn "Found next: " ?nearest crlf)
    (retract ?s ?g)
    (assert (state EXP_FOUND_NEXT_MACHINE)
	    (exp-next-machine ?nearest))
    else
    (if (and (not (any-factp ((?recognized machine-type)) (eq ?recognized:name ?old)))
	     (eq ?team ?team-color))
      then
      (printout t "Retrying last machine" crlf)
      (assert (state EXP_FOUND_NEXT_MACHINE)
	      (exp-next-machine ?old))
      else
      (printout t "prepare for production" crlf)
      (retract ?s ?g)
      (assert (state EXP_PREPARE_FOR_PRODUCTION))
    )

  )
)

;Require resource-locking
(defrule exp-require-resource-locking
  (phase EXPLORATION)
  ?s <- (state EXP_FOUND_NEXT_MACHINE)
  (exp-next-machine ?nextMachine)
  =>
  (printout t "Require lock for " ?nextMachine crlf)
  (retract ?s)
  (assert (state EXP_LOCK_REQUIRED)
	  (lock (type GET) (agent ?*ROBOT-NAME*) (resource ?nextMachine)))
)

;Wait for answer from MASTER
(defrule exp-check-resource-locking
  (phase EXPLORATION)
  ?s <- (state EXP_LOCK_REQUIRED)
  (exp-next-machine ?nextMachine)
  ?l <- (lock (type ACCEPT) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?nextMachine))
  =>
  (printout t "Lock accepted." crlf)
  (retract ?s ?l)
  (assert (state EXP_LOCK_ACCEPTED))
)

;Drive to next machine
(defrule exp-go-to-next-machine
  (phase EXPLORATION)
  ?s <- (state EXP_LOCK_ACCEPTED)
  ?n <- (exp-next-machine ?nextMachine)
  (machine-exploration (name ?nextMachine) (x ?) (y ?) (next ?) (look-pos ?lp))
  (not (driven-to-waiting-point))
  =>
  (printout t "Going to next machine." crlf)
  (retract ?s ?n)
  (assert (state EXP_DRIVING_TO_MACHINE)
          (goalmachine ?nextMachine))
  (skill-call ppgoto place (str-cat ?lp))
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
  (foreach ?m (pb-field-list ?p "machines")
    (bind ?name (pb-field-value ?m "name"))
    (bind ?team (sym-cat (pb-field-value ?m "team_color")))
    (do-for-fact ((?machine machine) (?me machine-exploration))
      (and (eq ?machine:name ?name) (eq ?me:name ?name))
      (modify ?machine (team ?team))
      (modify ?me (team ?team))
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
  (state ?s&:(neq ?s IDLE))
  (time $?now)
  ?ws <- (timer (name send-machine-reports) (time $?t&:(timeout ?now ?t 0.5)) (seq ?seq))
  (game-time $?game-time)
  (confval (path "/clips-agent/llsf2014/exploration/latest-send-last-report-time")
	   (value ?latest-report-time))
  (team-color ?team-color&~nil)
  =>
  (bind ?mr (pb-create "llsf_msgs.MachineReport"))
  (pb-set-field ?mr "team_color" ?team-color)
  (do-for-all-facts ((?machine machine-type)) TRUE
    ;send report for last machine only if the exploration phase is going to end
    ;or we are prepared for production
    (if (or
	 (< (length (find-all-facts ((?f machine-exploration))
				    (and (eq ?f:team ?team-color) ?f:recognized)))
	    (- (length (find-all-facts ((?f machine-exploration)) (eq ?f:team ?team-color))) 1))
	 (>= (nth$ 1 ?game-time) ?latest-report-time)
	 (eq ?s EXP_PREPARE_FOR_PRODUCTION_FINISHED))
     then
      (bind ?mre (pb-create "llsf_msgs.MachineReportEntry"))
      (pb-set-field ?mre "name" (str-cat ?machine:name))
      (pb-set-field ?mre "type" (str-cat ?machine:type))
      (pb-add-list ?mr "machines" ?mre)
    )
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
    (do-for-fact ((?m machine-exploration)) (eq ?m:name (sym-cat ?machine))
      (modify ?m (recognized TRUE))
      ;(printout t "Ich habe folgende Maschine bereits erkannt: " ?machine crlf)
    )
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


;Finish exploration phase if all machines are recognized
(defrule exp-prepare-for-production-get-lock-for-ins
  (phase EXPLORATION)
  (state EXP_PREPARE_FOR_PRODUCTION)
  ?et <- (exp-tactic NEAREST)
  (team-color ?team-color)
  (input-storage ?team-color ?ins)
  =>
  (printout t "Finished Exploration :-)" crlf)
  (retract ?et)
  (assert (exp-tactic GOTO-INS)
	  (lock (type GET) (agent ?*ROBOT-NAME*) (resource ?ins))
  )
)
(defrule exp-prepare-for-production-drive-to-ins
  (phase EXPLORATION)
  (state EXP_PREPARE_FOR_PRODUCTION)
  (exp-tactic GOTO-INS)
  (team-color ?team-color)
  (input-storage ?team-color ?ins)
  (lock (type ACCEPT) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?ins))
  =>
  (printout t "Waiting for production at " ?ins crlf)
  (skill-call ppgoto place (str-cat ?ins))
  (assert (prepare-for-production-goal ?ins))
)
(defrule exp-prepare-for-production-drive-to-wait-for-ins
  (phase EXPLORATION)
  (state EXP_PREPARE_FOR_PRODUCTION)
  (exp-tactic GOTO-INS)
  (team-color ?team-color)
  (input-storage ?team-color ?ins)
  (lock (type REFUSE) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?ins))
  ?lock <- (lock (type GET) (agent ?a) (resource ?ins))
  (wait-point ?ins ?wait-point)
  =>
  (printout t "Waiting for production at " ?wait-point crlf)
  (skill-call ppgoto place (str-cat ?wait-point))
  (retract ?lock)
  (assert (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?ins))
	  (prepare-for-production-goal ?wait-point))
)
(defrule exp-prepare-for-production-finished
  (phase EXPLORATION)
  ?s <- (state EXP_PREPARE_FOR_PRODUCTION)
  (exp-tactic GOTO-INS)
  ?skill-f <- (skill (name "ppgoto") (status FINAL))
  =>
  (retract ?skill-f ?s)
  (assert (timer (name annound-prepare-for-production-finished))
	  (state EXP_PREPARE_FOR_PRODUCTION_FINISHED))
)
(defrule exp-prepare-for-production-announce-finished
  (phase EXPLORATION)
  (state EXP_PREPARE_FOR_PRODUCTION_FINISHED)
  (time $?now)
  ?timer <- (timer (name beacon) (time $?t&:(timeout ?now ?t ?*WORLDMODEL-SYNC-PERIOD*)) (seq ?seq))
  (prepare-for-production-goal ?wait-point)
  =>
  (modify ?timer (time ?now) (seq (+ ?seq 1)))
  (bind ?msg (pb-create "llsf_msgs.PreparedForProduction"))
  (pb-set-field ?msg "peer_name" ?*ROBOT-NAME*)
  (pb-set-field ?msg "waiting_point" ?wait-point)
  (pb-broadcast ?msg)
  (pb-destroy ?msg)
)
(defrule exp-receive-prepare-for-production-announce-finished
  (phase EXPLORATION)
  ?s <- (state EXP_PREPARE_FOR_PRODUCTION)
  ?pf <- (protobuf-msg (type "llsf_msgs.PreparedForProduction") (ptr ?p) (rcvd-via BROADCAST))
  (team-color ?team-color)
  (input-storage ?team-color ?ins)
  =>
  ;the preperation is finished when someone stands at ins
  (if (eq ?ins (sym-cat (pb-field-value ?p "waiting_point")))
      then
      (retract ?s)
      (assert (state EXP_PREPARE_FOR_PRODUCTION_FINISHED))
  )
)
