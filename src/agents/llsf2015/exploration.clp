
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
  "Read configuration for exploration row order of machines"
  (declare (salience ?*PRIORITY-WM*))
  (phase EXPLORATION)
  (confval (path "/clips-agent/llsf2015/exploration/row-high") (list-value $?row-high))
  (confval (path "/clips-agent/llsf2015/exploration/row-mid") (list-value $?row-mid))
  (confval (path "/clips-agent/llsf2015/exploration/row-low") (list-value $?row-low))
  (confval (path "/clips-agent/llsf2015/exploration/row") (value ?agent-row))
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

(defrule exp-turn-line-over
  "Turn over line if we start on the left field"
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

(defrule exp-determine-first-machine
  "Determine the first machine in dependency of the role"
  (phase EXPLORATION)
  (exp-row (row $?row))
  (team-color ?team-color&~nil)
  (exp-machines-initialized)
  =>
  ;find first machine in row
  (bind ?first NONE)
  (progn$ (?m ?row)
    (do-for-fact ((?me zone-exploration)) (and (eq ?m ?me:name) (eq ?me:team ?team-color))
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
  "If exploration row is empty(explored) change exploration tactic from LINE to NEAREST"
  (declare (salience ?*PRIORITY-WM*))
  (first-exploration-machine NONE)
  ?s <- (state EXP_START)
  ?et <- (exp-tactic LINE)
  (exp-row (row $?row))
  =>
  ;go directly in the nearest-mode
  (printout t "No zone of my team is in my line" crlf)
  (retract ?s ?et)
  (assert (state EXP_IDLE)
	  (exp-tactic NEAREST)
	  (goalmachine (nth$ (length$ ?row) ?row)) ;getting nearest from last in row
  )
)

(defrule exp-start
  "Set up the state. There are two rounds. In the first the robotino drives to each machine in a defined cycle. After the first round the robotino drives to unrecognized machines again."
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
  ;disable delivery mode of the machine_signal plugin (for safety)
  (skill-call enable_switch iface "delivery" enable false)
)

(defrule exp-goto-first
  "Robotino drives to the first machine to start the first round."
  (phase EXPLORATION)
  ?s <- (state EXP_START)
  ?df <- (skill-done (name "enable_switch"))
  (exp-tactic LINE)
  (first-exploration-machine ?v)
  (zone-exploration (name ?v) (x ?) (y ?))
  (not (driven-to-waiting-point))
  =>
  (printout t "First machine: " ?v crlf)
  (retract ?s)
  (assert (state EXP_LOCK_REQUIRED)
    (lock (type GET) (agent ?*ROBOT-NAME*) (resource ?v))
    (exp-next-machine ?v)
  )
)

(defrule exp-drive-to-finished
  "Arriving at a machine in first or second round. Wait for tag vision."
  (phase EXPLORATION)
  ?final <- (skill (name "ppgoto") (status FINAL|FAILED)) 
  ?s <- (state EXP_DRIVING_TO_MACHINE)
  (time $?now)
  =>
  (printout t "Arrived. Looking for a tag." crlf)
  (retract ?s ?final)
  (assert (state EXP_LOOKING_FOR_TAG)
          (timer (name waiting-for-tag-since) (time ?now) (seq 1))
  )
)

(defrule exp-read-tag-from-exp-point
  "Read found tag in front of robot and drive nearer to the tag to get a better reading or prepare adding the tag to the navgraph when already driven to the tag."
  (phase EXPLORATION)
  ?ws <- (timer (name waiting-for-tag-since))
  ?s <- (state EXP_LOOKING_FOR_TAG)
  ?g <- (goalmachine ?old)
  ?ze <- (zone-exploration (name ?old))
  (confval (path "/clips-agent/llsf2015/exploration/needed-visibility-history") (value ?needed-vh))
  (team-color ?team-color)
  (tag-matching (tag-id ?tag) (team ?team-color) (machine ?machine) (side ?side))
  (not (found-tag (name ?machine)))
  ?tvi <- (TagVisionInterface (id "/tag-vision/info") (tags_visible ?num-tags&:(> ?num-tags 0)) (tag_id $?tag-ids&:(member$ ?tag ?tag-ids)))
  ?tagpos <- (Position3DInterface (id ?tag-if-id&:(eq ?tag-if-id 
						      (str-cat "/tag-vision/" 
							       (- (member$ ?tag ?tag-ids) 1)))) 
				  (visibility_history ?vh&:(> ?vh ?needed-vh)) 
				  (translation $?trans) (rotation $?rot)
				  (frame ?frame) (time $?timestamp))
  ?ent <- (exp-nearing-tag ?already-near)
  (time $?now)
  =>
  (printout t "Found Tag Nr." ?tag " (" ?machine " " ?side ")"  crlf)
  ; Do I have to 
  (if ?already-near
    then
    (printout t "Recognized Tag" crlf)
    ; transform to map-frame
    (if (tf-can-transform "/map" ?frame ?timestamp) then
      (bind ?tf-transrot (tf-transform-pose "/map" ?frame ?timestamp ?trans ?rot))
      (printout t "Transformed to " ?tf-transrot crlf)
      (assert (found-tag (name ?machine) (side ?side) (frame "/map")
			 (trans (subseq$ ?tf-transrot 1 3))
			 (rot (subseq$ ?tf-transrot 4 7))))
      else
      (printout warn "Can not transform " ?frame " to /map. Trying most current time" crlf)
      (if (tf-can-transform "/map" ?frame (create$ 0 0)) then
	(bind ?tf-transrot (tf-transform-pose "/map" ?frame ?timestamp ?trans ?rot))
	(assert (found-tag (name ?machine) (side ?side) (frame "/map")
			   (trans (subseq$ ?tf-transrot 1 3))
			   (rot (subseq$ ?tf-transrot 4 7))))
	else
	(printout error "Can not transform " ?frame " to /map. Tags positions are broken after synchronization" crlf)
	(assert (found-tag (name ?machine) (side ?side) (frame "/map")
			   (trans ?trans) (rot ?rot)))
      )
    )
    (assert (state EXP_ADD_TAG)
    	    (exp-nearing-tag FALSE)
    	    (worldmodel-change (machine ?machine) (change ADD_TAG)))
    else
    (printout t "Driving nearer to get a more precise position" crlf)
    ; (skill-call drive_tag id ?tag)
    (skill-call motor_move x 0.05)
    (printout warn "Use right skill to drive nearer to tag!" crlf)
    (assert (state EXP_DRIVING_NEARER_TO_TAG)
	    (exp-nearing-tag TRUE))
  )
  (retract ?s ?tvi ?tagpos ?ws ?ent)
)

(defrule exp-nearing-tag-finished
  "Arriving at a machine in first or second round. Wait for tag vision."
  (phase EXPLORATION)
  ?final <- (skill (name "motor_move") (status FINAL|FAILED)) 
  ?s <- (state EXP_DRIVING_NEARER_TO_TAG)
  (time $?now)
  =>
  (printout t "Arrived. Looking for a tag." crlf)
  (retract ?s ?final)
  (assert (state EXP_LOOKING_FOR_TAG)
          (timer (name waiting-for-tag-since) (time ?now) (seq 1))
  )
)

(defrule exp-report-found-tag
  (phase EXPLORATION)
  ?s <- (state EXP_ADD_TAG)
  ?g <- (goalmachine ?old)
  ?ze <- (zone-exploration (name ?old))
  (team-color ?team-color)
  (tag-matching (tag-id ?tag) (team ?team-color) (machine ?machine) (side ?side))
  ?ft <- (found-tag (name ?machine) (side ?side) (frame ?frame)
		    (trans $?trans) (rot $?rot) (already-added FALSE))
  =>
  (printout t "Add Tag Nr." ?tag " (" ?machine " " ?side ") to Navgraph-generation"  crlf)
  
  ;TODO: this might be the wrong zone
  (modify ?ze (machine ?machine) (still-to-explore FALSE))
  (assert (worldmodel-change (machine ?old) (change ZONE_STILL_TO_EXPLORE) (value FALSE)))

  (navgraph-add-all-new-tags)

  (printout t "Wait until ComputeMessage is processed" crlf)
  (retract ?s)
  (assert (state EXP_WAIT_BEFORE_DRIVE_TO_OUTPUT))
)

(defrule exp-no-tag-found
  "We couldn't see a tag => try next look-pos or skip zone"
  (phase EXPLORATION)
  (time $?now)
  ?ws <- (timer (name waiting-for-tag-since) (time $?t&:(timeout ?now ?t 3.0)))
  ?s <- (state EXP_LOOKING_FOR_TAG)
  ?g <- (goalmachine ?old)
  ?ze <- (zone-exploration (name ?old) (look-pos $?lp) (current-look-pos ?lp-index))
  ?ent <- (exp-nearing-tag ?nearing)
  =>
  (printout t "Found no tag from " (nth$ ?lp-index ?lp) crlf)
  (if ?nearing
    then
    (printout t "Lost tag after driving to it." crlf)
    (retract ?ent)
    (assert (exp-nearing-tag FALSE))
  )
  (if (< ?lp-index (length$ ?lp))
    then
    (printout t "Try to find tag from the next position." crlf)
    (modify ?ze (current-look-pos (+ 1 ?lp-index)))
    (assert (state EXP_LOCK_ACCEPTED)
	    (exp-next-machine ?old))
    (retract ?g)

    else
    (printout t "Couldn't find a tag in this zone!" crlf)
    (modify ?ze (current-look-pos 1))
    (assert (state EXP_IDLE)
	  (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?old)))
    )
  (retract ?ws ?s)
)

(defrule exp-drive-to-output-tag
  "Drive to the output tag of a found machine, so that we can align in front of the light-signal afterwards"
  (phase EXPLORATION)
  ?s <- (state EXP_WAIT_BEFORE_DRIVE_TO_OUTPUT)
  ; wait until the navgraph-generator has added the path to the output
  ?lncm <- (last-navgraph-compute-msg (id ?compute-msg-id))
  ?ngg-if <- (NavGraphWithMPSGeneratorInterface (id "/navgraph-generator-mps") (msgid ?compute-msg-id) (final TRUE))
  (goalmachine ?zone)
  (zone-exploration (name ?zone) (machine ?machine))
  =>
  (printout t "Driving to MPS light-signal." crlf)
  (retract ?s ?ngg-if ?lncm)
  (assert (state EXP_DRIVE_TO_OUTPUT))
  (skill-call ppgoto place (get-light-signal-side ?machine))
)

(defrule exp-align-in-front-of-light-signal
  "Preparing recognition of the light signals after we arrived at the output side."
  (phase EXPLORATION)
  ?final <- (skill (name "ppgoto") (status FINAL|FAILED)) 
  ?s <- (state EXP_DRIVE_TO_OUTPUT)
  =>
  (printout t "Aligning in front of light signal." crlf)
  (retract ?s ?final)
  (assert (state EXP_ALIGN_AT_OUTPUT))
  (skill-call align_tag x 0.2 y 0.1)
)

(defrule exp-align-in-front-of-light-signal-finished
  "Preparing recognition of the light signals after we aligned in front of it."
  (phase EXPLORATION)
  ?final <- (skill (name "align_tag") (status FINAL|FAILED)) 
  ?s <- (state EXP_ALIGN_AT_OUTPUT)
  (time $?now)
  =>
  (printout t "Aligned in front of light signal." crlf)
  (printout t "Reading light now." crlf)
  (retract ?s ?final)
  (assert (state EXP_WAITING_FOR_LIGHT_VISION)
          (timer (name waiting-since) (time ?now) (seq 1))
  )
)

(defrule exp-read-light-at-machine
  "RobotinoLightInterface has recognized the light-signals => memorize light-signals for further rules and prepare to drive to the next machine."
  (phase EXPLORATION)
  (time $?now)
  ?ws <- (timer (name waiting-since))
  ?s <- (state EXP_WAITING_FOR_LIGHT_VISION)
  ?g <- (goalmachine ?old)
  (zone-exploration (name ?old) (x ?) (y ?))
  (confval (path "/clips-agent/llsf2015/exploration/needed-visibility-history") (value ?needed-vh))
  ?rli <- (RobotinoLightInterface (id "/machine-signal/best") (red ?red) (yellow ?yellow) (green ?green) (visibility_history ?vh&:(> ?vh ?needed-vh)) (ready TRUE))
  =>
  (printout t "Identified machine" crlf)
  (printout t "Read light: red: " ?red " yellow: " ?yellow " green: " ?green crlf)
  (printout warn "TODO: ensure that tag is in the explored zone" crlf)
  (retract ?s ?rli ?ws)
  (assert (state EXP_IDLE)
    (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?old))
    (exploration-result (machine ?old) (red ?red) (yellow ?yellow) (green ?green)
			(zone ?old))
  )
)

(defrule exp-recognized-machine-failed-once
  "First recognition of lights failed => try again with a slightly other position"
  (phase EXPLORATION)
  (time $?now)
  ?ws <- (timer (name waiting-since) (time $?t&:(timeout ?now ?t 5.0)))
  ?s <- (state EXP_WAITING_FOR_LIGHT_VISION)
  ?g <- (goalmachine ?old)
  (zone-exploration (name ?old) (x ?) (y ?))
  (not (second-recognize-try))
  =>
  (printout t "Reading light at " ?old " failed first time." crlf)
  (printout t "Try again in from an other position." crlf)
  (assert (second-recognize-try)
  )
  (modify ?ws (time ?now))
  (skill-call motor_move x 0 y 0.15 vel_trans 0.1)
)

(defrule exp-recognized-machine-failed-twice
  "Second try of recognition of lights failed => drive to next mashine or retry (depending on the round)"
  (phase EXPLORATION)
  (time $?now)
  ?ws <- (timer (name waiting-since) (time $?t&:(timeout ?now ?t 5.0)))
  ?s <- (state EXP_WAITING_FOR_LIGHT_VISION)
  ?g <- (goalmachine ?old)
  (zone-exploration (name ?old) (x ?) (y ?) (next ?nextMachine))
  ?srt <- (second-recognize-try)
  =>
  (printout t "Reading light at " ?old " failed." crlf)
  (printout t "Waited 5 seconds on RobotinoLightInterface with ready = TRUE." crlf)
  (retract ?s ?ws ?srt)
  (assert (state EXP_IDLE)
	  (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?old))
  )
)

(defrule exp-second-retry-fact-retract
  "If driving again remove second-recognize-try fact."
  (state EXP_DRIVING_TO_MACHINE)
  ?srt <- (second-recognize-try)
  =>
  (retract ?srt)
)

(defrule exp-find-next-machine-line
  "Find next machine in exploration line."
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
    (if (and (any-factp ((?me zone-exploration)) (and (eq ?me:name (nth$ ?ind ?row))
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
    ; (printout t "Found next machine on line: " (nth$ ?ind ?row) crlf)
    (retract ?s ?g)
    (assert (state EXP_FOUND_NEXT_MACHINE)
	    (exp-next-machine (nth$ ?ind ?row)))
    
    else
    ;there is not machine on the line left
    (printout t "No next machine on the line" crlf)
    ;change strategy
    (printout t "Changing exploration strategy: driving to nearest machine starting at end of line" crlf)
    (retract ?t ?g)
    (assert (exp-tactic NEAREST)
	    (goalmachine (nth$ (length$ ?row) ?row))) ;getting nearest from last in row
  )
)

(defrule exp-find-next-machine-nearest
  "Find nearest machine close to last machine."
  (phase EXPLORATION)
  (exp-tactic NEAREST)
  ?s <- (state EXP_IDLE)
  ?g <- (goalmachine ?old)
  (team-color ?team-color)
  (zone-exploration (name ?old) (x ?x) (y ?y) (team ?team))
  =>
  ;find next machine nearest to last machine
  (bind ?nearest NONE)
  (bind ?min-dist 1000.0)
  (do-for-all-facts ((?me zone-exploration))
    (and (eq ?me:team ?team-color) (not ?me:recognized)
	 (not (any-factp ((?mt exploration-result)) (eq ?mt:machine ?me:name))))

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
    (if (and (not (any-factp ((?recognized exploration-result)) (eq ?recognized:machine ?old)))
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

(defrule exp-require-resource-locking
  "Request lock for next machine."
  (phase EXPLORATION)
  ?s <- (state EXP_FOUND_NEXT_MACHINE)
  (exp-next-machine ?nextMachine)
  =>
  (printout t "Require lock for " ?nextMachine crlf)
  (retract ?s)
  (assert (state EXP_LOCK_REQUIRED)
	  (lock (type GET) (agent ?*ROBOT-NAME*) (resource ?nextMachine)))
)

(defrule exp-check-resource-locking
  "Handle a lock that was accepted by the master"
  (phase EXPLORATION)
  ?s <- (state EXP_LOCK_REQUIRED)
  (exp-next-machine ?nextMachine)
  ?l <- (lock (type ACCEPT) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?nextMachine))
  =>
  (printout t "Lock accepted." crlf)
  (retract ?s ?l)
  (assert (state EXP_LOCK_ACCEPTED))
)

(defrule exp-go-to-next-machine
  "When lock was accepted move to the locked machine with ppgoto"
  (phase EXPLORATION)
  ?s <- (state EXP_LOCK_ACCEPTED)
  ?n <- (exp-next-machine ?nextMachine)
  (zone-exploration (name ?nextMachine) (x ?) (y ?) (next ?)
		    (look-pos $?lp) (current-look-pos ?lp-index))
  (not (driven-to-waiting-point))
  (pose (id ?pose-id&:(eq ?pose-id (nth$ ?lp-index ?lp)))
	(name ?pose-name) (ori ?pose-ori))
  =>
  (printout t "Going to next machine." crlf)
  (printout t "Try to see tag from " (nth$ ?lp-index ?lp)
	    " (" ?pose-name ")." crlf)
  (retract ?s ?n)
  (assert (state EXP_DRIVING_TO_MACHINE)
          (goalmachine ?nextMachine))
  (skill-call ppgoto place ?pose-name ori ?pose-ori)
)

(defrule exp-receive-type-light-pattern-matching
  "Receive light-pattern-to-type matching and save it in a fact."
  (phase EXPLORATION)
  ?pbm <- (protobuf-msg (type "llsf_msgs.ExplorationInfo") (ptr ?p))
  (not (have-exp-info))
  =>
  (retract ?pbm)
  (foreach ?m (pb-field-list ?p "machines")
    (bind ?name (sym-cat (pb-field-value ?m "name")))
    ; TODO remove M12 -> Z12 for new refbox version
    (bind ?zone-name (sym-cat (str-cat "Z" (sub-string 2 (str-length (str-cat ?name)) (str-cat ?name)))))
    (printout t "Machine " ?name " converted to zone " ?zone-name crlf)    
    (bind ?team (sym-cat (pb-field-value ?m "team_color")))
    (do-for-fact ((?me zone-exploration)) (eq ?me:name ?zone-name)
      (printout t "Zone " ?zone-name " is from team " ?team crlf)
      (modify ?me (team ?team))
    )
  )
  (assert (exp-machines-initialized) 
	  (have-exp-info))
)

(defrule exp-send-recognized-machines
  "Sending all results of machine recognition to the refbox every second"
  (phase EXPLORATION)
  (state ?s&:(neq ?s IDLE))
  (time $?now)
  ?ws <- (timer (name send-machine-reports) (time $?t&:(timeout ?now ?t 0.5)) (seq ?seq))
  (game-time $?game-time)
  (confval (path "/clips-agent/llsf2015/exploration/latest-send-last-report-time")
	   (value ?latest-report-time))
  (team-color ?team-color&~nil)
  (peer-id private ?peer)
  =>
  (bind ?mr (pb-create "llsf_msgs.MachineReport"))
  (pb-set-field ?mr "team_color" ?team-color)
  (do-for-all-facts ((?machine exploration-result)) TRUE
    ;send report for last machine only if the exploration phase is going to end
    ;or we are prepared for production
    (printout warn "TODO: correctly fill exploration report!" crlf)
    (if (or
	 (< (length (find-all-facts ((?f zone-exploration))
				    (and (eq ?f:team ?team-color) ?f:recognized)))
	    (- (length (find-all-facts ((?f zone-exploration)) (eq ?f:team ?team-color))) 1))
	 (>= (nth$ 1 ?game-time) ?latest-report-time)
	 (eq ?s EXP_PREPARE_FOR_PRODUCTION_FINISHED))
     then
      (bind ?mre (pb-create "llsf_msgs.MachineReportEntry"))
      (pb-set-field ?mre "name" (str-cat ?machine:machine))
      (pb-set-field ?mre "type" (str-cat ?machine:type))
      (pb-add-list ?mr "machines" ?mre)
    )
  )
  (pb-broadcast ?peer ?mr)
  (modify ?ws (time ?now) (seq (+ ?seq 1)))
)

(defrule exp-receive-recognized-machines
  (phase EXPLORATION)
  ?pbm <- (protobuf-msg (type "llsf_msgs.MachineReportInfo") (ptr ?p))
  =>
  (retract ?pbm)
  (unwatch facts zone-exploration)
  (foreach ?machine (pb-field-list ?p "reported_machines")
    (do-for-fact ((?m zone-exploration)) (eq ?m:machine (sym-cat ?machine))
      (modify ?m (recognized TRUE))
      ;(printout t "Ich habe folgende Maschine bereits erkannt: " ?machine crlf)
    )
  )
  (watch facts zone-exploration)
)

(defrule exp-convert-blink-to-blinking
  "The refbox sends BLINK but in the interface BLINKING is used. This rule converts BLINK to BLINKING."
  (declare (salience 10)) ; this rule has to fire before compose-type-light-pattern-matching
  (phase EXPLORATION)
  (type-spec-pre ?type ?light-color BLINK)
  =>
  (assert (type-spec-pre ?type ?light-color BLINKING))
)

(defrule exp-prepare-for-production-get-lock-for-ins
  "Finish exploration phase if all machines are recognized and request locks for pre-game positions."
  (phase EXPLORATION)
  (state EXP_PREPARE_FOR_PRODUCTION)
  ?et <- (exp-tactic NEAREST)
  (team-color ?team-color)
  (input-storage ?team-color ?ins ? ?)
  (secondary-storage ?team-color ?inssec ? ?)
  =>
  (printout t "Finished Exploration :-)" crlf)
  (retract ?et)
  (if (and (any-factp ((?inslock locked-resource)) (eq ?inslock:resource ?ins)) (not (any-factp ((?seclock locked-resource)) (eq ?seclock:resource ?inssec))))
    then
    (assert (exp-tactic GOTO-INS)
      (lock (type GET) (agent ?*ROBOT-NAME*) (resource ?inssec))
    )
    else
    (assert (exp-tactic GOTO-INS)
	    (lock (type GET) (agent ?*ROBOT-NAME*) (resource ?ins))
    )
  )
)

(defrule exp-prepare-for-production-drive-to-ins
  "When locks for pre-game positions are acquired, drive there with ppgoto."
  (phase EXPLORATION)
  (state EXP_PREPARE_FOR_PRODUCTION)
  (exp-tactic GOTO-INS)
  (team-color ?team-color)
  (or (input-storage ?team-color ?ins ? ?)
      (secondary-storage ?team-color ?ins ? ?))
  (lock (type ACCEPT) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?ins))
  =>
  (printout t "Waiting for production at " ?ins crlf)
  (skill-call drive_to place (str-cat ?ins))
  (assert (prepare-for-production-goal ?ins))
)

(defrule exp-prepare-for-production-drive-to-wait-for-ins
  "If insertion area lock was refused drive to wait-point"
  (phase EXPLORATION)
  (state EXP_PREPARE_FOR_PRODUCTION|EXP_PREPARE_FOR_PRODUCTION_FINISHED)
  (exp-tactic GOTO-INS)
  (team-color ?team-color)
  (or (input-storage ?team-color ?ins ? ?)
      (secondary-storage ?team-color ?ins ? ?))
  (lock (type REFUSE) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?ins))
  ?lock <- (lock (type GET) (agent ?a) (resource ?ins))
  (wait-point ?ins ?a ?wait-point)
  =>
  (printout t "Waiting for production at " ?wait-point crlf)
  (skill-call ppgoto place (str-cat ?wait-point))
  (retract ?lock)
  (assert (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?ins))
	  (prepare-for-production-goal ?wait-point))
)

(defrule exp-prepare-for-production-finished
  "When arrived at insertion area or waiting point finish preparation for production."
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
  "Announce the finishing of preparation."
  (phase EXPLORATION)
  (state EXP_PREPARE_FOR_PRODUCTION_FINISHED)
  (time $?now)
  ?timer <- (timer (name exploration-finished) (time $?t&:(timeout ?now ?t ?*WORLDMODEL-SYNC-PERIOD*)) (seq ?seq))
  (prepare-for-production-goal ?wait-point)
  (peer-id private ?peer)
  =>
  (modify ?timer (time ?now) (seq (+ ?seq 1)))
  (bind ?msg (pb-create "llsf_msgs.PreparedForProduction"))
  (pb-set-field ?msg "peer_name" ?*ROBOT-NAME*)
  (pb-set-field ?msg "waiting_point" ?wait-point)
  (pb-broadcast ?peer ?msg)
  (pb-destroy ?msg)
)

(defrule exp-receive-prepare-for-production-announce-finished
  "Receive finishing of preparation. Retract preparation facts."
  (phase EXPLORATION)
  ?s <- (state EXP_PREPARE_FOR_PRODUCTION)
  ?pf <- (protobuf-msg (type "llsf_msgs.PreparedForProduction") (ptr ?p))
  (team-color ?team-color)
  (input-storage ?team-color ?ins ? ?)
  =>
  ;the preperation is finished when someone stands at ins
  (if (eq ?ins (sym-cat (pb-field-value ?p "waiting_point")))
      then
      (retract ?s)
      (assert (state EXP_PREPARE_FOR_PRODUCTION_FINISHED))
  )
)

(defrule exp-nav-gen-test
  ?n <- (nav-gen-test)
  =>
  (retract ?n)
  (bind ?msg (blackboard-create-msg "NavGraphWithMPSGeneratorInterface::/navgraph-generator-mps" "UpdateStationByTagMessage"))
  (blackboard-set-msg-field ?msg "id" "CCS")
  (blackboard-set-msg-field ?msg "side" INPUT)
  (blackboard-set-msg-field ?msg "frame" "/map")
  (blackboard-set-msg-multifield ?msg "tag_translation" (create$ 0 0 0))
  (blackboard-set-msg-multifield ?msg "tag_rotation" (create$ 0 0 0 1))
  (blackboard-send-msg ?msg)
  (bind ?msg-2 (blackboard-create-msg "NavGraphWithMPSGeneratorInterface::/navgraph-generator-mps" "UpdateStationByTagMessage"))
  (blackboard-set-msg-field ?msg-2 "id" "MCS")
  (blackboard-set-msg-field ?msg-2 "side" INPUT)
  (blackboard-set-msg-field ?msg-2 "frame" "/map")
  (blackboard-set-msg-multifield ?msg-2 "tag_translation" (create$ 3 3 0))
  (blackboard-set-msg-multifield ?msg-2 "tag_rotation" (create$ 0 0 0.7 0.7))
  (blackboard-send-msg ?msg-2)
)
