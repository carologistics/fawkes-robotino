
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
  (confval (path "/clips-agent/rcll2016/exploration/row-high") (list-value $?row-high))
  (confval (path "/clips-agent/rcll2016/exploration/row-mid") (list-value $?row-mid))
  (confval (path "/clips-agent/rcll2016/exploration/row-low") (list-value $?row-low))
  (confval (path "/clips-agent/rcll2016/exploration/row") (value ?agent-row))
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
)

(defrule exp-goto-first
  "Robotino drives to the first machine to start the first round."
  (phase EXPLORATION)
  ?s <- (state EXP_START)
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

(defrule exp-explore-zone-skill-finished
  "Arriving at a machine in first or second round. Wait for tag vision."
  (phase EXPLORATION)
  ?final <- (skill-done (name "explore_zone") (status ?explore-zone-state&FINAL|FAILED))
  ?s <- (state EXP_DRIVING_TO_MACHINE)
  (time $?now)
  =>
  (printout t "Arrived. explore_zone was " ?explore-zone-state " Looking for a tag." crlf)
  (retract ?s ?final)
  (assert (state EXP_LOOKING_FOR_TAG)
          (explore-zone-state ?explore-zone-state)
  )
)

(defrule exp-read-tag-from-exp-point
  "Read found tag in front of robot and drive nearer to the tag to get a better reading or prepare adding the tag to the navgraph when already driven to the tag."
  (phase EXPLORATION)
  ?s <- (state EXP_LOOKING_FOR_TAG)
  ?g <- (goalmachine ?old)
  ?ze <- (zone-exploration (name ?old))
  (confval (path "/clips-agent/rcll2016/exploration/needed-visibility-history") (value ?needed-vh))
  (team-color ?team-color)
  (tag-matching (tag-id ?tag) (team ?team-color) (machine ?machine) (side ?side))
  (not (found-tag (name ?machine)))
  (last-zoneinfo (search-state YES) (tag-id ?tag))
  ?tagpos <- (Position3DInterface (id "/explore-zone/pose")
          (translation $?trans) (rotation $?rot)
				  (frame ?frame) (time $?timestamp))
  ?skill-finish-state <- (explore-zone-state ?explore-zone-state)
  (time $?now)
  =>
  (printout t "Found Tag Nr." ?tag " (" ?machine " " ?side ")"  crlf)
  (printout t "Recognized Tag" crlf)
  ; transform to map-frame
  (if (tf-can-transform "/map" ?frame ?timestamp) then
    (bind ?tf-transrot (tf-transform-pose "/map" ?frame ?timestamp ?trans ?rot))
    else
    (printout warn "Can not transform " ?frame " to /map. Trying most current time" crlf)
    (if (tf-can-transform "/map" ?frame (create$ 0 0)) then
      (bind ?tf-transrot (tf-transform-pose "/map" ?frame (create$ 0 0) ?trans ?rot))
      else
      (printout error "Can not transform " ?frame " to /map. Tags positions are broken!!!" crlf)
      (printout error "Check time diff between base and laptop" crlf)
      (assert (state EXP_IDLE)
              (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?old)))
      (retract ?s ?tagpos ?skill-finish-state)
      (return)
    )
  )
  (if (eq 7 (length$ ?tf-transrot)) then
    (synced-assert (str-cat "(found-tag (name " ?machine ") (side " ?side 
                            ") (frame \"/map\") (trans (create$ "
                            (implode$ (subseq$ ?tf-transrot 1 3)) ")) "
                            " (rot (create$ " (implode$ (subseq$ ?tf-transrot 4 7))
                            ")))"))
    else
    (printout error "Can not transform " ?frame " to /map. Transform is empty. Tags positions are broken!!!" crlf)
    (printout error "Check time diff between base and laptop" crlf)
    (assert (state EXP_IDLE)
            (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?old)))
    (retract ?s ?tagpos ?skill-finish-state)
    (return)
  )
  (assert (state EXP_ADD_TAG))
  (retract ?s ?tagpos ?skill-finish-state)
)

(defrule exp-report-found-tag
  (phase EXPLORATION)
  ?s <- (state EXP_ADD_TAG)
  ?g <- (goalmachine ?old)
  ?ze <- (zone-exploration (name ?old))
  (team-color ?team-color)
  (tag-matching (tag-id ?tag) (team ?team-color) (machine ?machine) (side ?side))
  ?ft <- (found-tag (name ?machine) (side ?side) (frame ?frame)
		    (trans $?trans) (rot $?rot))
  (not (navgraph-added-for-mps (name ?machine)))
  =>
  (printout t "Add Tag Nr." ?tag " (" ?machine " " ?side ") to Navgraph-generation"  crlf)

  (navgraph-add-all-new-tags)

  (printout t "Wait until ComputeMessage is processed" crlf)
  (retract ?s)
  (assert (state EXP_WAIT_BEFORE_DRIVE_TO_OUTPUT)
          (exp-current-zone (name ?machine)))
)

(defrule update-zone-info
  "Update zone info from ZoneInterface"
  ?zi <- (ZoneInterface (id "/explore-zone/info") (search_state ?state) (tag_id ?tag))
  ?lzi <- (last-zoneinfo (tag-id ?old-tag) 
    (search-state ?old-state&:(or (neq ?old-state ?state) (neq ?old-tag ?tag))))
  =>
  (retract ?lzi ?zi)
  (assert (last-zoneinfo (search-state ?state) (tag-id ?tag)))
)

(defrule exp-no-tag-found
  "We couldn't see a tag => try next look-pos or skip zone"
  (phase EXPLORATION)
  (time $?now)
  ?s <- (state EXP_LOOKING_FOR_TAG)
  ?g <- (goalmachine ?old)
  (last-zoneinfo (search-state NO))
  ?ze <- (zone-exploration (name ?old) (look-pos $?lp)
                           (times-searched ?times-searched))
  ?skill-finish-state <- (explore-zone-state ?explore-zone-state)
  =>
  (printout t "Found no tag in zone " ?old crlf)

  ; if explore_zone was final there probably is no mps in this zone
  (if (eq ?explore-zone-state FINAL) then
    (printout t "There probably is no mps in this zone!" crlf)
    (printout t "Try this zone again later ROBOCUP FIX!" crlf)
    ; count 2 times to first explore "maybe zones"
    (synced-modify ?ze times-searched (+ 2 ?times-searched))
  )

  (assert (state EXP_IDLE)
	  (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?old))
  )
  (retract ?s ?skill-finish-state)
)

(defrule exp-maybe-tag-found
  "We maybe see a tag => try next look-pos or skip zone"
  (phase EXPLORATION)
  (time $?now)
  ?s <- (state EXP_LOOKING_FOR_TAG)
  ?g <- (goalmachine ?old)
  (last-zoneinfo (search-state MAYBE|UNKNOWN))
  ?ze <- (zone-exploration (name ?old) (look-pos $?lp)
                           (times-searched ?times-searched))
  ?skill-finish-state <- (explore-zone-state ?explore-zone-state)
  =>
  (printout t "Maybe found tag in zone " ?old crlf)

  ; if explore_zone was final there probably is no mps in this zone
  (if (eq ?explore-zone-state (or FINAL FAILED)) then
    (printout t "There probably is no mps in this zone!" crlf)
    (printout t "Try this zone again later ROBOCUP FIX!" crlf)
    (synced-modify ?ze times-searched (+ 1 ?times-searched))
  )

  (assert (state EXP_IDLE)
	  (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?old))
  )  
  (retract ?s ?skill-finish-state)
)

(defrule exp-drive-to-output-tag-zone-correct
  "Drive to the output tag of a found machine, so that we can align in front of the light-signal afterwards. Case if the detected tag is in the zone we are currently exploring."
  (phase EXPLORATION)
  ?s <- (state EXP_WAIT_BEFORE_DRIVE_TO_OUTPUT)
  ?mtfz <- (exp-current-zone (name ?machine))
  (goalmachine ?zone)
  (zone-exploration (name ?zone) (machine ?machine))
  =>
  (printout t "Driving to MPS light-signal." crlf)
  (retract ?s ?mtfz)
  (assert (state EXP_DRIVE_TO_OUTPUT))
  (skill-call drive_to place (get-light-signal-side ?machine))
)

(defrule exp-drive-to-output-tag-zone-different
  "Drive to the output tag of a found machine, so that we can align in front of the light-signal afterwards. Case if the detected tag is in a different zone than we are currently exploring. We switch to the nearest tactic so the find-nearest rule can select this mps or another mps with found tag."
  (phase EXPLORATION)
  ?s <- (state EXP_WAIT_BEFORE_DRIVE_TO_OUTPUT)
  ?mtfz <- (exp-current-zone (name ?machine))
  (goalmachine ?zone)
  (zone-exploration (name ~?zone) (machine ?machine))
  ?tactic <- (exp-tactic ?)
  =>
  (printout error "Detected MPS is in a different zone than we are currently exploring. Skipping exploring the light now." crlf)
  (retract ?s ?mtfz ?tactic)
  (assert (state EXP_IDLE)
	  (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?zone))
          (exp-tactic NEAREST))
)

(defrule exp-explore-light-signal
  "Preparing recognition of the light signals after we arrived at the output side."
  (phase EXPLORATION)
  ?final <- (skill-done (name "drive_to") (status FINAL)) 
  ?s <- (state EXP_DRIVE_TO_OUTPUT)
  (goalmachine ?zone)
  (zone-exploration (name ?zone) (machine ?machine))
  =>
  (printout t "Explore light signal." crlf)
  (retract ?s ?final)
  (assert (state EXP_DETECT_LIGHT))
  (skill-call mps_detect_signal place ?machine)
)

(defrule exp-explore-light-signal-drive-to-failed
  "The robot was not able to drive to the output side, skip this zone for now"
  (phase EXPLORATION)
  ?final <- (skill-done (name "drive_to") (status FAILED)) 
  ?s <- (state EXP_DRIVE_TO_OUTPUT)
  (zone-exploration (name ?zone) (machine ?machine))
  ?g <- (goalmachine ?zone)
  =>
  (printout t "Driving to the light signal failed. Skipping this zone for now." crlf)
  (retract ?s ?final)
  (assert (state EXP_IDLE)
    (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?zone))
  )
)

(defrule exp-explore-light-signal-finished
  "RobotinoLightInterface has recognized the light-signals => memorize light-signals for further rules and prepare to drive to the next machine."
  (phase EXPLORATION)
  ?final <- (skill-done (name "mps_detect_signal") (status FINAL)) 
  ?l-signal <- (last-lights (red ?red) (yellow ?yellow) (green ?green))
  ?s <- (state EXP_DETECT_LIGHT)
  ?g <- (goalmachine ?zone)
  (zone-exploration (name ?zone) (machine ?machine))
  ?exp <- (exp-matching (red ?red) (yellow ?yellow) (green ?green) (mtype ?mtype) (found FALSE|MAYBE))
  =>
  (printout t "Explored light done." crlf)
  (printout warn "TODO: ensure that tag is in the explored zone" crlf)
  (printout warn "TODO: if failed, check again later" crlf)
  (retract ?s ?final ?l-signal)
  (assert (state EXP_IDLE)
    (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?zone))
  )
  (printout t "Read light: red: " ?red " yellow: " ?yellow " green: " ?green crlf)
  (assert (exploration-result (machine ?machine) (mtype ?mtype) (zone ?zone)))
  (synced-modify ?exp found MAYBE machine ?machine)
)

(defrule exp-explore-light-signal-failed
  "RobotinoLightInterface has *not* recognized the light-signals."
  (phase EXPLORATION)
  ?final <- (skill-done (name "mps_detect_signal") (status FAILED)) 
  ?s <- (state EXP_DETECT_LIGHT)
  ?g <- (goalmachine ?zone)
  (zone-exploration (name ?zone) (machine ?machine))
  =>
  (printout t "Explored light done." crlf)
  (retract ?s ?final)
  (assert (state EXP_IDLE)
    (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?zone))
  )
  (printout warn "Couldn't read light." crlf)
)

(defrule exp-explore-light-signal-wrong-signal
  "RobotinoLightInterface has recognized the light-signals => memorize light-signals for further rules and prepare to drive to the next machine."
  (phase EXPLORATION)
  ?final <- (skill-done (name "mps_detect_signal") (status FINAL)) 
  ?l-signal <- (last-lights (red ?red) (yellow ?yellow) (green ?green))
  ?s <- (state EXP_DETECT_LIGHT)
  ?g <- (goalmachine ?zone)
  (zone-exploration (name ?zone) (machine ?machine))
  (not (exp-matching (red ?red) (yellow ?yellow) (green ?green) (mtype ?mtype) (found FALSE|MAYBE)))
  =>
  (printout t "Explored light done." crlf)
  (retract ?s ?final ?l-signal)
  (assert (state EXP_IDLE)
    (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?zone))
  )
  (printout error "Light signal combination unknown: machine: " ?machine " red: " ?red " yellow: " ?yellow " green: " ?green crlf)
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

(defrule exp-find-next-machine-with-already-found-tag
  "When there is a mps with found tag that is not recognized, explore it first."
  (declare (salience ?*PRIORITY-EXP-PARTLY-EXPLORED*)) 
  (phase EXPLORATION)
  (exp-tactic NEAREST)
  ?s <- (state EXP_IDLE)
  ?g <- (goalmachine ?old)
  (team-color ?team-color)
  (zone-exploration (name ?zone) (team ?team) (still-to-explore FALSE)
                    (recognized FALSE) (machine ?mps&~UNKNOWN)
                    ; dont keep searching this machine forever
                    (times-searched ?times-searched&:(< ?times-searched 3)))
  (found-tag (name ?mps))
  (not (exploration-result (machine ?mps)))
  (not (locked-resource (resource ?zone)))
  =>
  ;we found the next machine
  (printout warn "Found next (Tag already found): " ?zone crlf)
  (retract ?s ?g)
  (assert (state EXP_FOUND_NEXT_MACHINE)
          (exp-next-machine ?zone)
  )
)

(defrule exp-find-next-machine-nearest
  "Find nearest machine close to last machine. When there is a mps with found tag that is not recognized, explore it first."
  (declare (salience ?*PRIORITY-EXP-NEAREST*)) 
  (phase EXPLORATION)
  (exp-tactic NEAREST)
  ?s <- (state EXP_IDLE)
  ?g <- (goalmachine ?old)
  (team-color ?team-color)
  (zone-exploration (name ?old) (x ?x) (y ?y) (team ?team))
  =>
  ; what is the fewest times a zone was searched 
  (bind ?min-times-searched 1000)
  (delayed-do-for-all-facts ((?me zone-exploration))
    (and (eq ?me:team ?team-color) (not ?me:recognized)
	 (not (any-factp ((?mt exploration-result)) (eq ?mt:machine ?me:name))))
    (if (and (not (any-factp ((?lock locked-resource)) (eq ?lock:resource ?me:name)))
             (< ?me:times-searched ?min-times-searched))
      then
      (bind ?min-times-searched ?me:times-searched)
    )
  )
  ; find next machine nearest to last machine with min times searched
  (bind ?nearest NONE)
  (bind ?min-dist 1000.0)
  (do-for-all-facts ((?me zone-exploration))
    (and (eq ?me:team ?team-color) (not ?me:recognized)
         (eq ?me:times-searched ?min-times-searched)
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

(defrule exp-go-to-next-zone-to-find-tag
  "When lock was accepted move to the exploration position to find a tag"
  (phase EXPLORATION)
  ?s <- (state EXP_LOCK_ACCEPTED)
  ?n <- (exp-next-machine ?next-zone)
  ?ze <- (zone-exploration (name ?next-zone) (x ?) (y ?) (next ?)
                           (look-pos $?lp) (current-look-pos ?lp-index)
                           (still-to-explore TRUE) (times-searched ?times-searched))
  (not (driven-to-waiting-point))
  (team-color ?team)
  =>
  (printout t "Going to next zone: " ?next-zone crlf)
  (retract ?s ?n)
  (assert (state EXP_DRIVING_TO_MACHINE)
          (goalmachine ?next-zone))
  (synced-modify ?ze times-searched (+ 1 ?times-searched))
  (bind ?zone-boarders (utils-get-zone-edges ?next-zone))
  (bind ?search-tags (utils-get-tags-str-still-to-explore ?team))
  (if (eq ?next-zone Z16) then
    ; change explore_zone order because the mps is placed very similar every time
    (skill-call explore_zone min_x (nth$ 1 ?zone-boarders) max_x (nth$ 2 ?zone-boarders)
                min_y (nth$ 3 ?zone-boarders) max_y (nth$ 4 ?zone-boarders)
                search_tags ?search-tags change_cluster_view true)
    else
    (skill-call explore_zone min_x (nth$ 1 ?zone-boarders) max_x (nth$ 2 ?zone-boarders)
                min_y (nth$ 3 ?zone-boarders) max_y (nth$ 4 ?zone-boarders)
                search_tags ?search-tags)
  )
)

(defrule exp-go-to-next-machine-with-a-found-tag
  "When lock was accepted and we already know the tag position
   skip finding the tag to directly explore the light signal"
  (phase EXPLORATION)
  ?s <- (state EXP_LOCK_ACCEPTED)
  ?n <- (exp-next-machine ?nextMachine)
  ?ze <- (zone-exploration (name ?nextMachine) (x ?) (y ?) (next ?)
                           (look-pos $?lp) (current-look-pos ?lp-index)
                           (still-to-explore FALSE) (times-searched ?times-searched))
  (not (driven-to-waiting-point))
  =>
  (printout t "Going to next machine, skip finding tag." crlf)
  (retract ?s ?n)
  (assert (state EXP_SKIP_FIND_TAG)
          (goalmachine ?nextMachine))
  (synced-modify ?ze times-searched (+ 1 ?times-searched))
)

(defrule exp-tag-found-by-other-robot
  "When we are driving to find a tag and another bot finds an mps in the zone we are driving to, drive directly to look for the light"
  (phase EXPLORATION)
  ?s <- (state EXP_DRIVING_TO_MACHINE)
  (goalmachine ?zone)
  (zone-exploration (name ?zone) (machine ?machine&~UNKNOWN))
  =>
  (printout t "Skip finding tag in " ?zone crlf)
  (retract ?s)
  (assert (state EXP_SKIP_FIND_TAG))
)

(defrule exp-drive-directly-to-detect-the-light
  "The machine in the zone was already found. Drive direcly to the machine to detect the light."
  (phase EXPLORATION)
  ?s <- (state EXP_SKIP_FIND_TAG)
  (goalmachine ?zone)
  (zone-exploration (name ?zone) (machine ?machine&~UNKNOWN))
  =>
  (printout t "Directly explore light-signal" crlf)
  (retract ?s)
  (assert (state EXP_DRIVE_TO_OUTPUT))
  (skill-call drive_to place (get-light-signal-side ?machine))
)

(defrule exp-receive-type-light-pattern-matching
  "Receive light-pattern-to-type matching and save it in a fact."
  (phase EXPLORATION)
  ?pbm <- (protobuf-msg (type "llsf_msgs.ExplorationInfo") (ptr ?p))
  (not (have-exp-info))
  (lock-role ?role)
  =>
  (retract ?pbm)
  (foreach ?m (pb-field-list ?p "zones")
    (bind ?name (sym-cat (pb-field-value ?m "zone")))
    (bind ?team (sym-cat (pb-field-value ?m "team_color")))
    (do-for-fact ((?me zone-exploration)) (eq ?me:name ?name)
      (printout t "Zone " ?name " is from team " ?team crlf)
      (modify ?me (team ?team))
    )
  )
  (if (eq ?role MASTER) then 
    (foreach ?sig (pb-field-list ?p "signals")
      (bind ?mtype (pb-field-value ?sig "type"))
      (bind ?red BLINKING)
      (bind ?yellow BLINKING)
      (bind ?green BLINKING)
      (progn$ (?light (pb-field-list ?sig "lights"))
        (bind ?light-state (pb-field-value ?light "state"))
        (switch (sym-cat (pb-field-value ?light "color"))
          (case RED then (bind ?red ?light-state))
          (case YELLOW then (bind ?yellow ?light-state))
          (case GREEN then (bind ?green ?light-state))
        )
      )
      (assert (exp-matching (mtype ?mtype) (red ?red) (yellow ?yellow) (green ?green) (found FALSE)))
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
  (confval (path "/clips-agent/rcll2016/exploration/latest-send-last-report-time")
	   (value ?latest-report-time))
  (team-color ?team-color&~nil)
  (peer-id private ?peer)
  =>
  (bind ?mr (pb-create "llsf_msgs.MachineReport"))
  (pb-set-field ?mr "team_color" ?team-color)
  (do-for-all-facts ((?machine exploration-result)) TRUE
    ;send report for last machine only if the exploration phase is going to end
    ;or we are prepared for production
    (if (or
	 (< (length (find-all-facts ((?f zone-exploration))
				    (and (eq ?f:team ?team-color) ?f:recognized)))
	    (- (length (find-all-facts ((?f zone-exploration)) (eq ?f:team ?team-color))) 1))
	 (>= (nth$ 1 ?game-time) ?latest-report-time)
	 (eq ?s EXP_PREPARE_FOR_PRODUCTION_FINISHED))
     then
      (bind ?mre (pb-create "llsf_msgs.MachineReportEntry"))
      (pb-set-field ?mre "name" (str-cat ?machine:machine))
      (pb-set-field ?mre "type" (str-cat ?machine:mtype))
      (pb-set-field ?mre "zone" (str-cat ?machine:zone))
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
  "When locks for pre-game positions are acquired, drive there with drive_to."
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
  ?skill-f <- (skill-done (name "ppgoto") (status FINAL))
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

(defrule exp-add-missing-tag-from-other-team
  "If the exploration finished and found only the mps of our team and not the mirrowed mps of the other team, we just add it as if the mps would stand there perfectly mirrowed."
  (phase PRODUCTION)
  (found-tag (name ?mps) (frame "/map") (trans $?trans) (rot $?rot) (side ?side))
  (machine (name ?mps) (team ?team&~nil))
  ;check that the mps of the other team was not found
  (not (found-tag (name ?mps-mirrow&:(and (neq ?mps ?mps-mirrow)
					  (eq (sub-string 2 (str-length (str-cat ?mps)) (str-cat ?mps))
					      (sub-string 2 (str-length (str-cat ?mps-mirrow)) (str-cat ?mps-mirrow)))))))
  =>
  (printout warn "Could not explore the machines of the other team, adding them as they would be perfectly mirrowed" crlf)
  ; mirrowing the position is easy
  (bind ?trans-mirrow (create$ (- 0 (nth$ 1 ?trans)) (nth$ 2 ?trans) 0))
  ; mirrowing the orientation can be done easily in the euler-space
  (bind ?yaw (tf-yaw-from-quat ?rot))
  (bind ?yaw-mirrow (+ (- 0 (- ?yaw ?*PI-HALF*)) ?*PI-HALF*))
  (if (> ?yaw-mirrow ?*PI*) then
    (bind ?yaw-mirrow (- ?yaw-mirrow ?*2PI*)))
  (if (< ?yaw-mirrow (- 0 ?*PI*)) then
    (bind ?yaw-mirrow (+ ?yaw-mirrow ?*2PI*)))
  (bind ?rot-mirrow (tf-quat-from-yaw ?yaw-mirrow))
  ; get name of the mirrowed mps
  (bind ?mps-mirrow (sym-cat (sub-string 2 (str-length ?mps) ?mps)))
  (if (eq ?team CYAN) then
    (bind ?mps-mirrow (sym-cat "M" ?mps-mirrow))
    else
    (bind ?mps-mirrow (sym-cat "C" ?mps-mirrow))
  )
  ;assert mirrowed tag
  (assert (found-tag (name ?mps-mirrow) (side ?side) (frame "/map")
                     (trans ?trans-mirrow) (rot ?rot-mirrow)))
)

(defrule exp-add-exp-zone-mps-from-other-team
  "If the exploration finished and we know the zones of our mpss we can derive in which zone the others team mpss are."
  (phase PRODUCTION)
  (machine (name ?mps) (team ?team&~nil))
  (zone-exploration (name ?zone) (machine ?mps))
  (not (zone-exploration (machine ?mps-mirrow&:(and (neq ?mps ?mps-mirrow)
					  (eq (sub-string 2 (str-length (str-cat ?mps)) (str-cat ?mps))
					      (sub-string 2 (str-length (str-cat ?mps-mirrow)) (str-cat ?mps-mirrow)))))))
  =>
  ; get name of the mirrowed mps
  (bind ?mps-mirrow (sym-cat (sub-string 2 (str-length ?mps) ?mps)))
  (if (eq ?team CYAN) then
    (bind ?mps-mirrow (sym-cat "M" ?mps-mirrow))
    else
    (bind ?mps-mirrow (sym-cat "C" ?mps-mirrow))
  )
  ;set zone-exploration
  (bind ?zone-int (eval (sub-string 2 (str-length (str-cat ?zone)) (str-cat ?zone))))
  (if (<= ?zone-int 12) then
    (bind ?zone-mirrow (sym-cat Z (+ ?zone-int 12)))
    else
    (bind ?zone-mirrow (sym-cat Z (- ?zone-int 12)))
  )
  (printout t "Zone of Mirrowed mps of " ?mps "," ?zone " is " ?zone-mirrow crlf)
  (do-for-fact ((?zone-expl-mir zone-exploration)) (eq ?zone-expl-mir:name ?zone-mirrow)
    (modify ?zone-expl-mir (machine ?mps-mirrow))
  )
)

; (defrule exp-remove-old-skill-facts
;   "If the exploration has passed and there are still some leftovers, remove them"
;   (phase PRODUCTION)
;   ?old <- (skill (name "explore_zone"))
;   ?new <- (skill (name ?something&:(neq ?old ?new)))
;   =>
;   (retract ?old)
; )

(defrule exp-report-was-right
  "If we reported a machine and the report was right, save this information to the fact-base"
  (phase EXPLORATION)
  ?exp <- (exp-matching (found MAYBE))
  (team-color ?team)
  (points ?team ?p)
  (lock-role MASTER)
  =>
  ;check that all reports were right
  (bind ?correct-points 0)
  (bind ?maybe-points 0)
  (do-for-all-facts ((?expmatch exp-matching)) TRUE
    (if (eq ?expmatch:found TRUE) then
      (bind ?correct-points (+ ?correct-points 8))
    )
    (if (eq ?expmatch:found MAYBE) then
      (bind ?maybe-points (+ ?maybe-points 8))
    )
  )
  (if (eq ?p (+ ?correct-points ?maybe-points)) then
    ; all maybe's are right 
    (delayed-do-for-all-facts ((?expmatch exp-matching)) (eq ?expmatch:found MAYBE)
      (synced-modify ?expmatch found TRUE)
    )
  )
)


(defrule exp-conclude-ds-report
  "When we found and reported all machines except for the DS, conclude the correct report"
  (phase EXPLORATION)
  (team-color ?team)
  (points ?team 40)
  (lock-role MASTER)
  (machine (mtype DS) (name ?ds) (team ?team))
  ?exp <- (exp-matching (found FALSE) (mtype ?mtype))
  (exp-matching (found TRUE) (machine C-BS|M-BS))
  (exp-matching (found TRUE) (machine C-CS1|M-CS1))
  (exp-matching (found TRUE) (machine C-CS2|M-CS2))
  (exp-matching (found TRUE) (machine C-RS1|M-RS1))
  (exp-matching (found TRUE) (machine C-RS2|M-RS2))
  =>
  (if (eq ?team CYAN) then
    (bind ?zone Z4)
    else
    (bind ?zone Z16)
  )
  (assert (exploration-result (machine ?ds) (mtype ?mtype) (zone ?zone)))
)

