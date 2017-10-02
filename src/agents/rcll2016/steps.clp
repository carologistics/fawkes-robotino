;---------------------------------------------------------------------------
;  steps.clp - rules how to execute the concrete steps of tasks
;
;  Created: Sat Feb 07 19:43:19 2015
;  Copyright  2015 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defrule step-drive-to-start
  (declare (salience ?*PRIORITY-STEP-START*))
  (phase PRODUCTION)
  (task (state running) (robot ?r&:(eq ?r ?*ROBOT-NAME*)) (current-step ?step-id))
  ?step <- (step (name drive-to) (state wait-for-activation) (task-priority ?p) (id ?step-id)
    (machine ?mps) (side ?side))
  ?state <- (state STEP-STARTED)
  =>
  (retract ?state)
  (synced-modify ?step state running)
  (if (eq ?side INPUT) then
    (bind ?place (sym-cat ?mps "-I"))
    else
    (bind ?place (sym-cat ?mps "-O"))
  )
  (assert (state WAIT-FOR-LOCK)
    (wait-for-lock (priority ?p) (res ?place))
    (skill-to-execute (skill drive_to) (args place ?place) (target ?mps))
  )
)

(defrule step-get-from-shelf
  (declare (salience ?*PRIORITY-STEP-START*))
  (phase PRODUCTION)
  (task (state running) (robot ?r&:(eq ?r ?*ROBOT-NAME*)) (current-step ?step-id))
  ?step <- (step (name get-from-shelf) (state wait-for-activation) (task-priority ?p) (id ?step-id)
		 (machine ?mps) (machine-feature SHELF))
  ?state <- (state STEP-STARTED)
  (team-color ?team)
  (game-time $?game-time)
  =>
  (retract ?state)
  (synced-modify ?step state running)
  (printout warn "TODO: Pick dynamically from different shelf positions." crlf)
  (assert (state WAIT-FOR-LOCK)
	  (skill-to-execute (skill get_product_from) (args place ?mps shelf TRUE) (target ?mps))
	  (wait-for-lock (priority ?p) (res (sym-cat ?mps "-I")))
  )
)

(defrule step-insert-start
  (declare (salience ?*PRIORITY-STEP-START*))
  (phase PRODUCTION)
  (task (state running) (robot ?r&:(eq ?r ?*ROBOT-NAME*)) (current-step ?step-id))
  ?step <- (step (id ?step-id) (name insert) (state wait-for-activation) (task-priority ?p)
                 (machine ?mps) (machine-feature ?feature&~SLIDE) (gate ?gate) (ring ?ring)
                 (already-at-mps ?already-at-mps))
  (machine (name ?mps) (mtype ?mtype) (state IDLE|PREPARED))
  (task (name ?task-name)  (robot ?r&:(eq ?r ?*ROBOT-NAME*)))
  ?state <- (state STEP-STARTED)
  (team-color ?team)
  (game-time $?game-time)
  =>
  (retract ?state)
  (synced-modify ?step state running)
  (assert (state WAIT-FOR-LOCK)
    ; default side of machine is input, thus we don't need it here
	  (wait-for-lock (priority ?p) (res (sym-cat ?mps "-I")))
  )
  (bind ?args (create$ place ?mps))
  
  (if ?already-at-mps then
    ;the shelf position we used last can be derived from the number of caps on the shelf
    (bind ?sslot MIDDLE)
    (do-for-fact ((?cs cap-station)) (eq ?cs:name ?mps)
      (switch ?cs:caps-on-shelf
        (case 3 then (bind ?sslot LEFT))
        (case 2 then (bind ?sslot RIGHT))
        (case 1 then (bind ?sslot MIDDLE))
      )
    )
    (bind ?args (create$ ?args atmps ?sslot))
  )
  
  (assert (skill-to-execute (skill bring_product_to) (args ?args) (target ?mps)))
  ; check if we have to instruct an mps:
  (if (and (eq ?mtype CS)
           (eq ?task-name fill-cap)) then
    (assert (mps-instruction (machine ?mps) (cs-operation RETRIEVE_CAP) (lock (sym-cat ?mps "-I"))))
  )
  (if (and (eq ?mtype CS)
           (member$ ?task-name (create$ produce-c0 produce-cx deliver))) then
    (assert (mps-instruction (machine ?mps) (cs-operation MOUNT_CAP) (lock (sym-cat ?mps "-I"))))
  )
  (if (and (eq ?mtype DS)
           (eq ?task-name deliver)) then
    (assert (mps-instruction (machine ?mps) (gate ?gate) (lock (sym-cat ?mps "-I"))))
  )
  (if (and (eq ?mtype RS)
           (member$ ?task-name (create$ add-first-ring add-additional-ring))) then
    (assert (mps-instruction (machine ?mps) (ring-color ?ring) (lock (sym-cat ?mps "-I"))))
  )
)

(defrule step-insert-slide-start
  (declare (salience ?*PRIORITY-STEP-START*))
  (phase PRODUCTION)
  (task (state running) (robot ?r&:(eq ?r ?*ROBOT-NAME*)) (current-step ?step-id))
  ?step <- (step (id ?step-id) (name insert) (state wait-for-activation) (task-priority ?p)
                 (machine ?mps) (machine-feature SLIDE) (gate ?gate) (ring ?ring)
                 (already-at-mps ?already-at-mps))
  (machine (name ?mps) (mtype ?mtype) (state ~DOWN))
  (ring-station (name ?mps) (bases-loaded ?bl&:(< ?bl 3)))
  (task (name ?task-name)  (robot ?r&:(eq ?r ?*ROBOT-NAME*)))
  ?state <- (state STEP-STARTED)
  (team-color ?team)
  (game-time $?game-time)
  =>
  (retract ?state)
  (synced-modify ?step state running)
  (assert (state WAIT-FOR-LOCK)
    ; default side of machine is input, thus we don't need it here
	  (wait-for-lock (priority ?p) (res (sym-cat ?mps "-I")))
  )
  (bind ?args (create$ place ?mps))
  (bind ?args (create$ ?args slide TRUE))
  (assert (skill-to-execute (skill bring_product_to) (args ?args) (target ?mps)))
)

(defrule step-get-output-start
  (declare (salience ?*PRIORITY-STEP-START*))
  (phase PRODUCTION)
  (task (state running) (robot ?r&:(eq ?r ?*ROBOT-NAME*)) (current-step ?step-id))
  ?step <- (step (name get-output) (state wait-for-activation) (task-priority ?p) (id ?step-id)
		 (machine ?mps) (side ?side))
  ?state <- (state STEP-STARTED)
  (team-color ?team)
  (game-time $?game-time)
  (machine (name ?mps) (mtype ?type) (state IDLE))
  =>
  (retract ?state)
  (synced-modify ?step state running)
  (if (and (eq ?type BS) (eq ?side INPUT)) then
    (bind ?res (sym-cat ?mps "-I"))
  else
    (bind ?res (sym-cat ?mps "-O"))
  )
  (assert (state WAIT-FOR-LOCK)
	  (skill-to-execute (skill get_product_from) (args place ?mps side (lowcase ?side)) (target ?mps))
	  (wait-for-lock (priority ?p) (res ?res))
  )
)

(defrule step-get-base-start
  (declare (salience ?*PRIORITY-STEP-START*))
  (phase PRODUCTION)
  (task (state running) (robot ?r&:(eq ?r ?*ROBOT-NAME*)) (current-step ?step-id))
  ?step <- (step (name get-base) (state wait-for-activation) (task-priority ?p) (id ?step-id)
		 (machine ?mps) (machine-feature ?feature) (base ?color) (side ?side))
  ?state <- (state STEP-STARTED)
  (team-color ?team)
  (machine (mtype BS) (name ?mps) (state PROCESSING|READY-AT-OUTPUT))
  ?bsc <- (bs-side-changed)
  =>
  (retract ?state ?bsc)
  (synced-modify ?step state running)
  (if (eq ?side INPUT) then
    (bind ?res (sym-cat ?mps "-I"))
  else
    (bind ?res (sym-cat ?mps "-O"))
  )
  (assert (state WAIT-FOR-LOCK)
    (skill-to-execute (skill get_product_from) (args place ?mps side (lowcase ?side)) (target ?mps))
    (wait-for-lock (priority ?p) (res ?res))
  )
)

(defrule step-deliver-start
  (declare (salience ?*PRIORITY-STEP-START*))
  (phase PRODUCTION)
  (task (name deliver) (state running) (robot ?r&:(eq ?r ?*ROBOT-NAME*)) (current-step ?step-id))
  ?step <- (step (name insert) (state wait-for-activation) (task-priority ?p) (id ?step-id)
		 (machine ?mps) (machine-feature ?feature) (base ?color))
  ?state <- (state STEP-STARTED)
  (team-color ?team)
  (holding ?produced-id)
  (product (id ?produced-id) (product-id ?product-id))
  ?of <- (order (product-id ?product-id) (in-production ?ip) (in-delivery ?id))
  =>
  (synced-modify ?of in-production (- ?ip 1) in-delivery (+ ?id 1))
)

;(defrule step-deliver-abort
;  (declare (salience ?*PRIORITY-STEP-FAILED*))
;  (phase PRODUCTION)
;  (game-time $?game-time)
;  (task (name deliver) (state running))
;  ?step <- (step (name get-output|insert) (state running))
;  ?state <- (state SKILL-EXECUTION)
;  (holding ?produced-id)
;  (product (id ?produced-id) (product-id ?product-id))
;  (order (product-id ?product-id) (in-production ?ip)
;         (end ?end&:(< ?end (- (nth$ 1 ?game-time) ?*DELIVER-ABORT-TIMEOUT*))))
;  ?ste <- (skill-to-execute (state running))
;  =>
;  (printout warn "Abort deliver because the order has expired" crlf)
;  (retract ?state ?ste)
;  (assert (state STEP-FAILED))
;  (modify ?step (state failed))
;)

(defrule step-discard-unknown-start
  "Open gripper to discard unknown base"
  (declare (salience ?*PRIORITY-STEP-START*))
  (phase PRODUCTION)
  (task (state running) (robot ?r&:(eq ?r ?*ROBOT-NAME*)) (current-step ?step-id))
  ?step <- (step (name discard) (state wait-for-activation) (task-priority ?p) (id ?step-id))
  ?state <- (state STEP-STARTED)
  (team-color ?team)
  =>
  (retract ?state)
  (synced-modify ?step state running)
  (assert (state WAIT-FOR-LOCK)
    (skill-to-execute (skill ax12gripper) (args command OPEN) (target NONE))
    (wait-for-lock (priority ?p) (res NONE))
  )
)

(defrule step-discard-unknown-finish
  "Unknown base discarded, release locks."
  (declare (salience ?*PRIORITY-STEP-FINISH*))
  (phase PRODUCTION)
  (task (state running) (robot ?r&:(eq ?r ?*ROBOT-NAME*)) (current-step ?step-id))
  ?step <- (step (name discard) (state running) (id ?step-id))
  ?state <- (state SKILL-FINAL)
  ?ste <- (skill-to-execute (skill ax12gripper)
			    (args $?args) (state ?skill-finish-state&final))
  ?h <- (holding ?product-id)
  =>
  (printout t "Base discarded"  crlf)
  (retract ?state ?ste ?h)
  (assert
    (state STEP-FINISHED)
    (holding NONE)
  )
  (synced-modify ?step state finished)
)

(defrule step-find-tag-start
  (declare (salience ?*PRIORITY-STEP-START*))
  (phase PRODUCTION)
  (task (state running) (robot ?r&:(eq ?r ?*ROBOT-NAME*)) (current-step ?step-id))
  ?step <- (step (name find-tag) (state wait-for-activation) (task-priority ?p) (id ?step-id)
		 (zone ?zone) (machine ?missing-mps))
  ?state <- (state STEP-STARTED)
  (game-time $?game-time)
  (team-color ?team)
  ?ze <- (zone-exploration (name ?zone) (times-searched ?times-searched))
  =>
  (retract ?state)
  (synced-modify ?step state running)
  (bind ?zone-boarders (utils-get-zone-edges ?zone))
  (bind ?search-tags (utils-get-tags-str-still-to-explore ?team))
  (assert (state WAIT-FOR-LOCK)
	  (skill-to-execute (skill explore_zone) (args min_x (nth$ 1 ?zone-boarders)
                        max_x (nth$ 2 ?zone-boarders)
                                                       min_y (nth$ 3 ?zone-boarders)
                                                       max_y (nth$ 4 ?zone-boarders)
                                                       search_tags ?search-tags)
                            (target ?zone))
	  (wait-for-lock (priority ?p) (res ?zone))
  )
  (synced-modify ?ze times-searched (+ 1 ?times-searched))
)

(defrule step-find-tag-finish
  "tag-find skill finished, try to find tag"
  (declare (salience ?*PRIORITY-STEP-FINISH*))
  (phase PRODUCTION)
  (task (state running) (robot ?r&:(eq ?r ?*ROBOT-NAME*)) (current-step ?step-id))
  (step (name find-tag) (state running) (id ?step-id))
  ?state <- (state SKILL-FINAL|SKILL-FAILED)
  ?ste <- (skill-to-execute (skill explore_zone)
			    (args $?args) (state ?skill-finish-state&final|failed))
  (time $?now)
  =>
  (printout t "explore_zone was " ?skill-finish-state " Searching for a tag now"  crlf)
  (retract ?state ?ste)
  (assert (state PROD_LOOKING_FOR_TAG)
   (timer (name waiting-for-tag-since) (time ?now) (seq 1))
   (explore-zone-state ?skill-finish-state))
)

(defrule step-find-tag-report-tag
  "Add found tag to navgraph and finish step."
  (phase PRODUCTION)
  (task (state running) (robot ?r&:(eq ?r ?*ROBOT-NAME*)) (current-step ?step-id))
  ?step <- (step (name find-tag) (state running) (zone ?zone) (id ?step-id))
  ?ws <- (timer (name waiting-for-tag-since))
  ?s <- (state PROD_LOOKING_FOR_TAG)
  ?zone-fact <- (zone-exploration (name ?zone))
  (confval (path "/clips-agent/rcll2016/exploration/needed-visibility-history") (value ?needed-vh))
  (tag-matching (tag-id ?tag) (machine ?machine) (side ?side))
  (not (found-tag (name ?machine)))
  (TagVisionInterface (id "/tag-vision/info") (tags_visible ?num-tags&:(> ?num-tags 0))
                      (tag_id $?tag-ids&:(member$ ?tag ?tag-ids)))
  (Position3DInterface (id ?tag-if-id&:(eq ?tag-if-id (str-cat "/tag-vision/" (- (member$ ?tag ?tag-ids) 1)))) 
                       (visibility_history ?vh&:(> ?vh ?needed-vh)) 
                       (translation $?trans) (rotation $?rot)
                       (frame ?frame) (time $?timestamp))
  ?skill-finish-state <- (explore-zone-state ?)
  =>
  (printout t "Found Tag Nr." ?tag " (" ?machine " " ?side ")"  crlf)
  ; transform to map-frame
  (if (tf-can-transform "/map" ?frame ?timestamp) then
    (bind ?tf-transrot (tf-transform-pose "/map" ?frame ?timestamp ?trans ?rot))
    (printout t "Transformed to " ?tf-transrot crlf)
    else
    (printout warn "Can not transform " ?frame " to /map. Trying most current time" crlf)
    (if (tf-can-transform "/map" ?frame (create$ 0 0)) then
      (bind ?tf-transrot (tf-transform-pose "/map" ?frame (create$ 0 0) ?trans ?rot))
      else
      (printout error "Can not transform " ?frame
                " to /map. Tags positions are broken." crlf)
      (printout error "Check time diff between base and laptop" crlf)
      (retract ?s ?ws ?skill-finish-state)
      (assert (state STEP-FAILED))
      (synced-modify ?step state failed)
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
    (retract ?s ?ws ?skill-finish-state)
    (assert (state STEP-FAILED))
    (synced-modify ?step state failed)
    (return)
  )
  (retract ?s ?ws ?skill-finish-state)
  (assert (state STEP-FINISHED))
  (synced-modify ?step state finished)

  (navgraph-add-all-new-tags)
)
(defrule step-find-tag-fail
  "Finish step if there is no tag in this zone."
  (phase PRODUCTION)
  (task (state running) (robot ?r&:(eq ?r ?*ROBOT-NAME*)) (current-step ?step-id))
  ?step <- (step (name find-tag) (state running) (zone ?zone) (machine ?machine) (id ?step-id))
  (time $?now)
  ?ws <- (timer (name waiting-for-tag-since) (time $?t&:(timeout ?now ?t 3.0)))
  ?s <- (state PROD_LOOKING_FOR_TAG)
  ?zone-fact <- (zone-exploration (name ?zone) (times-searched ?times-searched))
  ?skill-finish-state <- (explore-zone-state ?explore-zone-state)
  =>
  (printout t "No tag found in " ?zone crlf)
  (if (eq ?explore-zone-state final) then
    (printout t "There probably is no mps in this zone!" crlf)
    (printout t "Try this zone again later ROBOCUP FIX!" crlf)
    (synced-modify ?zone-fact times-searched (+ 1 ?times-searched))
  )
  (retract ?s ?ws ?skill-finish-state)
  (assert (state STEP-FAILED))
  (synced-modify ?step state failed)
)
(defrule step-find-tag-stop-when-other-bot-found-the-machine
  "tag-find can be stopped, because the machine we want to find
   was already found by another bot"
  (declare (salience ?*PRIORITY-STEP-FINISH*))
  (phase PRODUCTION)
  (task (state running) (robot ?r&:(eq ?r ?*ROBOT-NAME*)) (current-step ?step-id))
  ?step <- (step (name find-tag) (state running) (machine ?machine) (id ?step-id))
  ?state <- (state SKILL-EXECUTION)
  ?ste <- (skill-to-execute (skill drive_to)
			    (args $?args) (state running))
  (time $?now)
  (found-tag (name ?machine))
  =>
  (retract ?state ?ste)
  (assert (state STEP-FINISHED))
  (synced-modify ?step state finished)
  ; skill is not properly stopped, but it should work anyway
  ; because the next skill call overrides it
)

(defrule step-get-product-start
  (declare (salience ?*PRIORITY-STEP-START*))
  (phase PRODUCTION)
  (task (state running) (robot ?r&:(eq ?r ?*ROBOT-NAME*)) (current-step ?step-id))
  ?step <- (step (name get-output) (state wait-for-activation) (task-priority ?p) (id ?step-id)
		 (machine ?mps) (side ?side))
  ?state <- (state STEP-STARTED)
  (team-color ?team)
  =>
  (retract ?state)
  (synced-modify ?step state running)
  (if (eq ?side INPUT) then
    (bind ?res (sym-cat ?mps "-I"))
  else
    (bind ?res (sym-cat ?mps "-O"))
  )
  (assert (state WAIT-FOR-LOCK)
	  (skill-to-execute (skill get_product_from) (args place ?mps side (lowcase ?side)) (target ?mps ))
	  (wait-for-lock (priority ?p) (res ?res))
  )
)

(defrule step-instruct-mps
  (declare (salience ?*PRIORITY-STEP-START*))
  (phase PRODUCTION)
  (task (state running) (robot ?r&:(eq ?r ?*ROBOT-NAME*)) (current-step ?step-id))
  ?step <- (step (name instruct-mps) (state wait-for-activation) (side ?side) (id ?step-id)
                 (machine ?mps) (base ?base) (ring ?ring) (gate ?gate)
                 (cs-operation ?cs-op) (lock ?lock) (task-priority ?p))
  (machine (name ?mps) (mtype ?mtype) (state IDLE))
  ?state <- (state STEP-STARTED)
  (team-color ?team)
  =>
  (retract ?state)
  (switch ?mtype
    (case BS
      then
      (assert (mps-instruction (machine ?mps) (base-color ?base) (side ?side)))
    )
    (case CS
      then
      (assert (mps-instruction (machine ?mps) (cs-operation ?cs-op))))
    (case DS
      then
      (assert (mps-instruction (machine ?mps) (gate ?gate))))
    (case RS
      then
      (assert (mps-instruction (machine ?mps) (ring-color ?ring))))
  )
  (synced-modify ?step state finished)
  (assert (state STEP-FINISHED))
)

(defrule step-fail-machine-broken
  "Fail a step if the machine to be used becomes broken"
  (phase PRODUCTION)
  (task (state running) (robot ?r&:(eq ?r ?*ROBOT-NAME*)) (current-step ?step-id))
  ?step <- (step (name ?step-name) (state running) (machine ?mps) (id ?step-id)
                 (machine-feature ~SHELF))
  (machine (name ?mps) (state BROKEN))
  ?state <- (state SKILL-EXECUTION)
  ?ste <- (skill-to-execute (target ?mps))
  ?wfl <- (wait-for-lock (state use))
  =>
  (printout t "Failing step " ?step-name " because " ?mps " is broken" crlf)
  (synced-modify ?step state failed)
  (modify ?wfl (state finished))
  (retract ?state ?ste)
  (assert (state STEP-FAILED))
)

(defrule step-abort-filling-rs
  "abort filling 4th base into RS (can happen because of race condition)"
  (declare (salience ?*PRIORITY-STEP-FAILED*))
  (phase PRODUCTION)
  (task (state running) (robot ?r&:(eq ?r ?*ROBOT-NAME*)) (current-step ?step-id))
  ?step <- (step (id ?step-id) (name insert) (state running)
                 (machine ?mps) (machine-feature SLIDE)
                 (already-at-mps ?already-at-mps))
  (ring-station (name ?mps) (bases-loaded 3))
  ?state <- (state SKILL-EXECUTION)
  ?ste <- (skill-to-execute (state running))
  =>
  (printout t "Step: Abort filling 4th base into RS" crlf)
  (retract ?state ?ste)
  (assert (state STEP-FAILED))
  (synced-modify ?step state failed)
)

(defrule step-fail-insert-slide
  "skip filling 4th base into RS (can happen because of race condition)"
  (declare (salience ?*PRIORITY-STEP-START*))
  (phase PRODUCTION)
  (task (state running) (robot ?r&:(eq ?r ?*ROBOT-NAME*)) (current-step ?step-id))
  ?step <- (step (id ?step-id) (name insert) (state wait-for-activation)
                 (machine ?mps) (machine-feature SLIDE)
                 (already-at-mps ?already-at-mps))
  (ring-station (name ?mps) (bases-loaded 3))
  ?state <- (state STEP-STARTED)
  =>
  (retract ?state)
  (synced-modify ?step state failed)
  (assert (state STEP-FAILED))
)



(defrule step-wait-for-rs-start
  "We want to wait for an RS to be usable. Drive to a waiting position near the RS."
  (declare (salience ?*PRIORITY-STEP-START*))
  (phase PRODUCTION)
  ?step <- (step (id ?step-id) (name wait-for-rs) (state wait-for-activation) (task-priority ?p)
                 (machine ?mps))
  (machine (name ?mps))
  ?state <- (state STEP-STARTED)
  (game-time $?game-time)
  (place-waitpoint-assignment (place ?place&:(eq ?place (sym-cat ?mps "-I"))) (waitpoint ?waitpoint))
  =>
  (retract ?state)
  (synced-modify ?step state running)
  (assert (state WAIT-FOR-RS)
          (timer (name wait-for-rs-dont-call-two-skills))
  )
  (skill-call ppgoto place (str-cat ?waitpoint))
)

(defrule step-wait-for-rs-lock-before-finish
  "Wait until the RS can be used for production (IDLE, no other robot waiting, and loaded with enough bases).
   Furthermore we need to aquire the lock, to be sure noone commits to using the rs without enough bases."
  (declare (salience ?*PRIORITY-STEP-FINISH*))
  (phase PRODUCTION)
  (task (state running) (id ?task-id)  (robot ?r&:(eq ?r ?*ROBOT-NAME*)))
  (step (name wait-for-rs) (state running) (machine ?rs) (ring ?ring))
  ?state <- (state WAIT-FOR-RS)
  (machine (name ?rs) (state IDLE) (incoming $?i&~:(member$ PROD_RING ?i)))
  (ring (color ?ring) (req-bases ?br))
  (ring-station (name ?rs) (bases-loaded ?bl&:(>= ?bl ?br)))
  =>
  (retract ?state)
  (assert (state WAIT-FOR-RS-LOCK)
          (needed-task-lock (task-id ?task-id) (action PROD_RING) (place ?rs))
  )
)
(defrule step-wait-for-rs-finish
  "Finish after also getting the lock."
  (declare (salience ?*PRIORITY-STEP-FINISH*))
  (phase PRODUCTION)
  (task (state running) (id ?task-id) (robot ?r&:(eq ?r ?*ROBOT-NAME*)))
  ?step <- (step (name wait-for-rs))
  ?state <- (state WAIT-FOR-RS-LOCK)
  (needed-task-lock (task-id ?task-id) (action PROD_RING) (place ?rs) (resource ?res))
  (lock (type ACCEPT) (agent ?rn&:(eq ?rn ?*ROBOT-NAME*)) (resource ?res))
  (time $?now)
  ;ensure that no two skills are called directly after each other
  ?timer <- (timer (name wait-for-rs-dont-call-two-skills) (time $?t&:(timeout ?now ?t 1.0)))
  =>
  (retract ?state ?timer)
  (assert (state STEP-FINISHED))
  (synced-modify ?step state finished)
)
(defrule step-wait-for-rs-lock-refused
  "When the lock is refused someone else is using the rs. Go back to
the waiting state until we can use it again."
  (declare (salience ?*PRIORITY-STEP-FINISH*))
  (phase PRODUCTION)
  (task (state running) (id ?task-id) (robot ?r&:(eq ?r ?*ROBOT-NAME*)))
  ?step <- (step (name wait-for-rs))
  ?state <- (state WAIT-FOR-RS-LOCK)
  ?ntl <- (needed-task-lock (task-id ?task-id) (action PROD_RING) (place ?rs) (resource ?res))
  (lock (type REFUSE) (agent ?rn&:(eq ?rn ?*ROBOT-NAME*)) (resource ?res))
  =>
  (retract ?state ?ntl)
  (assert (state WAIT-FOR-RS)
          (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?res)))
)

(defrule step-wait-for-output-start
  (declare (salience ?*PRIORITY-STEP-START*))
  (phase PRODUCTION)
  (task (state running) (robot ?r&:(eq ?r ?*ROBOT-NAME*)) (current-step ?step-id))
  ?step <- (step (name wait-for-output) (state wait-for-activation) (machine ?mps) (id ?step-id))
  ?state <- (state STEP-STARTED)
  =>
  (retract ?state)
  (synced-modify ?step state running)
  (assert (state WAIT_FOR_OUTPUT))
)

(defrule step-wait-for-output-finish
  (declare (salience ?*PRIORITY-STEP-START*))
  (phase PRODUCTION)
  (task (state running) (robot ?r&:(eq ?r ?*ROBOT-NAME*)) (current-step ?step-id))
  ?step <- (step (name wait-for-output) (state running) (machine ?mps) (id ?step-id))
  ?state <- (state WAIT_FOR_OUTPUT)
  (machine (name ?mps) (state READY-AT-OUTPUT))
  =>
  (retract ?state)
  (synced-modify ?step state finished)
  (assert (state STEP-FINISHED))
)

(defrule step-acquire-lock-start
  (declare (salience ?*PRIORITY-STEP-START*))
  (phase PRODUCTION)
  ?step <- (step (name acquire-lock) (state wait-for-activation)
                 (lock ?lock) (task-priority ?p))
  ?state <- (state STEP-STARTED)
  =>
  (retract ?state)
  (assert (lock (type GET) (agent ?*ROBOT-NAME*) (resource ?lock) (priority ?p)))
  ; Retract all lock releases for ?res that gets the lock
  (do-for-all-facts ((?release lock)) (and (eq ?release:agent ?*ROBOT-NAME*)
                                           (eq ?release:resource ?lock)
                                           (eq ?release:type RELEASE))
    (retract ?release)
  )
  (synced-modify ?step state running)
  (assert (state WAIT-FOR-STEP-LOCK))
)

(defrule step-acquire-lock-finish
  (declare (salience ?*PRIORITY-STEP-FINISH*))
  (phase PRODUCTION)
  ?step <- (step (name acquire-lock) (state running)
                 (lock ?lock) (task-priority ?p))
  ?state <- (state WAIT-FOR-STEP-LOCK)
  (lock (type ACCEPT) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?lock))
  =>
  (retract ?state)
  (synced-modify ?step state finished)
  (assert (state STEP-FINISHED))
)

(defrule step-acquire-lock-release
  (declare (salience ?*PRIORITY-STEP-START*))
  (phase PRODUCTION)
  ?step <- (step (name release-lock) (state wait-for-activation)
                 (lock ?lock) (task-priority ?p))
  ?state <- (state STEP-STARTED)
  ?l <- (lock (type ACCEPT) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?lock))
  =>
  (retract ?state ?l)
  (synced-modify ?step state finished)
  (assert (state STEP-FINISHED)
          (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?lock)))
)
(defrule step-get-base-finish-fail
  (declare (salience ?*PRIORITY-STEP-FINISH*))
  (phase PRODUCTION)
  (task (state running) (robot ?r&:(eq ?r ?*ROBOT-NAME*)) (current-step ?step-id))
  ?step <- (step (name get-base) (state running) (side ?side) (id ?step-id))
  ?state <- (state ?result&SKILL-FINAL|SKILL-FAILED)
  ?ste <- (skill-to-execute (skill get_product_from)
                            (args $?args) (state final|failed))
  ?l <- (lock (type ACCEPT) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource PREPARE-BS))  
  (machine (name ?mps) (team ?team-color))
  ?bsf <- (base-station (name ?mps))
  =>
  (retract ?state ?ste ?l)
  ;release lock of instructing/using the BS. Can't be done in a later step because the task is aborted when the step fails.
  (if (eq ?result SKILL-FINAL)
    then
    (assert (state STEP-FINISHED))
    (synced-modify ?step state finished)
    (synced-modify ?bsf fail-side NONE)
    else
    (assert (state STEP-FAILED))
    (synced-modify ?step state failed)
    (synced-modify ?bsf fail-side ?side)
  )
  (assert (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource PREPARE-BS)))
)

;;;;;;;;;;;;;;;;
; common finish:
;;;;;;;;;;;;;;;;
(defrule step-common-finish
  (declare (salience ?*PRIORITY-STEP-FINISH*))
  (phase PRODUCTION)
  (task (state running) (robot ?r&:(eq ?r ?*ROBOT-NAME*)) (current-step ?step-id))
  ?step <- (step (name get-from-shelf|insert|get-output|drive-to) (state running) (id ?step-id))
  ?state <- (state SKILL-FINAL)
  ?ste <- (skill-to-execute (skill get_product_from|bring_product_to|ax12gripper|drive_to)
			    (args $?args) (state final))
  ; TODO add new skills with |skill
  =>
  (retract ?state ?ste)
  (assert (state STEP-FINISHED))
  (synced-modify ?step state finished)
)


;;;;;;;;;;;;;;;;
; common fail:
;;;;;;;;;;;;;;;;
(defrule step-common-fail
  (declare (salience ?*PRIORITY-STEP-FAILED*))
  (phase PRODUCTION)
  (task (state running) (robot ?r&:(eq ?r ?*ROBOT-NAME*)) (current-step ?step-id))
  ?step <- (step (name get-from-shelf|insert|get-output|discard|drive-to) (state running) (id ?step-id))
  ?state <- (state SKILL-FAILED)
  ?ste <- (skill-to-execute (skill get_product_from|bring_product_to|ax12gripper|drive_to)
			    (args $?args) (state failed))
  ; TODO add new skills with |skill
  =>
  (retract ?state ?ste)
  (assert (state STEP-FAILED))
  (synced-modify ?step state failed)
)

;;;;;;;;;;;;;;;;;
; other stuff
;;;;;;;;;;;;;;;;;
(defrule step-set-wait-for-lock-position
  "Set the position in a requested lock to be able to find a wait point near it"
  (skill-to-execute (skill get_product_from) (args $?args) (target ?mps))
  ?wfl <- (wait-for-lock (priority ?p) (res ?mps))
  =>
   input or output side?
  (bind ?navpoint (sym-cat ?navpoint "-O"))
  (if (or (member$ input ?args) (member$ shelf ?args)) then
    (bind ?navpoint (sym-cat ?navpoint "-I"))
  )
  (modify ?wfl (place ?navpoint))
)
