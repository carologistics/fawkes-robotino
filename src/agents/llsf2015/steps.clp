;---------------------------------------------------------------------------
;  steps.clp - rules how to execute the concrete steps of tasks
;
;  Created: Sat Feb 07 19:43:19 2015
;  Copyright  2015 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defrule step-get-from-shelf
  (declare (salience ?*PRIORITY-STEP-START*))
  (phase PRODUCTION)
  ?step <- (step (name get-from-shelf) (state wait-for-activation) (task-priority ?p)
		 (machine ?mps) (shelf-slot ?sslot))
  ?state <- (state STEP-STARTED)
  (team-color ?team)
  (game-time $?game-time)
  =>
  (retract ?state)
  (modify ?step (state running))
  (printout warn "TODO: Pick dynamically from different shelf positions." crlf)
  (assert (state WAIT-FOR-LOCK)
	  (skill-to-execute (skill get_product_from) (args place ?mps shelf LEFT) (target ?mps))
	  (wait-for-lock (priority ?p) (res ?mps))
  )
)

(defrule step-insert-start
  (declare (salience ?*PRIORITY-STEP-START*))
  (phase PRODUCTION)
  ?step <- (step (name insert) (state wait-for-activation) (task-priority ?p)
		 (machine ?mps) (machine-feature ?feature))
  ?state <- (state STEP-STARTED)
  (team-color ?team)
  (game-time $?game-time)
  =>
  (retract ?state)
  (modify ?step (state running))
  (assert (state WAIT-FOR-LOCK)
	  (skill-to-execute (skill bring_product_to) (args place ?mps)  (target ?mps))
	  (wait-for-lock (priority ?p) (res ?mps))
  )
)

(defrule step-get-output-start
  (declare (salience ?*PRIORITY-STEP-START*))
  (phase PRODUCTION)
  ?step <- (step (name get-output) (state wait-for-activation) (task-priority ?p)
		 (machine ?mps))
  ?state <- (state STEP-STARTED)
  (team-color ?team)
  (game-time $?game-time)
  =>
  (retract ?state)
  (modify ?step (state running))
  (assert (state WAIT-FOR-LOCK)
	  (skill-to-execute (skill get_product_from) (args place ?mps ) (target ?mps))
	  (wait-for-lock (priority ?p) (res ?mps))
  )
)

(defrule step-get-base-start
  (declare (salience ?*PRIORITY-STEP-START*))
  (phase PRODUCTION)
  ?step <- (step (name get-base) (state wait-for-activation) (task-priority ?p)
		 (machine ?mps) (machine-feature ?feature) (base ?color))
  ?state <- (state STEP-STARTED)
  (team-color ?team)
  =>
  (retract ?state)
  (modify ?step (state running))
  (printout warn "TODO: Pick bases from both BS sides" crlf)
  (assert (state WAIT-FOR-LOCK)
	  (skill-to-execute (skill get_product_from) (args place ?mps) (target ?mps))
	  (wait-for-lock (priority ?p) (res ?mps))
  )
)

(defrule step-find-tag-start
  (declare (salience ?*PRIORITY-STEP-START*))
  (phase PRODUCTION)
  ?step <- (step (name find-tag) (state wait-for-activation) (task-priority ?p)
		 (zone ?zone) (machine ?missing-mps))
  ?state <- (state STEP-STARTED)
  (game-time $?game-time)
  (team-color ?team)
  =>
  (retract ?state)
  (modify ?step (state running))
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
          (worldmodel-change (machine ?zone) (change ZONE_TIMES_SEARCHED_INCREMENT))
  )
)
(defrule step-find-tag-finish
  "tag-find skill finished, try to find tag"
  (declare (salience ?*PRIORITY-STEP-FINISH*))
  (phase PRODUCTION)
  (step (name find-tag) (state running))
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
  ?step <- (step (name find-tag) (state running) (zone ?zone))
  ?ws <- (timer (name waiting-for-tag-since))
  ?s <- (state PROD_LOOKING_FOR_TAG)
  ?zone-fact <- (zone-exploration (name ?zone))
  (confval (path "/clips-agent/llsf2015/exploration/needed-visibility-history") (value ?needed-vh))
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
      (modify ?step (state failed))
      (return)
    )
  )
  (if (eq 7 (length$ ?tf-transrot)) then
 
    (assert (found-tag (name ?machine) (side ?side) (frame "/map")
                       (trans (subseq$ ?tf-transrot 1 3))
                       (rot (subseq$ ?tf-transrot 4 7))))
    else
    (printout error "Can not transform " ?frame " to /map. Transform is empty. Tags positions are broken!!!" crlf)
    (printout error "Check time diff between base and laptop" crlf)
    (retract ?s ?ws ?skill-finish-state)
    (assert (state STEP-FAILED))
    (modify ?step (state failed))
    (return)
  )
  (retract ?s ?ws ?skill-finish-state)
  (assert (worldmodel-change (machine ?machine) (change ADD_TAG))
          (state STEP-FINISHED))
  (modify ?step (state finished))

  (navgraph-add-all-new-tags)
)
(defrule step-find-tag-fail
  "Finish step if there is no tag in this zone."
  (phase PRODUCTION)
  ?step <- (step (name find-tag) (state running) (zone ?zone) (machine ?machine))
  (time $?now)
  ?ws <- (timer (name waiting-for-tag-since) (time $?t&:(timeout ?now ?t 3.0)))
  ?s <- (state PROD_LOOKING_FOR_TAG)
  ?zone-fact <- (zone-exploration (name ?zone))
  ?skill-finish-state <- (explore-zone-state ?explore-zone-state)
  =>
  (printout t "No tag found in " ?zone crlf)
  (if (eq ?explore-zone-state final) then
    (printout t "There probably is no mps in this zone!" crlf)
    (assert (worldmodel-change (machine ?zone) (change ZONE_STILL_TO_EXPLORE)
                               (value FALSE)))
  )
  (retract ?s ?ws ?skill-finish-state)
  (assert (state STEP-FAILED))
  (modify ?step (state failed))
)
(defrule step-find-tag-stop-when-other-bot-found-the-machine
  "tag-find can be stopped, because the machine we want to find
   was already found by another bot"
  (declare (salience ?*PRIORITY-STEP-FINISH*))
  (phase PRODUCTION)
  ?step <- (step (name find-tag) (state running) (machine ?machine))
  ?state <- (state SKILL-EXECUTION)
  ?ste <- (skill-to-execute (skill drive_to)
			    (args $?args) (state running))
  (time $?now)
  (found-tag (name ?machine))
  =>
  (retract ?state ?ste)
  (assert (state STEP-FINISHED))
  (modify ?step (state finished))
  ; skill is not properly stopped, but it should work anyway
  ; because the next skill call overrides it
)

(defrule step-get-product-start
  (declare (salience ?*PRIORITY-STEP-START*))
  (phase PRODUCTION)
  ?step <- (step (name get-product) (state wait-for-activation) (task-priority ?p)
		 (machine ?mps))
  ?state <- (state STEP-STARTED)
  (team-color ?team)
  =>
  (retract ?state)
  (modify ?step (state running))
  (printout warn "TODO: use skill to get a puck from an MPS" crlf)
  (assert (state WAIT-FOR-LOCK)
	  (skill-to-execute (skill ppgoto) (args place (get-output ?mps)) (target ?mps))
	  (wait-for-lock (priority ?p) (res ?mps))
  )
)

; (defrule step-get-S0-start
;   (declare (salience ?*PRIORITY-STEP-START*))
;   (phase PRODUCTION)
;   ?step <- (step (name get-s0) (state wait-for-activation) (task-priority ?p))
;   ?state <- (state STEP-STARTED)
;   (team-color ?team)
;   (input-storage ?team ?ins ? ? )
;   (secondary-storage ?team ?inssec ? ?)
;   (game-time $?game-time)
;   =>
;   (retract ?state)
;   (assert (state WAIT-FOR-LOCK))
;   (modify ?step (state running))
;   ; (if (tac-check-for-secondary-ins ?ins ?inssec ?game-time)
;   ;   then
;   ;   (assert (skill-to-execute (skill get_s0) (args place ?inssec) (target ?inssec))
;   ; 	    (wait-for-lock (priority ?p) (res ?inssec))
;   ;   )
;   ;   else
;   ;   (assert (skill-to-execute (skill get_s0) (args place ?ins) (target ?ins))
;   ; 	    (wait-for-lock (priority ?p) (res ?ins))
;   ;   )
;   ; )
; )

; (defrule step-produce-at-start
;   (declare (salience ?*PRIORITY-STEP-START*))
;   (phase PRODUCTION)
;   ?step <- (step (name produce-at) (state wait-for-activation) (machine ?machine))
;   ?state <- (state STEP-STARTED)
;   (holding ~NONE)
;   (machine (name ?machine) (mtype ?mtype))
;   =>
;   (retract ?state)
;   (assert (state WAIT-FOR-LOCK)
; 	  (skill-to-execute (skill finish_puck_at) (args place ?machine) (target ?machine))
; 	  (dont-wait false)
; 	  (wait-for-lock (res ?machine))
;   )
;   (modify ?step (state running))
; )

; (defrule step-deliver-start
;   (declare (salience ?*PRIORITY-STEP-START*))
;   (phase PRODUCTION)
;   ?step <- (step (name deliver) (state wait-for-activation) (task-priority ?p))
;   ?state <- (state STEP-STARTED)
;   (holding P1|P2|P3)
;   (team-color ?team)
;   (deliver ?team ?deliver ? ?)
;   =>
;   (retract ?state)
;   (assert (state WAIT-FOR-LOCK)
; 	  (skill-to-execute (skill deliver) (args place ?deliver) (target ?deliver))
; 	  (wait-for-lock (priority ?p) (res ?deliver))
;   )
;   (modify ?step (state running))
; )

;;;;;;;;;;;;;;;;
; common finish:
;;;;;;;;;;;;;;;;
(defrule step-common-finish
  (declare (salience ?*PRIORITY-STEP-FINISH*))
  (phase PRODUCTION)
  ?step <- (step (name get-from-shelf|insert|get-output|get-base) (state running))
  ?state <- (state SKILL-FINAL)
  ?ste <- (skill-to-execute (skill ppgoto)
			    (args $?args) (state final))
  ; TODO add new skills with |skill
  =>
  (retract ?state ?ste)
  (assert (state STEP-FINISHED))
  (modify ?step (state finished))
)

;;;;;;;;;;;;;;;;
; common fail:
;;;;;;;;;;;;;;;;
(defrule step-common-fail
  (declare (salience ?*PRIORITY-STEP-FAILED*))
  (phase PRODUCTION)
  ?step <- (step (name get-from-shelf|insert|get-output) (state running))
  ?state <- (state SKILL-FAILED)
  ?ste <- (skill-to-execute (skill ppgoto)
			    (args $?args) (state failed))
  ; TODO add new skills with |skill
  =>
  (retract ?state ?ste)
  (assert (state STEP-FAILED))
  (modify ?step (state failed))
)
