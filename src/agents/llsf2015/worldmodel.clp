;---------------------------------------------------------------------------
;  worldmodel.clp - Robotino agent -- world model update rules
;
;  Created: Sat Jun 16 18:50:53 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; knowledge evaluation request
(defrule wm-recv-MachineInfo
  (protobuf-msg (type "llsf_msgs.MachineInfo") (ptr ?p))
  =>
  ;(printout t "***** Received MachineInfo *****" crlf)
  (foreach ?m (pb-field-list ?p "machines")
    (bind ?m-name (sym-cat (pb-field-value ?m "name")))
    (bind ?m-type (sym-cat (pb-field-value ?m "type")))
    (bind ?m-team (sym-cat (pb-field-value ?m "team_color")))
    (do-for-fact ((?machine machine))
      (and (eq ?machine:name ?m-name)
	   (or (neq ?machine:mtype ?m-type) (neq ?machine:team ?m-team)))

      (printout t "Machine " ?m-name " (" ?m-team ") is of type " ?m-type crlf)
      (printout warn "TODO: set available-colors for ring stations " ?m-type crlf)
      (modify ?machine (mtype ?m-type) (team ?m-team))
    )
  )
  (assert (received-machine-info))
)

(defrule wm-get-cap-from-shelf-final
  (declare (salience ?*PRIORITY-WM*))
  (state SKILL-FINAL)
  (skill-to-execute (skill get_product_from) (state final) (target ?mps))
  (step (name get-from-shelf) (state running))
  (cap-station (name ?mps) (assigned-cap-color ?color))
  ?hf <- (holding NONE)
  =>
  (retract ?hf)
  (printout t "Got a Puck from an CS shelf with a " ?color " cap to fill the CS" crlf)
  (bind ?puck-id (random-id))
  (assert (holding ?puck-id)
	  (worldmodel-change (puck-id ?puck-id) (change NEW_PUCK))
          (product (id ?puck-id) (cap ?color))
  )
)

(defrule wm-get-from-shelf-failed
  (declare (salience ?*PRIORITY-WM*))
  (state SKILL-FAILED)
  (step (name get-from-shelf) (state running))
  (skill-to-execute (skill get_product_from) (state failed) (target ?mps))
  (holding NONE)
  =>
  (printout warn "Could not get puck from shelf" crlf)
)

(defrule wm-insert-cap-into-cs-final
  (declare (salience ?*PRIORITY-WM*))
  (state SKILL-FINAL)
  (skill-to-execute (skill bring_product_to) (state final) (target ?mps))
  (step (name insert) (state running))
  ?mf <- (machine (name ?mps) (loaded-id 0) (produced-id 0))
  ?csf <- (cap-station (name ?mps))
  ?hf <- (holding ?puck-id)
  ?pf <- (product (id ?puck-id) (cap ?cap))
  =>
  (retract ?hf)
  (printout warn "TODO: use worldmodel change messages" crlf)
  (printout t "Inserted a Puck from an CS shelf with a " ?cap " cap to fill the CS" crlf)
  (assert (holding NONE))
  ; there is no relevant waiting time until the cs has finished the loading step right?
  (assert (worldmodel-change (machine ?mps) (change SET_PRODUCED) (amount ?puck-id))
	  (worldmodel-change (machine ?mps) (change SET_CAP_LOADED) (value ?cap))
	  (worldmodel-change (puck-id ?puck-id) (change SET_CAP) (value NONE))
  )
)

(defrule wm-insert-cap-failed
  (declare (salience ?*PRIORITY-WM*))
  (state SKILL-FAILED)
  (skill-to-execute (skill bring_product_to) (state failed) (target ?mps))
  (step (name insert) (state running))
  ?hf <- (holding ?puck-id)
  ?pf <- (product (id ?puck-id))
  =>
  (retract ?hf ?pf)
  (printout t "Inserted a Puck into the MPS " ?mps " failed" crlf)
  (printout error "TODO: check if we still have apuck or not" crlf)
  (assert (holding NONE))
)

(defrule wm-get-output-final
  (declare (salience ?*PRIORITY-WM*))
  (state SKILL-FINAL)
  (skill-to-execute (skill get_product_from) (state final) (target ?mps))
  (step (name get-output) (state running))
  ?mf <- (machine (name ?mps) (produced-id ?puck-id))
  ?hf <- (holding NONE)
  =>
  (retract ?hf)
  (printout t "Fetched a Puck from the output of " ?mps crlf)
  (assert (holding ?puck-id)
	  (worldmodel-change (machine ?mps) (change SET_PRODUCED) (amount 0)))
)

(defrule wm-get-output-failed
  (declare (salience ?*PRIORITY-WM*))
  (state SKILL-FAILED)
  (skill-to-execute (skill ppgoto) (state failed) (target ?mps))
  (step (name get-output) (state running))
  ?mf <- (machine (name ?mps) (produced-id ?puck-id))
  =>
  (printout warn "TODO: use right skill in worldmodel for get-output" crlf)
  (printout t "Failed to fetch a Puck from the output of " ?mps crlf)
  (printout t "I assume there is no more output puck at " ?mps crlf)
  (assert (worldmodel-change (machine ?mps) (change SET_PRODUCED) (amount 0)))
)

(defrule wm-store-lights
  (declare (salience ?*PRIORITY-WM*))
  ?rf <- (RobotinoLightInterface (id "Light determined") (ready TRUE)
				 (red ?red) (green ?green) (yellow ?yellow))
  =>
  (retract ?rf)
  ; remove all possibly existing last-lights facts
  (delayed-do-for-all-facts ((?ll last-lights)) TRUE (retract ?ll))
  (bind ?lights (create$ (sym-cat GREEN- ?green) (sym-cat YELLOW- ?yellow) (sym-cat RED- ?red) )
  )
  (assert (last-lights ?lights))
  (printout t "***** Lights 5 " ?lights crlf)
)

(defrule wm-goto-deliver-failed
  (declare (salience ?*PRIORITY-WM*))
  (state SKILL-FAILED)
  (skill-to-execute (skill deliver) (state failed))
  ?hf <- (holding ~NONE)
  (puck-in-gripper ?puck)
  =>
  (if (not ?puck) then
    (retract ?hf)
    (assert (holding NONE))
  )
  (printout error "Delivery failed. Try again if I have a puck." crlf) 
)

(defrule wm-goto-failed
  (declare (salience ?*PRIORITY-WM-LOW*))
  (state SKILL-FAILED)
  (skill-to-execute (skill finish_puck_at) (state failed) (target ?name))
  ?gtdw <- (dont-wait ?dont-wait)
  ?hf <- (holding ?)
  (puck-in-gripper ?puck)
  =>
  (retract ?gtdw)
  (if (not ?puck) then
    (retract ?hf)
    (assert (holding NONE))
  )
  (printout error "Production failed at " ?name crlf) 
)

(defrule wm-take-puck-to-failed
  (declare (salience ?*PRIORITY-WM-LOW*))
  (state SKILL-FAILED)
  (skill-to-execute (skill take_puck_to) (state failed) (target ?name))
  ?hf <- (holding ~NONE)
  (puck-in-gripper ?puck)
  =>
  (if (not ?puck) then
    (retract ?hf)
    (assert (holding NONE))
    (printout error "Lost puck during take_puck_to" crlf)
  )
)

(defrule wm-drive-to-failed
  (declare (salience ?*PRIORITY-WM-LOW*))
  (state SKILL-FAILED)
  (skill-to-execute (skill drive_to) (state failed) (target ?name))
  ?hf <- (holding ~NONE)
  (puck-in-gripper ?puck)
  =>
  (if (not ?puck) then
    (retract ?hf)
    (assert (holding NONE))
    (printout error "Lost puck during drive_to" crlf)
  )
)

(defrule wm-goto-light
  (declare (salience ?*PRIORITY-WM*))
  (state SKILL-FINAL)
  (skill-to-execute (skill finish_puck_at) (state final))
  (not (lights))
  ?lf <- (last-lights $?lights&:(> (length$ ?lights) 0))
  =>
  (retract ?lf)
  (assert (lights ?lights))
)

; (defrule wm-proc-complete-without-robot
;   (declare (salience ?*PRIORITY-WM*))
;   (time $?now)
;   ?mf <- (machine (name ?name) (mtype ?mtype) (output ?output) (loaded-with $?lw) (junk ?jn)
;            (final-prod-time $?fpt&:(and (timeout ?now ?fpt 0.5) (neq (nth$ 1 ?fpt) 0)))
; 	   (produced-puck NONE)
;          )
;   =>
;   (printout t "Production completed at " ?name "|" ?mtype crlf)
;   (modify ?mf (final-prod-time (create$ 0 0)) (produced-puck ?output) (loaded-with (create$)) (junk (- (+ ?jn (length$ ?lw)) 1)))
; )

; (defrule wm-out-of-order-proc-started
;   "machine out of order and we left the last puck there, so the machine starts producing when getting active again"
;   (declare (salience ?*PRIORITY-WM*))
;   (state SKILL-FINAL)
;   (skill-to-execute (skill finish_puck_at) (state final) (target ?name))
;   ?gtdw <- (dont-wait true)
;   ?hf <- (holding ?was-holding)
;   ?lf <- (lights GREEN-OFF YELLOW-OFF RED-ON)
;   ?mf <- (machine (name ?name) (mtype ?mtype) (loaded-with $?lw))
;   (time $?now)
;   (production-time ?mtype ?min-prod-time ?)
;   (out-of-order-time max ?ooo-max)
;   (out-of-order-time recycle-max ?ooo-recycle-max)
;   =>
;   (retract ?hf ?lf ?gtdw)
;   (assert (holding NONE))
;   (printout t "Machine Out Of Order; Production starts afterwards at " ?name "|" ?mtype crlf)
;   (if (eq ?mtype RECYCLE)
;     then
;     (bind ?ooo-time ?ooo-recycle-max)
;     else
;     (bind ?ooo-time ?ooo-max)
;   )
;   (assert (worldmodel-change (machine ?name) (change ADD_LOADED_WITH) (value ?was-holding))
; 	  (worldmodel-change (machine ?name) (change SET_PROD_FINISHED_TIME) (amount (+ (nth$ 1 ?now) ?min-prod-time ?ooo-time)))
; 	  (worldmodel-change (machine ?name) (change SET_OUT_OF_ORDER_UNTIL) (amount (+ (nth$ 1 ?now) ?ooo-time)))
;   )
; )

; (defrule wm-out-of-order-goto-aborted
;   "machine out of order and we aborted loading the machine"
;   (declare (salience ?*PRIORITY-WM*))
;   (state SKILL-FINAL)
;   (skill-to-execute (skill finish_puck_at) (state final) (target ?name))
;   ?gtdw <- (dont-wait false)
;   ?lf <- (lights GREEN-OFF YELLOW-OFF RED-ON)
;   ?mf <- (machine (name ?name) (mtype ?mtype) (loaded-with $?lw))
;   (time $?now)
;   (out-of-order-time max ?ooo-max)
;   (out-of-order-time recycle-max ?ooo-recycle-max)
;   =>
;   (retract ?lf ?gtdw)
;   (printout t "Machine Out Of Order; Loading/Producing this machine aborted at " ?name "|" ?mtype crlf)
;   (if (eq ?mtype RECYCLE)
;     then
;     (bind ?ooo-time ?ooo-recycle-max)
;     else
;     (bind ?ooo-time ?ooo-max)
;   )
;   (assert (worldmodel-change (machine ?name) (change SET_OUT_OF_ORDER_UNTIL) (amount (+ (nth$ 1 ?now) ?ooo-time))))
; )

(defrule wm-proc-delivered
  (declare (salience ?*PRIORITY-WM*))
  (state SKILL-FINAL)
  (skill-to-execute (skill deliver) (state final))
  ?hf <- (holding ?was-holding&~NONE)
  ;?lf <- (lights $?)
  =>
  (retract ?hf)
  (assert (holding NONE)
	  (delivered ?was-holding)
  )
  (printout t "Delivered " ?was-holding crlf)
)

(defrule wm-proc-wtf
  (declare (salience ?*PRIORITY-WM-LOW*))
  ?s <- (state SKILL-FINAL)
  (skill-to-execute (skill finish_puck_at) (state final) (target ?name))
  ?gtdw <- (dont-wait ?dont-wait)
  ?lf <- (lights $?)
  ?hf <- (holding ?)
  ?mf <- (machine (name ?name) (mtype ?mtype))
  (puck-in-gripper ?have-puck)
  =>
  (printout error "WTF? Unhandled light code at " ?name "|" ?mtype crlf) 
  (retract ?lf ?gtdw)
  (if (not ?have-puck)
    then
    (retract ?hf ?s)
    (assert (holding NONE))
  )
  (assert (state SKILL-FAILED))
)

; (defrule wm-get-produced-final
;   (declare (salience ?*PRIORITY-WM*))
;   (state SKILL-FINAL)
;   (skill-to-execute (skill get_produced) (state final) (target ?name))
;   ?hf <- (holding NONE)
;   ?mf <- (machine (name ?name) (output ?output))
;   =>
;   (retract ?hf)
;   (assert (holding ?output))
;   (printout t "Got Produced Puck." crlf)
;   (assert (worldmodel-change (machine ?name) (change REMOVE_PRODUCED)))
; )

; (defrule wm-get-produced-failed
;   (declare (salience ?*PRIORITY-WM*))
;   (state SKILL-FINAL)
;   (skill-to-execute (skill get_produced) (state failed) (target ?name))
;   ?hf <- (holding NONE)
;   ;?mf <- (machine (name ?name) (output ?output))
;   =>
;   (retract ?hf)
;   (assert (holding NONE))
;   (printout error "Got Produced Puck failed." crlf)
;   ;allow one problem. when the second occurs the machine gets blocked
;   (assert (worldmodel-change (machine ?name) (change SET_DOUBTFUL_WORLDMODEL)))
;   (assert (worldmodel-change (machine ?name) (change REMOVE_PRODUCED)))
;   (printout error "remove produced puck in wm!" crlf)
; )

; (defrule wm-store-puck-final
;   (declare (salience ?*PRIORITY-WM*))
;   (state SKILL-FINAL)
;   (skill-to-execute (skill store_puck) (state final) (target ?name))
;   ?hf <- (holding ?puck)
;   ?mf <- (puck-storage (name ?name))
;   =>
;   (retract ?hf)
;   (assert (holding NONE))
;   (printout t "Successfully stored puck." crlf)
;   (assert (worldmodel-change (machine ?name) (change ADD_LOADED_WITH) (value ?puck)))
; )

; (defrule wm-store-puck-failed
;   (declare (salience ?*PRIORITY-WM*))
;   (state SKILL-FAILED)
;   (skill-to-execute (skill store_puck) (state failed) (target ?name))
;   ?hf <- (holding ?puck)
;   ?mf <- (puck-storage (name ?name))
;   (puck-in-gripper ?puck-in-gripper)
;   =>
;   (if (not ?puck-in-gripper) then
;     (retract ?hf)
;     (assert (holding NONE))
;   )
;   (printout error "Store puck failed" crlf)
; )

; (defrule wm-get-stored-puck-final
;   (declare (salience ?*PRIORITY-WM*))
;   (state SKILL-FINAL)
;   (skill-to-execute (skill get_stored_puck) (state final) (target ?name))
;   ?hf <- (holding NONE)
;   ?mf <- (puck-storage (name ?name) (puck ?puck))
;   =>
;   (retract ?hf)
;   (assert (holding ?puck))
;   (printout t "Successfully got stored puck." crlf)
;   (assert (worldmodel-change (machine ?name) (change REMOVE_LOADED_WITH) (value ?puck)))
; )

; (defrule wm-get-stored-puck-failed
;   (declare (salience ?*PRIORITY-WM*))
;   (state SKILL-FAILED)
;   (skill-to-execute (skill get_stored_puck) (state failed) (target ?name))
;   ?hf <- (holding NONE)
;   ?mf <- (puck-storage (name ?name) (puck ?puck))
;   =>
;   (printout error "Failed to get stored puck." crlf)
;   (assert (worldmodel-change (machine ?name) (change REMOVE_LOADED_WITH) (value ?puck)))
; )

(defrule wm-worldmodel-change-set-agent
  "Set the agent field in a new worldmodel change. We know that the change is from this agent because otherwise the field would be set."
  (declare (salience ?*PRIORITY-WM*))
  ?wmc <- (worldmodel-change (agent DEFAULT))
  =>
  (modify ?wmc (agent (sym-cat ?*ROBOT-NAME*)))
)

(defrule wm-process-wm-change-before-sending-machine
  (declare (salience ?*PRIORITY-WM*))
  ?wmc <- (worldmodel-change (machine ?machine) (change ?change) (value ?value) (amount ?amount) (already-applied FALSE) (agent ?agent&~DEFAULT))
  ?m <- (machine (name ?machine) (loaded-id ?loaded-with) (incoming $?incoming) (incoming-agent $?incoming-agent) (produced-id ?produced))
  =>
  (bind ?could-apply-change TRUE)
  (switch ?change
    (case SET_LOADED then 
      (modify ?m (loaded-id ?amount))
    )
    (case REMOVE_LOADED then 
      (modify ?m (loaded-id 0))
    )
    (case ADD_INCOMING then 
      (modify ?m (incoming (append$ ?incoming ?value))
	         (incoming-agent (append$ ?incoming-agent ?agent)))
    )
    (case REMOVE_INCOMING then 
      (modify ?m (incoming (delete-member$ ?incoming ?value))
	         ;every agent should do only one thing at a machine
	         (incoming-agent (delete-member$ ?incoming-agent ?agent)))
    )
    (case SET_PROD_FINISHED_TIME then 
      (modify ?m (final-prod-time (create$ ?amount 0)))
    )
    (case SET_OUT_OF_ORDER_UNTIL then
      (modify ?m (out-of-order-until (create$ ?amount 0)))
    )
    (case SET_PRODUCED then 
      (modify ?m (produced-id ?amount))
    )
    (case REMOVE_PRODUCED then 
      (modify ?m (produced-id 0))
    )
    (default  
      (bind ?could-apply-change FALSE)
    )
  )
  (if ?could-apply-change then
    (modify ?wmc (already-applied TRUE))
  )
)

(defrule wm-process-wm-change-before-sending-product
  (declare (salience ?*PRIORITY-WM*))
  ?wmc <- (worldmodel-change (puck-id ?puck-id) (change ?change) (value ?value) (already-applied FALSE))
  ?p <- (product (id ?puck-id) (rings $?rings))
  =>
  (bind ?could-apply-change TRUE)
  (switch ?change
    (case ADD_RING then 
      (modify ?p (rings (append$ ?rings ?value)))
    )
    (case SET_CAP then 
      (modify ?p (cap ?value))
    )
    (case REMOVE_PUCK then 
      (retract ?p)
    )
    (case SET_BASE then
      (modify ?p (base ?value))
    )
    (default  
      (bind ?could-apply-change FALSE)
    )
  )
  (if ?could-apply-change then
    (modify ?wmc (already-applied TRUE))
  )
)

(defrule wm-process-wm-change-before-sending-cap-station
  (declare (salience ?*PRIORITY-WM*))
  ?wmc <- (worldmodel-change (machine ?mps) (change ?change) (value ?value) (already-applied FALSE))
  ?cs <- (cap-station (name ?mps))
  =>
  (bind ?could-apply-change TRUE)
  (switch ?change
    (case SET_CAP_LOADED then 
      (modify ?cs (cap-loaded ?value))
    )
    (default  
      (bind ?could-apply-change FALSE)
    )
  )
  (if ?could-apply-change then
    (modify ?wmc (already-applied TRUE))
  )
)

(defrule wm-process-wm-change-before-sending-ring-station
  (declare (salience ?*PRIORITY-WM*))
  ?wmc <- (worldmodel-change (machine ?mps) (change ?change) (value ?value) (amount ?amount) (already-applied FALSE))
  ?rs <- (ring-station (name ?mps))
  =>
  (bind ?could-apply-change TRUE)
  (switch ?change
    (case SET_SELECTED_COLOR then 
      (modify ?rs (selected-color ?value))
    )
    (case SET_BASES_NEEDED then 
      (modify ?rs (bases-needed ?amount))
    )
    (default  
      (bind ?could-apply-change FALSE)
    )
  )
  (if ?could-apply-change then
    (modify ?wmc (already-applied TRUE))
  )
)


(defrule wm-process-wm-change-at-order-before-sending
  (declare (salience ?*PRIORITY-WM*))
  ?wmc <- (worldmodel-change (order ?id) (change SET_IN_DELIVERY)
			     (value ?puck) (amount ?amount) (already-applied FALSE))
  ?order <- (order (id ?id))
  =>
  (modify ?order (in-delivery ?amount))
  (modify ?wmc (already-applied TRUE))
)

(defrule wm-process-wm-change-exploration-zone
  "apply wm change to local worldmodel before sending"
  (declare (salience ?*PRIORITY-WM*))
  ?wmc <- (worldmodel-change (machine ?zone) (value ?value) (agent ?agent)
                             (change ?change) (already-applied FALSE))
  ?zone-fact <- (zone-exploration (name ?zone) (times-searched ?times-searched)
                                  (incoming $?incoming) (incoming-agent $?incoming-agent))
  =>  
  (switch ?change
    (case ZONE_STILL_TO_EXPLORE then 
      (modify ?zone-fact (still-to-explore ?value))
    )
    (case ZONE_MACHINE_IDENTIFIED then 
      (modify ?zone-fact (machine ?value))
    )
    (case ZONE_TIMES_SEARCHED_INCREMENT then 
      (modify ?zone-fact (times-searched (+ 1 ?times-searched)))
    )
    (case ADD_INCOMING then 
      (modify ?zone-fact (incoming (append$ ?incoming ?value))
              (incoming-agent (append$ ?incoming-agent ?agent)))
    )
    (case REMOVE_INCOMING then 
      (modify ?zone-fact (incoming (delete-member$ ?incoming ?value))
	         ;every agent should do only one thing at a machine
	         (incoming-agent (delete-member$ ?incoming-agent ?agent)))
    )
    (default
      (printout error "Worldmodel-Change Type " ?change
                " is not handled for zonex. Worlmodel is probably wrong." crlf)
    )
  )
  (modify ?wmc (already-applied TRUE))
)

(defrule wm-process-wm-change-before-sending-puck-storage
  (declare (salience ?*PRIORITY-WM*))
  ?wmc <- (worldmodel-change (machine ?storage) (change ?change) (value ?value) (already-applied FALSE) (agent ?agent))
  ?ps <- (puck-storage (name ?storage) (incoming $?incoming) (incoming-agent $?incoming-agent))
  =>
  (bind ?could-apply-change TRUE)
  (switch ?change
    ; (case ADD_LOADED_WITH then 
    ;   (modify ?ps (puck ?value))
    ; )
    ; (case REMOVE_LOADED_WITH then
    ;   (modify ?ps (puck NONE))
    ; )
    (case ADD_INCOMING then 
      (modify ?ps (incoming (append$ ?incoming ?value))
	          (incoming-agent (append$ ?incoming-agent ?agent)))
    )
    (case REMOVE_INCOMING then 
      (modify ?ps (incoming (delete-member$ ?incoming ?value))
	         ;every agent should do only one thing at a machine
	         (incoming-agent (delete-member$ ?incoming-agent ?agent)))
    )
    (default  
      (bind ?could-apply-change FALSE)
    )
  )
  (if ?could-apply-change then
    (modify ?wmc (already-applied TRUE))
  )
)

(defrule wm-process-wm-change-before-sending-zone
  (declare (salience ?*PRIORITY-WM*))
  ?wmc <- (worldmodel-change (machine ?zone) (change ?change) (value ?value) (amount ?amount) (already-applied FALSE) (agent ?agent&~DEFAULT))
  ?zone-fact <- (zone-exploration (name ?zone) (incoming $?incoming) (incoming-agent $?incoming-agent))
  =>
  (bind ?could-apply-change TRUE)
  (switch ?change
    (case ADD_INCOMING then 
      (modify ?zone-fact (incoming (append$ ?incoming ?value))
              (incoming-agent (append$ ?incoming-agent ?agent)))
    )
    (case REMOVE_INCOMING then 
      (modify ?zone-fact (incoming (delete-member$ ?incoming ?value))
	      ;every agent should do only one thing at a machine
              (incoming-agent (delete-member$ ?incoming-agent ?agent)))
    )
    (default  
      (bind ?could-apply-change FALSE)
    )
  )
  (if ?could-apply-change then
    (modify ?wmc (already-applied TRUE))
  )
)

(defrule wm-update-pose
  (declare (salience ?*PRIORITY-CLEANUP*))
  ?pif <- (Position3DInterface (id "Pose") (translation $?pos))
  ?pose <- (pose (x ?) (y ?))
  =>
  (modify ?pose (x (nth$ 1 ?pos)) (y (nth$ 2 ?pos)))
  (retract ?pif)
)

; (defrule wm-update-puck-in-gripper
;   (declare (salience ?*PRIORITY-CLEANUP*))
;   ?rif <- (RobotinoSensorInterface (id "Robotino") (distance $?distances))
;   ?pig <- (puck-in-gripper ?puck)
;   (confval (path "/hardware/robotino/puck_sensor/index") (value ?index))
;   (confval (path "/hardware/robotino/puck_sensor/trigger_dist") (value ?trigger))
;   =>
;   (bind ?dist (nth$ (+ ?index 1) ?distances))
;   (if (and (< ?dist ?trigger) (not ?puck))
;     then
;     ;(printout t "Have puck in gripper" crlf)
;     (retract ?pig)
;     (assert (puck-in-gripper TRUE))
    
;     else
    
;     (if (and (> ?dist ?trigger) ?puck)
; 	then
;       ;(printout t "Have no puck in gripper" crlf)
;       (retract ?pig)
;       (assert (puck-in-gripper FALSE))
;     )
;   )
;   (retract ?rif)
; )

(deffunction wm-remove-incoming-by-agent (?agent)
  "remove all entries of machine-incoming fields of a specific agent (e.g. after a lost connection)"
  (delayed-do-for-all-facts ((?m machine)) (member$ (sym-cat ?agent) ?m:incoming-agent)
    (bind ?new-incoming ?m:incoming)
    (bind ?new-incoming-agent ?m:incoming-agent)
    (while (member$ (sym-cat ?agent) ?new-incoming-agent)
      (bind ?index (member$ (sym-cat ?agent) ?new-incoming-agent))
      (bind ?new-incoming (delete$ ?new-incoming ?index ?index))
      (bind ?new-incoming-agent (delete$ ?new-incoming-agent ?index ?index))
    )
    (modify ?m (incoming ?new-incoming) (incoming-agent ?new-incoming-agent))
  ) 
)

(defrule wm-remove-out-of-order
  (time $?now)
  ?m <- (machine (out-of-order-until $?ooo&:(and (neq (nth$ 1 ?ooo) 0)
						 (timeout ?now ?ooo 0.0))))
  =>
  (modify ?m (out-of-order-until (create$ 0 0)))
)
