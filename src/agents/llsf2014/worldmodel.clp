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
      (modify ?machine (mtype ?m-type) (team ?m-team)
	      (output (sym-cat (pb-field-value ?m "output"))))
    )
  )
  (assert (received-machine-info))
)

(defrule wm-get-s0-final
  (declare (salience ?*PRIORITY-WM*))
  (state GET-S0-FINAL)
  ?hf <- (holding NONE)
  =>
  (retract ?hf)
  (assert (holding S0))
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
  (state GOTO-FAILED)
  ?tf <- (goto-target deliver1|deliver2)
  ?hf <- (holding ?)
  (puck-in-gripper ?puck)
  =>
  (retract ?tf)
  (if (not ?puck) then
    (retract ?hf)
    (assert (holding NONE))
  )
  (printout error "Delivery failed. Try again if I have a puck." crlf) 
)

(defrule wm-goto-failed
  (declare (salience ?*PRIORITY-WM-LOW*))
  (state GOTO-FAILED)
  ?tf <- (goto-target ?name)
  ?gtdw <- (goto-dont-wait ?dont-wait)
  ?hf <- (holding ?)
  (puck-in-gripper ?puck)
  =>
  (retract ?tf ?gtdw)
  (if (not ?puck) then
    (retract ?hf)
    (assert (holding NONE))
  )
  (printout error "Production failed at " ?name crlf) 
)

(defrule wm-goto-light
  (declare (salience ?*PRIORITY-WM*))
  (state GOTO-FINAL)
  (not (lights))
  ?lf <- (last-lights $?lights&:(> (length$ ?lights) 0))
  =>
  (retract ?lf)
  (assert (lights ?lights))
)

(defrule wm-goto-proc-complete
  (declare (salience ?*PRIORITY-WM*))
  (state GOTO-FINAL)
  ?tf <- (goto-target ?name)
  ?gtdw <- (goto-dont-wait ?dont-wait)
  ?hf <- (holding ?holding-old)
  ?lf <- (lights GREEN-ON YELLOW-OFF RED-OFF)
  ?mf <- (machine (name ?name) (mtype ?mtype) (output ?output) (loaded-with $?lw) (junk ?jn))
  =>
  (retract ?tf ?hf ?lf ?gtdw)
  (if (and (eq ?dont-wait true)
	   (or (eq ?mtype T2)
	       (eq ?mtype T3)
	       (eq ?mtype T4)
	       (eq ?mtype T5)))
    then
    (printout error "Production finished directly althoug we didn't want to wait so long" crlf)
    (assert (holding ?holding-old))
    (return)
  )
  (assert (holding ?output))
  (printout t "Production completed at " ?name "|" ?mtype crlf)
  ;TODO worldmodel change sync
  (foreach ?puck ?lw
    (assert (worldmodel-change (machine ?name) (change REMOVE_LOADED_WITH) (value ?puck)))
  )
  (assert (worldmodel-change (machine ?name) (change SET_NUM_CO) (amount (+ ?jn (length$ ?lw)))))
)

(defrule wm-proc-complete-without-robot
  (declare (salience ?*PRIORITY-WM*))
  (time $?now)
  ?mf <- (machine (name ?name) (mtype ?mtype) (output ?output) (loaded-with $?lw) (junk ?jn)
           (final-prod-time $?fpt&:(and (timeout ?now ?fpt 0.5) (neq (nth$ 1 ?fpt) 0)))
	   (produced-puck NONE)
         )
  =>
  (printout t "Production completed at " ?name "|" ?mtype crlf)
  ;TODO worldmodel change sync
  (foreach ?puck ?lw
    (assert (worldmodel-change (machine ?name) (change REMOVE_LOADED_WITH) (value ?puck)))
  )
  (assert (worldmodel-change (machine ?name) (change SET_NUM_CO) (amount (- (+ ?jn (length$ ?lw)) 1))))
  (modify ?mf (final-prod-time (create$ 0 0)) (produced-puck ?output))
)

(defrule wm-proc-need-more-ressources
  (declare (salience ?*PRIORITY-WM*))
  (state GOTO-FINAL)
  ?tf <- (goto-target ?name)
  ?gtdw <- (goto-dont-wait ?dont-wait)
  ?hf <- (holding ?was-holding)
  ?lf <- (lights GREEN-OFF YELLOW-ON RED-OFF)
  ?mf <- (machine (name ?name) (mtype ?mtype) (loaded-with $?lw))
  =>
  (retract ?hf ?lf ?tf ?gtdw)
  (assert (holding NONE))
  (if (or (eq ?mtype T1) (eq ?mtype T5))
    then
    (printout error "ERROR: READ YELLOW-ON AT " ?name "|" ?mtype crlf)
    (printout error "This should not happen!" crlf)
    (return)
  )
  (printout t "Production needs more resources at " ?name "|" ?mtype crlf)
  (assert (worldmodel-change (machine ?name) (change ADD_LOADED_WITH) (value ?was-holding)))
)

(defrule wm-proc-inprogress
  (declare (salience ?*PRIORITY-WM*))
  (state GOTO-FINAL)
  ?tf <- (goto-target ?name)
  ?gtdw <- (goto-dont-wait ?dont-wait)
  ?hf <- (holding ?was-holding)
  ?lf <- (lights GREEN-ON YELLOW-ON RED-OFF)
  ?mf <- (machine (name ?name) (mtype ?mtype) (loaded-with $?lw))
  (time $?now)
  (production-time ?mtype ?min-prod-time ?)
  =>
  (retract ?hf ?lf ?tf ?gtdw)
  (assert (holding NONE))
  (printout t "Production in progress at " ?name "|" ?mtype crlf)
  (assert (worldmodel-change (machine ?name) (change ADD_LOADED_WITH) (value ?was-holding))
	  (worldmodel-change (machine ?name) (change SET_PROD_FINISHED_TIME) (amount (+ (nth$ 1 ?now) ?min-prod-time)))
  )
)

(defrule wm-proc-invalid
  (declare (salience ?*PRIORITY-WM*))
  ?s <- (state GOTO-FINAL)
  ?tf <- (goto-target ?name)
  ?gtdw <- (goto-dont-wait ?dont-wait)
  ?lf <- (lights GREEN-OFF YELLOW-BLINKING RED-OFF)
  ?mf <- (machine (name ?name) (mtype ?mtype) (loaded-with $?lw))
  ?hf <- (holding ?puck)
  (role ?role)
  =>
  (printout error "Production invalid at " ?name "|" ?mtype crlf)
  ;keep problem in mind to block machine when if another problem occurs
  (assert (worldmodel-change (machine ?name) (change SET_DOUBTFUL_WORLDMODEL)))
  (retract ?gtdw)
  ;how to handle the situation:
  (if (or (eq ?mtype RECYCLE) 
	  (eq ?mtype T1)
	  (eq ?mtype T5)) 
    then
    ;simply ignore and go on
    (retract ?lf ?tf ?hf)
    (assert (holding NONE))
    (return)
  )
  (if (eq ?puck S0)
    then
    ;it is likely that there already is a S0 because of a to early finished production
    ;remove wrongly loaded puck in wm (if there are S1 and S2 we remove S1 and hope for the best)
    (if (or (subsetp ?lw (create$ S1))
	    (subsetp ?lw (create$ S1 S2)))
      then
      (assert (worldmodel-change (machine ?name) (change REMOVE_LOADED_WITH) (value S1)))
      else
      (if (subsetp ?lw (create$ S2))
	then
	(assert (worldmodel-change (machine ?name) (change REMOVE_LOADED_WITH) (value S2)))
      )
    )
    (assert (worldmodel-change (machine ?name) (change ADD_LOADED_WITH) (value S0)))
    (retract ?lf ?tf ?hf)
    (assert (holding NONE))
    (return)
  )
  (if (and (eq ?puck S1) (eq ?mtype T2))
    then
    ;assume we brought a S0
    (if (not(subsetp ?lw (create$ S0)))
      then
      (assert (worldmodel-change (machine ?name) (change ADD_LOADED_WITH) (value S0)))
    )
    (retract ?lf ?tf ?hf)
    (assert (holding NONE))
    (return)
  )
  (if (eq ?puck S1) ;at T3/T4
    then
    (if (subsetp ?lw (create$ S0))
      then
      ;we brought a second S0
      (retract ?lf ?tf ?hf)
      (assert (holding NONE))
      (return)
    )
    ;there is a S0 and S2
    ;it is more likely that the S2 is a S1 than that the S1 is a S0
    (retract ?lf ?tf ?hf)
    (assert (holding NONE))
    (assert (worldmodel-change (machine ?name) (change REMOVE_LOADED_WITH) (value S2)))
    (assert (worldmodel-change (machine ?name) (change ADD_LOADED_WITH) (value S1)))
    (return)
  )
  ;we tried to bring a S2, it is likely that we instead brought a second S0 or S1
  (retract ?lf ?tf ?hf)
  (assert (holding NONE))
  (return)
)

(defrule wm-proc-delivered
  (declare (salience ?*PRIORITY-WM*))
  (state GOTO-FINAL)
  ?tf <- (goto-target deliver1|deliver2)
  ?hf <- (holding ?was-holding)
  ;?lf <- (lights $?)
  =>
  (retract ?hf ?tf)
  (assert (holding NONE)
	  (delivered ?was-holding)
  )
  (printout t "Delivered " ?was-holding crlf)
)

(defrule wm-proc-wtf
  (declare (salience ?*PRIORITY-WM-LOW*))
  (state GOTO-FINAL)
  ?tf <- (goto-target ?name)
  ?gtdw <- (goto-dont-wait ?dont-wait)
  ?lf <- (lights $?)
  ?hf <- (holding ?)
  ?mf <- (machine (name ?name) (mtype ?mtype))
  (role ?role)
  =>
  (printout error "WTF? Unhandled light code at " ?name "|" ?mtype crlf) 
  (retract ?lf ?tf ?hf ?gtdw)
  (assert (holding NONE))
)

(defrule wm-get-produced-final
  (declare (salience ?*PRIORITY-WM*))
  (state GET-PRODUCED-FINAL)
  ?tf <- (get-produced-target ?name)
  ?hf <- (holding NONE)
  ?mf <- (machine (name ?name) (output ?output))
  =>
  (retract ?hf ?tf)
  (assert (holding ?output))
  (printout t "Got Produced Puck." crlf)
  (assert (worldmodel-change (machine ?name) (change REMOVE_PRODUCED)))
)

(defrule wm-get-produced-failed
  (declare (salience ?*PRIORITY-WM*))
  (state GET-PRODUCED-FAILED)
  ?tf <- (get-produced-target ?name)
  ?hf <- (holding NONE)
  ;?mf <- (machine (name ?name) (output ?output))
  =>
  (retract ?hf ?tf)
  (assert (holding NONE))
  (printout error "Got Produced Puck failed." crlf)
  ;allow one problem. when the second occurs the machine gets blocked
  (assert (worldmodel-change (machine ?name) (change SET_DOUBTFUL_WORLDMODEL)))
  (assert (worldmodel-change (machine ?name) (change REMOVE_PRODUCED)))
  (printout error "remove produced puck in wm!" crlf)
)

(defrule wm-get-consumed-final
  (declare (salience ?*PRIORITY-WM*))
  (state GET-CONSUMED-FINAL)
  ?tf <- (get-consumed-target ?name)
  ?hf <- (holding NONE)
  ?mf <- (machine (name ?name) (junk ?num-junk))
  =>
  (retract ?hf ?tf)
  (assert (holding CO))
  (printout t "Got Consumed Puck." crlf)
  (assert (worldmodel-change (machine ?name) (change SET_NUM_CO) (amount (- ?num-junk 1))))
)

(defrule wm-get-consumed-failed
  (declare (salience ?*PRIORITY-WM*))
  (state GET-CONSUMED-FAILED)
  ?tf <- (get-consumed-target ?name)
  ?hf <- (holding NONE)
  ?mf <- (machine (name ?name) (junk ?))
  =>
  (retract ?hf ?tf)
  (assert (holding NONE))
  (printout error "Got Consumed Puck failed. Assuming holding no puck and junk vanished." crlf)
  ;block this machine to avoid more accidents
  (assert (worldmodel-change (machine ?name) (change SET_RECYCLE_BLOCKED)))
  ;also block recycling because we want recycling points
  (assert (worldmodel-change (machine ?name) (change SET_PRODUCE_BLOCKED)))
)

(defrule wm-process-wm-change-before-sending
  (declare (salience ?*PRIORITY-WM*))
  ?wmc <- (worldmodel-change (machine ?machine) (change ?change) (value ?value) (amount ?amount) (already-applied FALSE))
  ?m <- (machine (name ?machine) (loaded-with $?loaded-with) (incoming $?incoming) (junk ?junk) (produced-puck ?produced) (doubtful-worldmodel ?doubtful-wm))
  =>
  (switch ?change
    (case ADD_LOADED_WITH then 
      (modify ?m (loaded-with (append$ ?loaded-with ?value)))
    )
    (case REMOVE_LOADED_WITH then
      (modify ?m (loaded-with (delete-member$ ?loaded-with ?value)))
    )
    (case ADD_INCOMING then 
      (modify ?m (incoming (append$ ?incoming ?value)))
    )
    (case REMOVE_INCOMING then 
      (modify ?m (incoming (delete-member$ ?incoming ?value)))
    )
    (case SET_NUM_CO then 
      (modify ?m (junk ?amount))
    )
    (case SET_PROD_FINISHED_TIME then 
      (modify ?m (final-prod-time (create$ ?amount 0)))
    )
    (case REMOVE_PRODUCED then 
      (modify ?m (produced-puck NONE))
    )
    (case SET_PRODUCE_BLOCKED then 
      (modify ?m (produce-blocked TRUE))
    )
    (case RESET_PRODUCE_BLOCKED then 
      (modify ?m (produce-blocked FALSE))
    )
    (case SET_RECYCLE_BLOCKED then 
      (modify ?m (recycle-blocked TRUE))
    )
    (case SET_DOUBTFUL_WORLDMODEL then 
      (if ?doubtful-wm
	then
	;second problem -> block this machine
	(modify ?m (recycle-blocked TRUE))
	(modify ?m (produce-blocked TRUE))
	else
        ;one time is ok, set warning
	(modify ?m (doubtful-worldmodel TRUE))
      )
    )
  )
  (modify ?wmc (already-applied TRUE))
)
(defrule wm-process-wm-change-at-order-before-sending
  (declare (salience ?*PRIORITY-WM*))
  ?wmc <- (worldmodel-change (order ?id) (change ADD_IN_DELIVERY)
			     (value ?puck) (amount ?amount) (already-applied FALSE))
  ?order <- (order (id ?id))
  =>
  (modify ?order (in-delivery ?amount))
  (modify ?wmc (already-applied TRUE))
)

(defrule wm-update-pose
  (declare (salience ?*PRIORITY-CLEANUP*))
  ?pif <- (Position3DInterface (id "Pose") (translation $?pos))
  ?pose <- (pose (x ?) (y ?))
  =>
  (modify ?pose (x (nth$ 1 ?pos)) (y (nth$ 2 ?pos)))
  (retract ?pif)
)

(defrule wm-update-puck-in-gripper
  (declare (salience ?*PRIORITY-CLEANUP*))
  ?rif <- (RobotinoSensorInterface (id "Robotino") (distance $?distances))
  ?pig <- (puck-in-gripper ?puck)
  (confval (path "/hardware/robotino/puck_sensor/index") (value ?index))
  (confval (path "/hardware/robotino/puck_sensor/trigger_dist") (value ?trigger))
  =>
  (bind ?dist (nth$ (+ ?index 1) ?distances))
  (if (and (< ?dist ?trigger) (not ?puck))
    then
    ;(printout t "Have puck in gripper" crlf)
    (retract ?pig)
    (assert (puck-in-gripper TRUE))
    
    else
    
    (if (and (> ?dist ?trigger) ?puck)
	then
      ;(printout t "Have no puck in gripper" crlf)
      (retract ?pig)
      (assert (puck-in-gripper FALSE))
    )
  )
)

(defrule wm-dirty-fix-for-junk-T2
  (declare (salience ?*PRIORITY-WM*))
  ?m <- (machine (junk ?n&:(> ?n 1)) (mtype T2) (name ?name))
  (not (junk-failed T2))
  (time $?now)
  =>
  (printout error "Amount of CO pucks at " ?name " wrong." crlf)
  (printout error "Saving fact-list to  agent-snapshot-" (nth$ 1 ?now) ".clp" crlf)
  (save-facts (str-cat "agent-snapshot-junk-wrong-" (nth$ 1 ?now) ".clp") visible)
  
  (modify ?m (junk 1))
  (assert (junk-failed T2))
)
(defrule wm-dirty-fix-for-junk-T1-T5
  (declare (salience ?*PRIORITY-WM*))
  ?m <- (machine (junk ?n&:(neq ?n 0)) (mtype T1|T5) (name ?name))
  (not (junk-failed T1-T5))
  (time $?now)
  =>
  (printout error "Amount of CO pucks at " ?name " wrong." crlf)
  (printout error "Saving fact-list to  agent-snapshot-" (nth$ 1 ?now) ".clp" crlf)
  (save-facts (str-cat "agent-snapshot-junk-wrong-" (nth$ 1 ?now) ".clp") visible)
  
  (modify ?m (junk 0))
  (assert (junk-failed T1-T5))
)
(defrule wm-dirty-fix-for-junk-T3-T4
  (declare (salience ?*PRIORITY-WM*))
  ?m <- (machine (junk ?n&:(> ?n 2)) (mtype T3|T4) (name ?name))
  (not (junk-failed T3-T4))
  (time $?now)
  =>
  (printout error "Amount of CO pucks at " ?name " wrong." crlf)
  (printout error "Saving fact-list to  agent-snapshot-" (nth$ 1 ?now) ".clp" crlf)
  (save-facts (str-cat "agent-snapshot-junk-wrong-" (nth$ 1 ?now) ".clp") visible)
  
  (modify ?m (junk 2))
  (assert (junk-failed T3-T4))
)