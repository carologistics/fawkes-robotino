;---------------------------------------------------------------------------
;  worldmodel.clp - Robotino agent -- world model update rules
;
;  Created: Sat Jun 16 18:50:53 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; knowledge evaluation request
(defrule wm-recv-MachineInfo
  ?pb-msg <- (protobuf-msg (type "llsf_msgs.MachineInfo") (ptr ?p))
  =>
  ;(printout t "***** Received MachineInfo *****" crlf)
  (foreach ?m (pb-field-list ?p "machines")
    (bind ?m-name (sym-cat (pb-field-value ?m "name")))
    (bind ?m-type (sym-cat (pb-field-value ?m "type")))
    (bind ?m-team (sym-cat (pb-field-value ?m "team_color")))
    (do-for-fact ((?machine machine))
      (eq ?machine:name ?m-name)
      
      (if (or (neq ?machine:mtype ?m-type) (neq ?machine:team ?m-team)) then
        (printout t "Machine " ?m-name " (" ?m-team ") is of type " ?m-type crlf)
        (printout warn "TODO: set available-colors for ring stations " ?m-type crlf)
        (modify ?machine (mtype ?m-type) (team ?m-team))
        
        else
        (bind ?m-prepared (sym-cat (pb-field-value ?m "prepared")))
        (if (neq ?m-prepared ?machine:prepared) then
          (modify ?machine (prepared ?m-prepared))
        )
      )
    )
  )
  (assert (received-machine-info))
  (retract ?pb-msg)
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
  (synced-assert (str-cat "(product (id " ?puck-id ") (cap " ?color "))"))
  (assert (holding ?puck-id))
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
  ?mps-f <- (machine (name ?mps))
  ?cs-f <- (cap-station (name ?mps))
  (step (name insert) (state running))
  (task (name fill-cap))
  ;an inserted puck without cap will be the final product
  ?mf <- (machine (name ?mps) (loaded-id 0) (produced-id 0))
  ?csf <- (cap-station (name ?mps))
  ?hf <- (holding ?puck-id)
  ?pf <- (product (id ?puck-id) (cap ?cap))
  =>
  (retract ?hf)
  (printout t "Inserted a Puck from an CS shelf with a " ?cap " cap to fill the CS" crlf)
  (assert (holding NONE))
  (synced-modify ?mps-f produced-id ?puck-id)
  (synced-modify ?cs-f cap-loaded ?cap)
  (synced-modify ?pf cap NONE)
)

(defrule wm-insert-product-into-cs-final
  (declare (salience ?*PRIORITY-WM*))
  (state SKILL-FINAL)
  (skill-to-execute (skill bring_product_to) (state final) (target ?mps))
  (step (name insert) (state running))
  (task (name produce-c0|deliver-c0))
  ?mf <- (machine (name ?mps) (loaded-id 0) (produced-id 0))
  ?csf <- (cap-station (name ?mps) (cap-loaded ?cap))
  ?hf <- (holding ?product-id)
  ?pf <- (product (id ?product-id) (cap NONE))
  =>
  (retract ?hf)
  (printout t "Inserted product " ?product-id " to be finished in the CS" crlf)
  (assert (holding NONE))
  ; there is no relevant waiting time until the cs has finished the loading step right?
  (synced-modify ?mf produced-id ?product-id)
  (synced-modify ?csf cap-loaded NONE)
  (synced-modify ?pf cap ?cap)
)

(defrule wm-insert-product-into-ds-final
  (declare (salience ?*PRIORITY-WM*))
  (state SKILL-FINAL)
  (skill-to-execute (skill bring_product_to) (state final) (target ?mps))
  (step (name insert) (state running))
  (task (name produce-c0|deliver-c0))
  ?mf <- (machine (name ?mps) (mtype DS))
  ?hf <- (holding ?product-id&~NONE)
  =>
  (retract ?hf)
  (printout t "Delivered product " ?product-id " to " ?mps crlf)
  (assert (holding NONE))
)

(defrule wm-insert-failed
  (declare (salience ?*PRIORITY-WM*))
  (state SKILL-FAILED)
  (skill-to-execute (skill bring_product_to) (state failed) (target ?mps))
  (step (name insert) (state running))
  ?hf <- (holding ?puck-id)
  ?pf <- (product (id ?puck-id))
  =>
  (retract ?hf ?pf)
  (printout t "Insertion of " ?puck-id " into " ?mps " failed" crlf)
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
  (printout t "Fetched Puck " ?puck-id " from the output of " ?mps crlf)
  (assert (holding ?puck-id))
  (synced-modify ?mf produced-id 0)
)

(defrule wm-get-output-failed
  (declare (salience ?*PRIORITY-WM*))
  (state SKILL-FAILED)
  (skill-to-execute (skill get_product_from) (state failed) (target ?mps))
  (step (name get-output) (state running))
  ?mf <- (machine (name ?mps) (produced-id ?puck-id&~0))
  =>
  (printout t "Failed to fetch a Puck from the output of " ?mps crlf)
  (printout t "I assume there is no more output puck at " ?mps crlf)
  (synced-modify ?mf produced-id 0)
)

(defrule wm-store-lights
  (declare (salience ?*PRIORITY-WM*))
  ?rf <- (RobotinoLightInterface (id "Light determined") (ready TRUE)
				 (red ?red) (green ?green) (yellow ?yellow))
  =>
  (retract ?rf)
  ; remove all possibly existing last-lights facts
  (delayed-do-for-all-facts ((?ll last-lights)) TRUE (retract ?ll))
  (assert (last-lights (green ?green) (yellow ?yellow) (red ?red)))
  (printout t "***** Lights 5 (green " ?green ") (yellow " ?yellow ") (red " ?red ")" crlf)
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

;(defrule wm-goto-failed
;  (declare (salience ?*PRIORITY-WM-LOW*))
;  (state SKILL-FAILED)
;  (skill-to-execute (skill finish_puck_at) (state failed) (target ?name))
;  ?gtdw <- (dont-wait ?dont-wait)
;  ?hf <- (holding ?)
;  (puck-in-gripper ?puck)
;  =>
;  (retract ?gtdw)
;  (if (not ?puck) then
;    (retract ?hf)
;    (assert (holding NONE))
;  )
;  (printout error "Production failed at " ?name crlf) 
;)

;(defrule wm-take-puck-to-failed
;  (declare (salience ?*PRIORITY-WM-LOW*))
;  (state SKILL-FAILED)
;  (skill-to-execute (skill take_puck_to) (state failed) (target ?name))
;  ?hf <- (holding ~NONE)
;  (puck-in-gripper ?puck)
;  =>
;  (if (not ?puck) then
;    (retract ?hf)
;    (assert (holding NONE))
;    (printout error "Lost puck during take_puck_to" crlf)
;  )
;)

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

;(defrule wm-goto-light
;  (declare (salience ?*PRIORITY-WM*))
;  (state SKILL-FINAL)
;  (skill-to-execute (skill finish_puck_at) (state final))
;  (not (lights))
;  ?lf <- (last-lights $?lights&:(> (length$ ?lights) 0))
;  =>
;  (retract ?lf)
;  (assert (lights ?lights))
;)

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

(defrule wm-set-bs-output-color
  "Set the correct loaded-id after color is ordered at BS"
  (declare (salience ?*PRIORITY-WM*))
  ?bs <- (machine (mtype BS) (produced-id 0) (prepared TRUE))
  (step (name get-base) (state running) (base ?base-color))
  =>
  (bind ?product-id (random-id))
  (synced-assert (str-cat "(product (id " ?product-id
                          ") (base " ?base-color ") (cap NONE))"))
  (modify ?bs (produced-id ?product-id))
)

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
