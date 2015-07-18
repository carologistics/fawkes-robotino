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
  ; (printout t "***** Received MachineInfo *****" crlf)
  (foreach ?m (pb-field-list ?p "machines")
    (bind ?m-name (sym-cat (pb-field-value ?m "name")))
    (bind ?m-type (sym-cat (pb-field-value ?m "type")))
    (bind ?m-team (sym-cat (pb-field-value ?m "team_color")))
    (bind ?m-state (sym-cat (pb-field-value ?m "state")))
    (do-for-fact ((?machine machine))
      (eq ?machine:name ?m-name)
      
      (if (or (neq ?machine:team ?m-team) (neq ?machine:state ?m-state)) then
        (modify ?machine (state ?m-state) (team ?m-team))
      )
    )
    ; set available rings for ring-stations
    (if (eq ?m-type RS) then
      (do-for-fact ((?rs ring-station)) (eq ?rs:name ?m-name)
        (if (eq 0 (length$ ?rs:available-colors)) then
          (bind ?colors (create$))
          (progn$ (?rc (pb-field-list ?m "ring_colors"))
            (bind ?colors (insert$ ?colors 1 (utils-remove-prefix ?rc "RING_")))
          )
          (modify ?rs (available-colors ?colors))
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

(defrule wm-insert-product-into-rs-final
  (declare (salience ?*PRIORITY-WM*))
  (state SKILL-FINAL)
  (skill-to-execute (skill bring_product_to) (state final) (target ?mps))
  (step (name insert) (state running) (ring ?ring-color))
  (ring (color ?ring-color) (req-bases ?rb))
  (task (name add-first-ring))
  ?mf <- (machine (name ?mps) (loaded-id 0) (produced-id 0))
  ?rsf <- (ring-station (name ?mps) (bases-loaded ?bl))
  ?hf <- (holding ?product-id)
  ?pf <- (product (id ?product-id) (rings $?r))
  =>
  (retract ?hf)
  (printout t "Inserted product " ?product-id " to have a ring assembled in the RS" crlf)
  (assert (holding NONE))
  ; there is no relevant waiting time until the cs has finished the loading step right?
  (synced-modify ?mf produced-id ?product-id)
  (synced-modify ?rsf bases-loaded (- ?bl ?rb))
  (synced-add-to-multifield ?pf rings ?ring-color)
)

(defrule wm-insert-product-into-cs-final
  (declare (salience ?*PRIORITY-WM*))
  (state SKILL-FINAL)
  (skill-to-execute (skill bring_product_to) (state final) (target ?mps))
  (step (name insert) (state running))
  (task (name produce-c0|produce-cx))
  ?mf <- (machine (name ?mps) (loaded-id 0) (produced-id 0))
  ?csf <- (cap-station (name ?mps) (cap-loaded ?cap))
  ?hf <- (holding ?produced-id)
  ?pf <- (product (id ?produced-id) (product-id ?product-id) (cap NONE))
  ?of <- (order (product-id ?product-id) (in-production ?ip))
  =>
  (retract ?hf)
  (printout t "Inserted product " ?produced-id " to be finished in the CS" crlf)
  (assert (holding NONE))
  ; there is no relevant waiting time until the cs has finished the loading step right?
  (synced-modify ?mf produced-id ?produced-id)
  (synced-modify ?csf cap-loaded NONE)
  (synced-modify ?pf cap ?cap)
)

(defrule wm-insert-product-into-ds-final
  (declare (salience ?*PRIORITY-WM*))
  (state SKILL-FINAL)
  (skill-to-execute (skill bring_product_to) (state final) (target ?mps))
  (step (name insert) (state running))
  (task (name deliver))
  ?mf <- (machine (name ?mps) (mtype DS))
  ?hf <- (holding ?produced-id&~NONE)
  (product
    (id ?produced-id)
    (product-id ?product-id)
  )
  ?of <- (order (product-id ?product-id)
    (quantity-requested ?qr) (quantity-delivered ?qd)
    (begin ?begin) 
    (delivery-gate ?gate) (in-delivery ?id)
  )
  =>
  (retract ?hf)
  (printout t "Delivered product " ?product-id " to " ?mps crlf)
  (assert (holding NONE))
  (synced-modify ?of in-delivery (- ?id 1) quantity-delivered (+ ?qd 1))
)

(defrule wm-insert-base-into-rs-slide-final
  (declare (salience ?*PRIORITY-WM*))
  (state SKILL-FINAL)
  (skill-to-execute (skill bring_product_to) (state final) (target ?mps))
  (step (name insert) (state running) (machine-feature SLIDE))
  (task (name fill-rs))
  ?rsf <- (ring-station (name ?mps) (bases-needed ?bn) (bases-loaded ?bl))
  ?hf <- (holding ?product-id)
  ?pf <- (product (id ?product-id))
  =>
  (retract ?hf ?pf)
  (printout t "Inserted base " ?product-id " into slide of " ?mps crlf)
  (assert (holding NONE))
  (if (> ?bn 0) then
    (bind ?rsf (synced-modify ?rsf bases-needed (- ?bn 1)))
  )
  (if (< ?bl 3) then
    (synced-modify ?rsf bases-loaded (+ ?bl 1))
  )
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
  (step (name get-output|get-base) (state running))
  ?mf <- (machine (name ?mps) (produced-id ?produced-id))
  ?hf <- (holding NONE)
  =>
  (retract ?hf)
  (printout t "Fetched product " ?produced-id " from the output of " ?mps crlf)
  (assert (holding ?produced-id))
  (synced-modify ?mf produced-id 0)
)

(defrule wm-get-output-failed
  (declare (salience ?*PRIORITY-WM*))
  (state SKILL-FAILED)
  (skill-to-execute (skill get_product_from) (state failed) (target ?mps))
  (step (name get-output|get-base) (state running))
  ?mf <- (machine (name ?mps) (produced-id ?puck-id&~0))
  =>
  (printout t "Failed to fetch a product from the output of " ?mps crlf)
  (printout t "I assume there is no more output product at " ?mps crlf)
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
  ?grip <- (AX12GripperInterface (id "Gripper AX12") (holds_puck ?holds-puck))
  ?pig <- (puck-in-gripper ?puck)
  =>
  (if (eq ?holds-puck TRUE)
    then
    (printout t "Have puck in gripper" crlf)
    (retract ?pig)
    (assert (puck-in-gripper TRUE))
  
    else
    (printout t "Have no puck in gripper" crlf)
    (retract ?pig)
    (assert (puck-in-gripper FALSE))
  )
  (retract ?grip)
)

(defrule wm-set-bs-output-color
  "Set the correct loaded-id after color is ordered at BS"
  (declare (salience ?*PRIORITY-WM*))
  ?bs <- (machine (mtype BS) (produced-id 0) (state PROCESSED|PREPARED|READY-AT-OUTPUT))
  (step (name get-base) (state running) (base ?base-color))
  (step (name get-base) (state running) (base ?base-color) (product-id ?product-id))
  (not (skill-to-execute (skill get_product_from) (state final|failed)))
  =>
  (bind ?produced-id (random-id))
  (synced-assert (str-cat "(product (id " ?produced-id ") (product-id " ?product-id
                          ") (base " ?base-color ") (cap NONE))"))
  (synced-modify ?bs produced-id ?produced-id)
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

(defrule wm-reset-broken-machine
  "When a machine is in the broken state, the referee has to remove all pucks from it and the state is resetted"
  (declare (salience ?*PRIORITY-WM*))
  ?m <- (machine (name ?name) (state BROKEN) (loaded-id ?lid) (produced-id ?pid)
                 (final-prod-time $?fpt&:(or (neq ?lid 0)
                                             (neq ?pid 0)
                                             (neq (nth$ 1 ?fpt) 0))))
  =>
  (printout warn "MPS " ?name " is broken and has to be reset!" crlf)
  (modify ?m (loaded-id 0) (produced-id 0) (final-prod-time (create$ 0 0)))
)

(defrule wm-reset-broken-cap-station
  "When a machine is in the broken state, the referee has to remove all pucks from it and the state is resetted"
  (declare (salience ?*PRIORITY-WM*))
  (machine (name ?name) (state BROKEN) (mtype CS))
  ?cs <- (cap-station (name ?name) (cap-loaded ~NONE))
  =>
  (printout warn "cap-station " ?name " is broken and has to be reset!" crlf)
  (modify ?cs (cap-loaded NONE))
)

(defrule wm-reset-broken-ring-station
  "When a machine is in the broken state, the referee has to remove all pucks from it and the state is resetted"
  (declare (salience ?*PRIORITY-WM*))
  (machine (name ?name) (state BROKEN) (mtype RS))
  ?rs <- (ring-station (name ?name) (bases-loaded ?bl)
                       (selected-color ?sc&:(or (neq ?bl 0)
                                                (neq ?sc NONE))))
  =>
  (printout warn "ring-station " ?name " is broken and has to be reset!" crlf)
  (modify ?rs (bases-loaded 0) (selected-color NONE))
)
