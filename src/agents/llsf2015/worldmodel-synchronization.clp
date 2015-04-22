;---------------------------------------------------------------------------
;  worldmodel-synchronization.clp - Synchronize the worldmodel with the other robots
;
;  Created: Mon Mar 10 15:03:07 2014
;  Copyright  2014  Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

;publish worldmodel
(defrule worldmodel-sync-publish-worldmodel
  (time $?now)
  ?s <- (timer (name send-worldmodel-sync) (time $?t&:(timeout ?now ?t ?*WORLDMODEL-SYNC-PERIOD*)) (seq ?seq))
  (lock-role MASTER)
  (peer-id private ?peer)  
  =>
  ;construct worldmodel msg
  (bind ?worldmodel (pb-create "llsf_msgs.Worldmodel"))
  (delayed-do-for-all-facts ((?machine machine)) TRUE
    ;construct submsg for each machine
    (bind ?m-msg (pb-create "llsf_msgs.MachineWMState"))
    ;set name
    (pb-set-field ?m-msg "name" (str-cat ?machine:name))
    ;set loaded_with)
    (pb-set-field ?m-msg "loaded_id" ?machine:loaded-id)
    ;set incoming
    (foreach ?incoming-action (fact-slot-value ?machine incoming)
      (pb-add-list ?m-msg "incoming" (str-cat ?incoming-action))
    )
    ;set agents responsible for incoming fields
    (foreach ?incoming-agent (fact-slot-value ?machine incoming-agent)
      (pb-add-list ?m-msg "incoming_agent" (str-cat ?incoming-agent))
    )
    ;set production finished time
    (pb-set-field ?m-msg "prod_finished_time" (nth$ 1 ?machine:final-prod-time))
    ;set out-of-order-until
    (pb-set-field ?m-msg "out_of_order_until" (nth$ 1 ?machine:out-of-order-until))
    ;set produced
    (pb-set-field ?m-msg "produced_id" ?machine:produced-id)

    ;add additional fields for cap-stations:
    (if (eq ?machine:mtype CS) then
      (do-for-fact ((?cs cap-station)) (eq ?cs:name ?machine:name)
	(pb-set-field ?m-msg "cap_loaded" ?cs:cap-loaded)
      )
    )

    ;add additional fields for ring-stations:
    (if (eq ?machine:mtype RS) then
      (do-for-fact ((?rs ring-station)) (eq ?rs:name ?machine:name)
	(if (neq ?rs:selected-color NONE) then
	  (pb-set-field ?m-msg "ring_color_selected" (sym-cat RING_ ?rs:selected-color))
	  ; empty field means NONE
	)
	(pb-set-field ?m-msg "ring_bases_needed" ?rs:bases-needed)
      )
    )
    
    ;add sub-msg to worldmodel msg
    (pb-add-list ?worldmodel "machines" ?m-msg) ; destroys ?m
  )

  ;add worldmodel about orders
  (delayed-do-for-all-facts ((?order order)) TRUE
    ;construct submsg for each order
    (bind ?o-msg (pb-create "llsf_msgs.WorldmodelOrder"))
    (pb-set-field ?o-msg "id" ?order:id)
    (pb-set-field ?o-msg "in_delivery" ?order:in-delivery)
    (pb-add-list ?worldmodel "orders" ?o-msg) ; destroys ?o-msg
  )
  
  ;add worldmodel about puck-storage
  (delayed-do-for-all-facts ((?ps puck-storage)) TRUE
    ;construct submsg for each machine
    (bind ?ps-msg (pb-create "llsf_msgs.PuckStorageState"))
    ;set name
    (pb-set-field ?ps-msg "name" (str-cat ?ps:name))
    (if (neq ?ps:puck NONE)
      then
      ;set puck
      (pb-set-field ?ps-msg "puck" ?ps:puck)
    )
    ;set incoming
    (foreach ?incoming-action (fact-slot-value ?ps incoming)
      (pb-add-list ?ps-msg "incoming" (str-cat ?incoming-action))
    )
    ;set agents responsible for incoming fields
    (foreach ?incoming-agent (fact-slot-value ?ps incoming-agent)
      (pb-add-list ?ps-msg "incoming_agent" (str-cat ?incoming-agent))
    )
    (pb-add-list ?worldmodel "storage" ?ps-msg)
  )

  ;add worldmodel about exploration-zones
  (delayed-do-for-all-facts ((?zone zone-exploration)) TRUE
    ;construct submsg for each machine
    (bind ?zone-msg (pb-create "llsf_msgs.ZoneState"))
    (pb-set-field ?zone-msg "name" (str-cat ?zone:name))
    (pb-set-field ?zone-msg "machine" (str-cat ?zone:machine))
    (pb-set-field ?zone-msg "recognized" ?zone:recognized)
    (pb-set-field ?zone-msg "still_to_explore" ?zone:still-to-explore)
    (pb-set-field ?zone-msg "times_searched" ?zone:times-searched)

    (pb-add-list ?worldmodel "zones" ?zone-msg)
  )

  ;add worldmodel about all (intermediate-) products
  (delayed-do-for-all-facts ((?prod product)) TRUE
    ;construct submsg for each product
    (bind ?prod-msg (pb-create "llsf_msgs.ProductState"))
    (pb-set-field ?prod-msg "id" ?prod:id)
    (if (neq ?prod:base UNKNOWN) then
      (pb-set-field ?prod-msg "base" (sym-cat BASE_ ?prod:base))
    )
    (progn$ (?ring ?prod:rings)
      (pb-add-list ?prod-msg "rings" (sym-cat RING_ ?ring))
    )
    (pb-set-field ?prod-msg "cap" ?prod:cap)

    (pb-add-list ?worldmodel "products" ?prod-msg)
  )

  (pb-broadcast ?peer ?worldmodel)
  (pb-destroy ?worldmodel)

  (modify ?s (time ?now) (seq (+ ?seq 1)))
)

;receive worldmodel
(defrule worldmodel-sync-receive-worldmodel
  ?msg <- (protobuf-msg (type "llsf_msgs.Worldmodel") (ptr ?p))
  (not (lock-role MASTER))
  (state ?state)
  =>
  ; (printout t "***** Received Worldmodel *****" crlf)
  (unwatch facts machine)
  (unwatch facts cap-station)
  (unwatch facts ring-station)
  (unwatch facts order)
  (unwatch facts puck-storage)
  (unwatch facts zone-exploration)
  ;update worldmodel about machines
  (foreach ?m-msg (pb-field-list ?p "machines")
    (do-for-fact ((?machine machine))
		   (eq ?machine:name (sym-cat (pb-field-value ?m-msg "name")))
			
      ;update loaded-with
      (bind ?loaded-id (pb-field-value ?m-msg "loaded_id"))
      ;update incoming
      (bind $?incoming (create$))
      (progn$ (?incoming-action (pb-field-list ?m-msg "incoming"))
	(bind ?incoming (append$ ?incoming (sym-cat ?incoming-action)))
      )
      ;update agents responsible for incoming-fields
      (bind $?incoming-agent (create$))
      (progn$ (?agent (pb-field-list ?m-msg "incoming_agent"))
	(bind ?incoming-agent (append$ ?incoming-agent (sym-cat ?agent)))
      )
      ;update production finished time
      (bind ?prod-finished-time (pb-field-value ?m-msg "prod_finished_time"))
      ;update out-of-order-until
      (bind ?out-of-order-until (pb-field-value ?m-msg "out_of_order_until"))
      ;update produced-id
      (bind ?produced-id (pb-field-value ?m-msg "produced_id"))
      
      (modify ?machine (incoming ?incoming) (loaded-id ?loaded-id)
	               (final-prod-time (create$ ?prod-finished-time 0))
		       (produced-id ?produced-id)
		       (incoming-agent ?incoming-agent)
		       (out-of-order-until (create$ ?out-of-order-until 0))
      )

      ; update additional fields of cap-stations
      (if (eq ?machine:mtype CS) then
	(do-for-fact ((?cs cap-station))  (eq ?machine:name ?cs:name)
	  (modify ?cs (cap-loaded (pb-field-value ?m-msg "cap_loaded")))
	)
      )

      ; update additional fields of ring-stations
      (if (eq ?machine:mtype RS) then
	(do-for-fact ((?rs ring-station))  (eq ?machine:name ?rs:name)
	  (bind ?ring-color-selected NONE)
	  (if (pb-has-field ?m-msg "ring_color_selected") then
	    (bind ?ring-color-selected (sym-cat (utils-remove-prefix (pb-field-value ?m-msg "ring_color_selected") RING_))) 
	  )
	  (modify ?rs (selected-color ?ring-color-selected)
		  (bases-needed (pb-field-value ?m-msg "ring_bases_needed")))
	)
      )
    )
  )
  ;update worldmodel about orders
  (foreach ?o-msg (pb-field-list ?p "orders")
    (do-for-fact ((?order order))
        (eq ?order:id (pb-field-value ?o-msg "id"))

      (bind ?in-del (pb-field-value ?o-msg "in_delivery"))
      (modify ?order (in-delivery ?in-del))
    )
  )
  ;update worldmodel about puck-storage
  (foreach ?ps-msg (pb-field-list ?p "storage")
    (do-for-fact ((?storage puck-storage))
		   (eq ?storage:name (sym-cat (pb-field-value ?ps-msg "name")))	
      ;update incoming
      (bind $?incoming (create$))
      (progn$ (?incoming-action (pb-field-list ?ps-msg "incoming"))
	(bind ?incoming (append$ ?incoming (sym-cat ?incoming-action)))
      )
      ;update agents responsible for incoming-fields
      (bind $?incoming-agent (create$))
      (progn$ (?agent (pb-field-list ?ps-msg "incoming_agent"))
	(bind ?incoming-agent (append$ ?incoming-agent (sym-cat ?agent)))
      )
      ;update loaded-with
      (if (pb-has-field ?ps-msg "puck")
	then
	(modify ?storage (incoming ?incoming) (incoming-agent ?incoming-agent)
		         (puck (pb-field-value ?ps-msg "puck")))
	else
	(modify ?storage (incoming ?incoming) (incoming-agent ?incoming-agent)
		         (puck NONE))
      )	
    )
  )

  ;update worldmodel about zones
  (foreach ?zone-msg (pb-field-list ?p "zones")
    (do-for-fact ((?zone zone-exploration))
		 (eq ?zone:name (sym-cat (pb-field-value ?zone-msg "name")))
      (modify ?zone (machine (sym-cat (pb-field-value ?zone-msg "machine")))
	      (recognized (pb-field-value ?zone-msg "recognized"))
	      (still-to-explore (pb-field-value ?zone-msg "still_to_explore"))
              (times-searched (pb-field-value ?zone-msg "times_searched")))
    )
  )

  ;update worldmodel about (intermediate-)products
  (foreach ?prod-msg (pb-field-list ?p "products")
    (bind ?product-exists FALSE)
    (bind ?prod-id (pb-field-value ?prod-msg "id"))
    (bind ?prod-cap (pb-field-value ?prod-msg "cap"))
    (if (pb-has-field ?prod-msg "base") then
      (bind ?prod-base (utils-remove-prefix (pb-field-value ?prod-msg "base") BASE_))
      else
      (bind ?prod-base UNKNOWN)
    )
    (bind ?prod-rings (create$ ))
    (progn$ (?ring (pb-field-list ?prod-msg "rings"))
      (bind ?prod-rings (append$ ?prod-rings (utils-remove-prefix ?ring RING_)))
    )
    ;modify if product already exists
    (do-for-fact ((?prod product)) (eq ?prod:id ?prod-id)
      (bind ?product-exists TRUE)
      (modify ?prod (base ?prod-base) (rings ?prod-rings) (cap ?prod-cap))
    )
    ;assert if it is new
    (if (not ?product-exists) then
      (assert (product (id ?prod-id) (base ?prod-base) (rings ?prod-rings) (cap ?prod-cap)))
    )
  )
  (retract ?msg)
  (watch facts machine)
  (watch facts cap-station)
  (watch facts ring-station)
  (watch facts order)
  (watch facts puck-storage)
  (watch facts zone-exploration)
)

(defrule worldmodel-sync-publish-tag-poses
  (time $?now)
  ?s <- (timer (name send-tag-poses-sync) (time $?t&:(timeout ?now ?t ?*WORLDMODEL-SYNC-PERIOD*)) (seq ?seq))
  (lock-role MASTER)
  (peer-id private ?peer)  
  =>
  ;construct worldmodel msg
  (bind ?all-tags-msgs (pb-create "llsf_msgs.TagPositions"))

  ;add all tag-positions
  (delayed-do-for-all-facts ((?tag found-tag)) TRUE
    ;construct submsg for each machine

    (bind ?tag-msg (pb-create "llsf_msgs.TagPosition"))
    (pb-set-field ?tag-msg "machine" (str-cat ?tag:name))
    (pb-set-field ?tag-msg "side" ?tag:side)
    (pb-set-field ?tag-msg "frame" (str-cat ?tag:frame))
    (pb-add-list ?tag-msg "trans" (nth$ 1 ?tag:trans))
    (pb-add-list ?tag-msg "trans" (nth$ 2 ?tag:trans))
    (pb-add-list ?tag-msg "trans" (nth$ 3 ?tag:trans))
    (pb-add-list ?tag-msg "rot" (nth$ 1 ?tag:rot))
    (pb-add-list ?tag-msg "rot" (nth$ 2 ?tag:rot))
    (pb-add-list ?tag-msg "rot" (nth$ 3 ?tag:rot))
    (pb-add-list ?tag-msg "rot" (nth$ 4 ?tag:rot))

    (pb-add-list ?all-tags-msgs "tag_positions" ?tag-msg)
  )

  (pb-broadcast ?peer ?all-tags-msgs)
  (pb-destroy ?all-tags-msgs)

  (modify ?s (time ?now) (seq (+ ?seq 1)))
)


(defrule worldmodel-sync-receive-tag-poses
  ?msg <- (protobuf-msg (type "llsf_msgs.TagPositions") (ptr ?p))
  (not (lock-role MASTER))
  =>
  ; (printout t "***** Received Tag Poses *****" crlf)

  ;update worldmodel about tag-positions
  (foreach ?tag-pose-msg (pb-field-list ?p "tag_positions")
    ; create tag fact if not already known
    (if (not (any-factp ((?ft found-tag)) 
			(eq ?ft:name (sym-cat (pb-field-value ?tag-pose-msg "machine")))))
      then
      (assert (found-tag (name (sym-cat (pb-field-value ?tag-pose-msg "machine")))
			 (side (sym-cat (pb-field-value ?tag-pose-msg "side")))
			 (frame (pb-field-value ?tag-pose-msg "frame"))
			 (trans (pb-field-list ?tag-pose-msg "trans"))
			 (rot (pb-field-list ?tag-pose-msg "rot")))
	      )
      (printout t "Adding tag with side: " (sym-cat (pb-field-value ?tag-pose-msg "side"))
		crlf)
    )
  )
)

;send worldmodel change
(defrule worldmodel-sync-send-change
  (time $?now)
  ?wmc <- (worldmodel-change (machine ?m) (change ?change) (value ?value)
			     (puck-id ?puck-id)
			     (amount ?amount) (id ?id) (order ?order) (agent ?agent)
			     (last-sent $?ls&:(timeout ?now ?ls ?*WORLDMODEL-CHANGE-SEND-PERIOD*)))
  (not (lock-role MASTER))
  (peer-id private ?peer)
  =>
  ;(printout t "sending worldmodel change" crlf)
  ;set random id (needed by the master to determine if a change was already appied)
  (if (eq ?id 0) then
    (bind ?id (random 1 99999999))
  )
  (bind ?change-msg (pb-create "llsf_msgs.WorldmodelChange"))
  (if (neq ?m NONE) then
    (pb-set-field ?change-msg "place" (str-cat ?m))
  )
  (if (neq ?order 0) then 
    (pb-set-field ?change-msg "place" (str-cat "Order-" ?order))
  )
  (if (neq ?puck-id 0) then 
    (pb-set-field ?change-msg "place" (str-cat ?puck-id))
  )
  (pb-set-field ?change-msg "change" (str-cat ?change))
  (pb-set-field ?change-msg "agent" (str-cat ?agent))
  (pb-set-field ?change-msg "id" ?id)
  (if (member$ ?change (create$ ADD_INCOMING REMOVE_INCOMING ZONE_STILL_TO_EXPLORE
				ZONE_MACHINE_IDENTIFIED ADD_RING SET_CAP
				SET_CAP_LOADED SET_SELECTED_COLOR SET_BASE)) then
    (pb-set-field ?change-msg "value_string" (str-cat ?value))
  )
  (if (member$ ?change (create$ SET_NUM_CO SET_PROD_FINISHED_TIME
				SET_OUT_OF_ORDER_UNTIL SET_IN_DELIVERY
				SET_LOADED SET_PRODUCED REMOVE_PRODUCED
				REMOVE_LOADED REMOVE_PUCK SET_BASES_NEEDED)) then
    (pb-set-field ?change-msg "value_uint32" ?amount)
  )
  (if (eq ?change ADD_TAG)
    then
    ; Find corresponding tag-fact
    (do-for-fact ((?tag found-tag)) (eq ?tag:name ?m)
      (bind ?tag-msg (pb-create "llsf_msgs.TagPosition"))
      (pb-set-field ?tag-msg "machine" (str-cat ?tag:name))
      (pb-set-field ?tag-msg "side" ?tag:side)
      (pb-set-field ?tag-msg "frame" (str-cat ?tag:frame))
      (pb-add-list ?tag-msg "trans" (nth$ 1 ?tag:trans))
      (pb-add-list ?tag-msg "trans" (nth$ 2 ?tag:trans))
      (pb-add-list ?tag-msg "trans" (nth$ 3 ?tag:trans))
      (pb-add-list ?tag-msg "rot" (nth$ 1 ?tag:rot))
      (pb-add-list ?tag-msg "rot" (nth$ 2 ?tag:rot))
      (pb-add-list ?tag-msg "rot" (nth$ 3 ?tag:rot))
      (pb-add-list ?tag-msg "rot" (nth$ 4 ?tag:rot))
      
      (printout t "Sending Tag Pos: " ?tag:name ", " ?tag:frame 
		", (" ?tag:trans "), " ?tag:rot ")" crlf)

      (pb-set-field ?change-msg "tag_position" ?tag-msg)
    )
  )
  (if (eq ?change NEW_PUCK)
    then
    ; Find corresponding product-fact
    (do-for-fact ((?prod product)) (eq ?prod:id ?puck-id)
      (bind ?prod-msg (pb-create "llsf_msgs.ProductState"))
      (pb-set-field ?prod-msg "id" ?prod:id)
      (if (neq ?prod:base UNKNOWN) then
        (pb-set-field ?prod-msg "base" (sym-cat BASE_ ?prod:base))
      )
      (progn$ (?ring ?prod:rings)
        (pb-add-list ?prod-msg "rings" (sym-cat RING_ ?ring))
      )
      (pb-set-field ?prod-msg "cap" ?prod:cap)

      (pb-set-field ?change-msg "value_puckstate" ?prod-msg)
    )
  )
  (pb-broadcast ?peer ?change-msg)
  (pb-destroy ?change-msg)
  (modify ?wmc (last-sent ?now) (id ?id))
)

;the master does not have to send the change, because it sends the whole worldmodel
(defrule worldmodel-sync-retract-as-master
  (declare (salience ?*PRIORITY-CLEANUP*))
  ?wmc <- (worldmodel-change (machine ?m) (change ?change) (value ?value))
  (lock-role MASTER)
  =>
  (printout warn "retract wmc" crlf)
  (retract ?wmc)
)

;receive worldmodel change
(defrule worldmodel-sync-receive-change
  ?pmsg <- (protobuf-msg (type "llsf_msgs.WorldmodelChange") (ptr ?p))
  (lock-role MASTER)
  ?arf <- (already-received-wm-changes $?arc)
  (peer-id private ?peer)
  (time $?now)
  =>
  ;ensure that this change was not already applied
  (bind ?id (pb-field-value ?p "id"))
  (if (not (member$ ?id ?arc)) then
    ;(printout t "receiving new worldmodel change (" (pb-field-value ?p "change") ")" crlf)
    (retract ?arf)
    (assert (already-received-wm-changes (append$ ?arc ?id)))
    ;apply change
    (if (pb-has-field ?p "place") then
      ;change machine if change is about a machine
      (do-for-fact ((?machine machine))
          (eq ?machine:name (sym-cat (pb-field-value ?p "place")))

        (switch (sym-cat (pb-field-value ?p "change"))
          (case SET_LOADED then 
	    (modify ?machine (loaded-id (pb-field-value ?p "value_uint32")))
          )
          (case REMOVE_LOADED then
	    (modify ?machine (loaded-id 0))
          )
          (case ADD_INCOMING then 
            (modify ?machine (incoming (append$ ?machine:incoming
						(sym-cat (pb-field-value ?p "value_string"))))
		             (incoming-agent (append$ ?machine:incoming-agent
						      (sym-cat (pb-field-value ?p "agent")))))
          )
          (case REMOVE_INCOMING then 
	    (if (member$ (pb-field-value ?p "value_string") ?machine:incoming)
	      then
	      (modify ?machine (incoming (delete-member$ ?machine:incoming
							 (pb-field-value ?p "value_string")))
	                     ;every agent should do only one thing at a machine
		             (incoming-agent (delete-member$ ?machine:incoming-agent
                                               (sym-cat (pb-field-value ?p "agent")))))
	      else
	      ;After the change of a decision based on a new worldmodel the remove msg might have arrived before the add message. Wait until there is a field to remove or a timer has passed
	      ;this is a workaround and could be solved properly with sequence numbers for the change msgs
	      (assert (delayed-worldmodel-change REMOVE_INCOMING
				?machine:name (pb-field-value ?p "value_string")
				(sym-cat (pb-field-value ?p "agent")) ?now))
	    )
          )
          (case SET_PROD_FINISHED_TIME then 
            (modify ?machine (final-prod-time (create$ (pb-field-value ?p "value_uint32") 0)))
          )
	  (case SET_OUT_OF_ORDER_UNTIL then
	    (modify ?machine (out-of-order-until (create$ (pb-field-value ?p "value_uint32") 0)))
	  )
          (case SET_PRODUCED then 
	    (modify ?machine (produced-id (pb-field-value ?p "value_uint32")))
          )
          (case REMOVE_PRODUCED then
	    (modify ?machine (produced-id 0))
          )
          (case ADD_TAG then
	    ; create tag fact if not already known
	    (if (not (any-factp ((?ft found-tag)) 
				(eq ?ft:name (pb-field-value ?p "place"))))
	      then
	      (bind ?tag-pose-msg (pb-field-value ?p "tag_position"))
	      (assert (found-tag (name (sym-cat (pb-field-value ?tag-pose-msg "machine")))
				 (side (sym-cat (pb-field-value ?tag-pose-msg "side")))
				 (frame (pb-field-value ?tag-pose-msg "frame"))
				 (trans (pb-field-list ?tag-pose-msg "trans"))
				 (rot (pb-field-list ?tag-pose-msg "rot")))
	      )
	      (printout t "Received Tag: " (sym-cat (pb-field-value ?tag-pose-msg "machine"))
			(sym-cat (pb-field-value ?tag-pose-msg "side"))
			(pb-field-value ?tag-pose-msg "frame")
			" trans: " (pb-field-list ?tag-pose-msg "trans")
			" rot: " (pb-field-list ?tag-pose-msg "rot")
			crlf)
	    )
	    ; set machine stuff
	    ; set explration stuff
          )
	  ;for cap-stations:
	  (case SET_CAP_LOADED then
	    (do-for-fact ((?cs cap-station)) (eq ?cs:name ?machine:name)
	      (modify ?cs (cap-loaded (sym-cat (pb-field-value ?p "value_string"))))
	    )
	  )
	  ;for ring-stations:
	  (case SET_SELECTED_COLOR then
	    (do-for-fact ((?rs ring-station)) (eq ?rs:name ?machine:name)
	      (modify ?rs (selected-color (sym-cat (pb-field-value ?p "value_string"))))
	    )
	  )
	  (case SET_BASES_NEEDED then
	    (do-for-fact ((?rs ring-station)) (eq ?rs:name ?machine:name)
	      (modify ?rs (bases-needed (pb-field-value ?p "value_uint32")))
	    )
	  )
	  (default
	    (printout error "Worldmodel-Change Type " (sym-cat (pb-field-value ?p "change"))
		      " is not handled for machine. Worlmodel is probably wrong." crlf)
	  )
        )
      )
      ;change puck-storage if change is about a puck-storage
      (do-for-fact ((?storage puck-storage))
          (eq ?storage:name (sym-cat (pb-field-value ?p "place")))

        (switch (sym-cat (pb-field-value ?p "change"))
          (case ADD_LOADED_WITH then 
	    (modify ?storage (puck (pb-field-value ?p "value_puckstate")))
          )
          (case REMOVE_LOADED_WITH then
            (modify ?storage (puck NONE))
          )
          (case ADD_INCOMING then 
            (modify ?storage (incoming (append$ ?storage:incoming
                                         (pb-field-value ?p "value_string")))
                             (incoming-agent (append$ ?storage:incoming-agent
                                               (sym-cat (pb-field-value ?p "agent")))))
          )
          (case REMOVE_INCOMING then 
            (modify ?storage (incoming (delete-member$ ?storage:incoming
                                         (pb-field-value ?p "value_string")))
	                     ;every agent should do only one thing at a machine
		             (incoming-agent (delete-member$ ?storage:incoming-agent
                                               (sym-cat (pb-field-value ?p "agent")))))
          )
	  (default
	    (printout error "Worldmodel-Change Type " (sym-cat (pb-field-value ?p "change"))
		      " is not handled for puck-storage. Worlmodel is probably wrong." crlf)
	  )
        )
      )
      ;change puck-storage if change is about an order
      (do-for-fact ((?order order))
          (eq (str-cat "Order-" ?order:id) (pb-field-value ?p "place"))
        (switch (sym-cat (pb-field-value ?p "change"))
          (case _IN_DELIVERY then 
	    (modify ?order (in-delivery (pb-field-value ?p "value_uint32")))
          )
	)
      )
      ;change zone-exploration if change is about a zone
      (do-for-fact ((?zone zone-exploration))
          (eq ?zone:name (sym-cat (pb-field-value ?p "place")))

        (switch (sym-cat (pb-field-value ?p "change"))
          (case ZONE_STILL_TO_EXPLORE then 
	    (modify ?zone (still-to-explore (sym-cat (pb-field-value ?p "value_string"))))
          )
          (case ZONE_MACHINE_IDENTIFIED then 
	    (modify ?zone (machine (sym-cat (pb-field-value ?p "value_string"))))
          )
          (case ZONE_TIMES_SEARCHED_INCREMENT then 
	    (modify ?zone (times-searched (+ 1 ?zone:times-searched)))
          )
	  (default
	    (printout error "Worldmodel-Change Type " (sym-cat (pb-field-value ?p "change"))
		      " is not handled for zone. Worlmodel is probably wrong." crlf)
	  )
        )
      )
      ;change product if change is about a product
      (if (eq (sym-cat (pb-field-value ?p "change")) NEW_PUCK) then
	(bind ?prod-msg (pb-field-value ?p "value_puckstate"))
	(bind ?prod-id (pb-field-value ?prod-msg "id"))
        (if (pb-has-field ?prod-msg "base") then
          (bind ?prod-base (utils-remove-prefix (pb-field-value ?prod-msg "base") BASE_))
          else
          (bind ?prod-base UNKNOWN)
        )
	(bind ?prod-rings (create$ ))
	(progn$ (?ring (pb-field-list ?prod-msg "rings"))
	  (bind ?prod-rings (append$ ?prod-rings 
				     (utils-remove-prefix ?ring RING_)))
	)
	(bind ?prod-cap (pb-field-value ?prod-msg "cap"))
	(assert (product (id ?prod-id) (base ?prod-base) (cap ?prod-cap) (rings ?prod-rings)))
	
	else
	;modify existing puck fact if there is one
	(do-for-fact ((?prod product)) (eq ?prod:id (eval (pb-field-value ?p "place")))
          (switch (sym-cat (pb-field-value ?p "change"))
	    (case SET_BASE then 
	      (modify ?prod (base (sym-cat (pb-field-value ?p "value_string"))))
	    )
	    (case SET_CAP then 
	      (modify ?prod (cap (sym-cat (pb-field-value ?p "value_string"))))
	    )
	    (case REMOVE_PUCK then 
	      (retract ?prod)
	    )
	    (case ADD_RING then 
	      (modify ?prod (rings (append$ ?prod:rings 
					    (sym-cat (pb-field-value ?p "value_string")))))
	    )
	    (default
	      (printout error "Worldmodel-Change Type "
			(sym-cat (pb-field-value ?p "change"))
			" is not handled for puck. Worlmodel is probably wrong." crlf)
	    )
	  )
        )
      )
    )
  )
  (retract ?pmsg)
  ;send acknowledgment
  (bind ?msg-ack (pb-create "llsf_msgs.WorldmodelChangeAck"))
  (pb-set-field ?msg-ack "id" ?id)  
  (pb-broadcast ?peer ?msg-ack)
  (pb-destroy ?msg-ack)
)

(defrule worldmodel-sync-receive-change-ack
  ?pmsg <- (protobuf-msg (type "llsf_msgs.WorldmodelChangeAck") (ptr ?p))
  (not (lock-role MASTER))
  =>
  (do-for-fact ((?wmc worldmodel-change))
	       (eq ?wmc:id (pb-field-value ?p "id"))
    (retract ?wmc)
  )
  (retract ?pmsg)
)

(defrule worldmodel-sync-apply-delayed-worldmodel-change
  ?dwf <- (delayed-worldmodel-change REMOVE_INCOMING ?machine-name ?field-to-remove ?agent-removing $?)
  ?machine <- (machine (name ?machine-name)
		       (incoming $?incoming&:(member$ ?field-to-remove ?incoming))
		       (incoming-agent $?incoming-agent&:(member$ ?agent-removing ?incoming-agent)))
  =>
  (modify ?machine (incoming (delete-member$ ?incoming ?field-to-remove))
	  (incoming-agent (delete-member$ ?incoming-agent ?agent-removing)))
)

(defrule worldmodel-sync-delete-delayed-worldmodel-change-after-timeout
  (time $?now)
  ?dwf <- (delayed-worldmodel-change ? ? ? ? $?time-asserted&:(timeout ?now ?time-asserted ?*DELAYED-WORLDMODEL-CHANGE-TIMEOUT*))
  =>
  (retract ?dwf)
)

; (defrule worldmodel-sync-debug
;   (lock-role SLAVE)
;   =>
;   (assert (found-tag (name C-BS) (side OUTPUT)(frame "/map")
; 		       (trans (create$ -4.9 4.9 0)) (rot (tf-quat-from-yaw -1.14)))
; 	    (worldmodel-change (machine C-BS) (change ADD_TAG)))
; )

; (defrule worldmodel-sync-debug
;   (lock-role SLAVE)
;   =>
;   (assert (found-tag (name C-DS) (side OUTPUT)(frame "/map")
; 		       (trans (create$ 4.9 4.9 0)) (rot (tf-quat-from-yaw -1.14)))
; 	    (worldmodel-change (machine C-DS) (change ADD_TAG)))
; )
