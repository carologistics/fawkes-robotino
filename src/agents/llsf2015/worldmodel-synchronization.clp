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
  ; (printout t "sending worldmodel" crlf)
  ;construct worldmodel msg
  (bind ?worldmodel (pb-create "llsf_msgs.Worldmodel"))
  (delayed-do-for-all-facts ((?machine machine)) TRUE
    ;construct submsg for each machine
    (bind ?m-msg (pb-create "llsf_msgs.MachineState"))
    ;set name
    (pb-set-field ?m-msg "name" (str-cat ?machine:name))
    ;set loaded_with
    (foreach ?puck-type (fact-slot-value ?machine loaded-with)
      (pb-add-list ?m-msg "loaded_with" ?puck-type)
    )
    ;set incoming
    (foreach ?incoming-action (fact-slot-value ?machine incoming)
      (pb-add-list ?m-msg "incoming" ?incoming-action)
    )
    ;set agents responsible for incoming fields
    (foreach ?incoming-agent (fact-slot-value ?machine incoming-agent)
      (pb-add-list ?m-msg "incoming_agent" (str-cat ?incoming-agent))
    )
    ;set production finished time
    (pb-set-field ?m-msg "prod_finished_time" (nth$ 1 ?machine:final-prod-time))
    ;set out-of-order-until
    (pb-set-field ?m-msg "out_of_order_until" (nth$ 1 ?machine:out-of-order-until))
    ;set puck under rfid
    (if (neq ?machine:produced-puck NONE) then
      (pb-set-field ?m-msg "puck_under_rfid" ?machine:produced-puck)
    )
    (pb-set-field ?m-msg "produce_blocked" ?machine:produce-blocked)
    (pb-set-field ?m-msg "recycle_blocked" ?machine:recycle-blocked)
    ;set amount of junk
    (pb-set-field ?m-msg "junk" ?machine:junk)
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
      (pb-add-list ?ps-msg "incoming" ?incoming-action)
    )
    ;set agents responsible for incoming fields
    (foreach ?incoming-agent (fact-slot-value ?ps incoming-agent)
      (pb-add-list ?ps-msg "incoming_agent" (str-cat ?incoming-agent))
    )
    (pb-add-list ?worldmodel "storage" ?ps-msg)
  )

  (pb-broadcast ?peer ?worldmodel)
  (pb-destroy ?worldmodel)
)

;receive worldmodel
(defrule worldmodel-sync-receive-worldmodel
  ?msg <- (protobuf-msg (type "llsf_msgs.Worldmodel") (ptr ?p))
  (not (lock-role MASTER))
  (state ?state)
  =>
  ; (printout t "***** Received Worldmodel *****" crlf)
  (unwatch facts machine)
  (unwatch facts order)
  (unwatch facts puck-storage)
  ;update worldmodel about machines
  (foreach ?m-msg (pb-field-list ?p "machines")
    (do-for-fact ((?machine machine))
		   (and (eq ?machine:name (sym-cat (pb-field-value ?m-msg "name")))
			;prevent idle P3 robot from using an old world model the master sends before the master gets the change at the T5
			(or (neq ?state IDLE) (neq ?machine:mtype T5)))
			
      ;update loaded-with
      (bind $?loaded-with (create$))
      (progn$ (?puck-type (pb-field-list ?m-msg "loaded_with"))
	(bind ?loaded-with (append$ ?loaded-with (sym-cat ?puck-type)))
      )
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
      (if (pb-has-field ?m-msg "puck_under_rfid")
	then (bind ?puck-under-rfid (sym-cat (pb-field-value ?m-msg "puck_under_rfid")))
	else (bind ?puck-under-rfid NONE)
      )
      (bind ?produce-blocked (if (eq 0 (pb-field-value ?m-msg "produce_blocked"))
				 then FALSE
				 else TRUE))
      (bind ?recycle-blocked (if (eq 0 (pb-field-value ?m-msg "recycle_blocked"))
				 then FALSE
				 else TRUE))
      ;update junk
      (bind ?junk (pb-field-value ?m-msg "junk"))
      (modify ?machine (incoming ?incoming) (loaded-with ?loaded-with)
	               (final-prod-time (create$ ?prod-finished-time 0))
		       (produced-puck ?puck-under-rfid) (produce-blocked ?produce-blocked)
		       (recycle-blocked ?recycle-blocked) (junk ?junk)
		       (incoming-agent ?incoming-agent)
		       (out-of-order-until (create$ ?out-of-order-until 0))
      )
    )
    (retract ?msg)
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
    (retract ?msg)
  )
  (watch facts machine)
  (watch facts order)
  (watch facts puck-storage)
)

;send worldmodel change
(defrule worldmodel-sync-send-change
  (time $?now)
  ?wmc <- (worldmodel-change (machine ?m) (change ?change) (value ?value)
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
    (pb-set-field ?change-msg "machine" (str-cat ?m))
  )
  (if (neq ?order 0) then 
    (pb-set-field ?change-msg "order" ?order)
  )
  (pb-set-field ?change-msg "change" ?change)
  (pb-set-field ?change-msg "agent" (str-cat ?agent))
  (pb-set-field ?change-msg "id" ?id)
  (if (member$ ?change (create$ ADD_LOADED_WITH REMOVE_LOADED_WITH)) then
    (pb-set-field ?change-msg "loaded_with" ?value)
  )
  (if (member$ ?change (create$ ADD_INCOMING REMOVE_INCOMING)) then
    (pb-set-field ?change-msg "incoming" ?value)
  )
  (if (eq ?change SET_NUM_CO) then
    (pb-set-field ?change-msg "num_CO" ?amount)
  )
  (if (eq ?change SET_PROD_FINISHED_TIME) then
    (pb-set-field ?change-msg "prod_finished_time" ?amount)
  )
  (if (eq ?change SET_OUT_OF_ORDER_UNTIL) then
    (pb-set-field ?change-msg "out_of_order_until" ?amount)
  )
  (if (eq ?change SET_IN_DELIVERY) then
    (pb-set-field ?change-msg "in_delivery" ?amount)
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
    (if (pb-has-field ?p "machine") then
      ;change machine if change is about a machine
      (do-for-fact ((?machine machine))
          (eq ?machine:name (sym-cat (pb-field-value ?p "machine")))

        (switch (pb-field-value ?p "change")
          (case ADD_LOADED_WITH then 
	    (modify ?machine (loaded-with (append$ ?machine:loaded-with (pb-field-value ?p "loaded_with"))))
          )
          (case REMOVE_LOADED_WITH then
            (modify ?machine (loaded-with (delete-member$ ?machine:loaded-with (pb-field-value ?p "loaded_with"))))
          )
          (case ADD_INCOMING then 
            (modify ?machine (incoming (append$ ?machine:incoming
                                         (pb-field-value ?p "incoming")))
                             (incoming-agent (append$ ?machine:incoming-agent
                                               (sym-cat (pb-field-value ?p "agent")))))
          )
          (case REMOVE_INCOMING then 
	    (if (member$ (pb-field-value ?p "incoming") ?machine:incoming)
	      then
	      (modify ?machine (incoming (delete-member$ ?machine:incoming
							 (pb-field-value ?p "incoming")))
	                     ;every agent should do only one thing at a machine
		             (incoming-agent (delete-member$ ?machine:incoming-agent
                                               (sym-cat (pb-field-value ?p "agent")))))
	      else
	      ;After the change of a decision based on a new worldmodel the remove msg might have arrived before the add message. Wait until there is a field to remove or a timer has passed
	      ;this is a workaround and could be solved properly with sequence numbers for the change msgs
	      (assert (delayed-worldmodel-change REMOVE_INCOMING
				?machine:name (pb-field-value ?p "incoming")
				(sym-cat (pb-field-value ?p "agent")) ?now))
	    )
          )
          (case SET_NUM_CO then 
            (modify ?machine (junk (pb-field-value ?p "num_CO")))
          )
          (case SET_PROD_FINISHED_TIME then 
            (modify ?machine (final-prod-time (create$ (pb-field-value ?p "prod_finished_time") 0)))
          )
	  (case SET_OUT_OF_ORDER_UNTIL then
	    (modify ?machine (out-of-order-until (create$ (pb-field-value ?p "out_of_order_until") 0)))
	  )
          (case REMOVE_PRODUCED then 
            (modify ?machine (produced-puck NONE))
          )
          (case SET_PRODUCE_BLOCKED then 
            (modify ?machine (produce-blocked TRUE))
          )
          (case RESET_PRODUCE_BLOCKED then 
            (modify ?machine (produce-blocked FALSE))
          )
          (case SET_RECYCLE_BLOCKED then 
            (modify ?machine (recycle-blocked TRUE))
          )
          (case SET_DOUBTFUL_WORLDMODEL then 
	    (if ?machine:doubtful-worldmodel
	      then
	      ;second problem -> block this machine
	      (modify ?machine (recycle-blocked TRUE))
	      (modify ?machine (produce-blocked TRUE))
	      else
	      ;one time is ok, set warning
	      (modify ?machine (doubtful-worldmodel TRUE))
	    )
          )
        )
      )
      ;change puck-storage if change is about a puck-storage
      (do-for-fact ((?storage puck-storage))
          (eq ?storage:name (sym-cat (pb-field-value ?p "machine")))

        (switch (pb-field-value ?p "change")
          (case ADD_LOADED_WITH then 
	    (modify ?storage (puck (pb-field-value ?p "loaded_with")))
          )
          (case REMOVE_LOADED_WITH then
            (modify ?storage (puck NONE))
          )
          (case ADD_INCOMING then 
            (modify ?storage (incoming (append$ ?storage:incoming
                                         (pb-field-value ?p "incoming")))
                             (incoming-agent (append$ ?storage:incoming-agent
                                               (sym-cat (pb-field-value ?p "agent")))))
          )
          (case REMOVE_INCOMING then 
            (modify ?storage (incoming (delete-member$ ?storage:incoming
                                         (pb-field-value ?p "incoming")))
	                     ;every agent should do only one thing at a machine
		             (incoming-agent (delete-member$ ?storage:incoming-agent
                                               (sym-cat (pb-field-value ?p "agent")))))
          )
        )
      )
    )
    (if (pb-has-field ?p "order") then
      (do-for-fact ((?order order))
          (eq ?order:id (pb-field-value ?p "order"))

        (switch (sym-cat (pb-field-value ?p "change"))
          (case _IN_DELIVERY then 
	    (modify ?order (in-delivery (pb-field-value ?p "in_delivery")))
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