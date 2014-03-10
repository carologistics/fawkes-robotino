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
    ;add sub-msg to worldmodel msg
    (pb-add-list ?worldmodel "machines" ?m-msg) ; destroys ?m
  )
  (pb-broadcast ?worldmodel)
  (pb-destroy ?worldmodel)
)

;receive worldmodel
(defrule worldmodel-sync-receive-worldmodel
  (protobuf-msg (type "llsf_msgs.Worldmodel") (ptr ?p) (rcvd-via BROADCAST))
  (not (lock-role MASTER))
  =>
  ; (printout t "***** Received Worldmodel *****" crlf)
  (foreach ?m-msg (pb-field-list ?p "machines")
    (do-for-fact ((?machine machine))
      (eq ?machine:name (sym-cat (pb-field-value ?m-msg "name")))
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
      (modify ?machine (incoming ?incoming) (loaded-with ?loaded-with))
    )
  )
)

(defrule debug-worldmodel
  (lock-role MASTER)
  ?m <- (machine (name M1))
  (not (debug-inited))
  =>
  (modify ?m (loaded-with (create$ S1 S2)) (incoming (create$ PICK_PROD)))
  (assert (debug-inited))
)

(deftemplate worldmodel-change
  (slot machine (type SYMBOL) (allowed-values M1 M2 M3 M4 M5 M6 M7 M8 M9 M10 D1 D2 D3 TST R1 R2))
  (slot change (type SYMBOL) (allowed-values ADD_LOADED_WITH REMOVE_LOADED_WITH ADD_INCOMING REMOVE_INCOMING))
  (slot value (type SYMBOL) (allowed-symbols S0 S1 S2 BRING_S0 BRING_S1 BRING_S2 PICK_PROD PICK_CO))
)

;send worldmodel change
(defrule worldmodel-sync-send-change
  ?wmc <- (worldmodel-change (machine ?m) (change ?change) (value ?value))
  (not (lock-role MASTER))
  =>
  (printout t "sending worldmodel change" crlf)
  (bind ?change-msg (pb-create "llsf_msgs.WorldmodelChange"))
  (pb-set-field ?change-msg "machine" (str-cat ?m))
  (pb-set-field ?change-msg "change" ?change)
  (if (member$ ?change (create$ ADD_LOADED_WITH REMOVE_LOADED_WITH)) then
    (pb-set-field ?change-msg "loaded_with" ?value)
  )
  (if (member$ ?change (create$ ADD_INCOMING REMOVE_INCOMING)) then
    (pb-set-field ?change-msg "incoming" ?value)
  )
  (pb-broadcast ?change-msg)
  (pb-destroy ?change-msg)
  (retract ?wmc)
)
;the master does not have to send the change, because it sends the whole worldmodel
(defrule worldmodel-sync-send-loaded-with-add
  ?wmc <- (worldmodel-change (machine ?m) (change ?change) (value ?value))
  (lock-role MASTER)
  =>
  (retract ?wmc)
)

;receive worldmodel change
(defrule worldmodel-sync-receive-change
  (protobuf-msg (type "llsf_msgs.WorldmodelChange") (ptr ?p) (rcvd-via BROADCAST))
  (lock-role MASTER)
  =>
  (printout t "receiving worldmodel change" crlf)
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
        (modify ?machine (incoming (append$ ?machine:incoming (pb-field-value ?p "incoming"))))
      )
      (case REMOVE_INCOMING then 
        (modify ?machine (incoming (delete-member$ ?machine:incoming (pb-field-value ?p "incoming"))))
      )
    )
  )
)