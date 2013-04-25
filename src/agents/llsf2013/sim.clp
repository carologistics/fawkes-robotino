
;---------------------------------------------------------------------------
;  sim.clp - Robotino agent -- simulation
;
;  Created: Sat Jun 16 16:58:22 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate sim-machine
  (slot name (type SYMBOL) (allowed-values M1 M2 M3 M4 M5 M6 M7 M8 M9 M10 D1 D2 D3 TST R1 R2))
  (slot mtype (type SYMBOL) (allowed-values T1 T2 T3 T4 T5 DELIVER TEST RECYCLE) (default TEST))
  (multislot loaded-with (type INTEGER) (default))
  (multislot lights (type SYMBOL)
	     (allowed-values RED-ON RED-BLINK YELLOW-ON YELLOW-BLINK GREEN-ON GREEN-BLINK)
	     (default) (cardinality 0 3))
  (slot puck-id (type INTEGER) (default 0))
   ; x y theta (meters and rad)
  (multislot pose (type FLOAT) (cardinality 3 3) (default 0.0 0.0 0.0))
)

(deftemplate sim-machine-spec
  (slot mtype (type SYMBOL) (allowed-values T1 T2 T3 T4 T5 DELIVER TEST RECYCLE))
  (multislot inputs (type SYMBOL) (allowed-symbols S0 S1 S2) (default))
  (slot output (type SYMBOL) (allowed-symbols NONE S0 S1 S2 P1 P2 P3))
)

(deftemplate sim-puck
  (slot index (type INTEGER))
  (slot id (type INTEGER))
  (slot state (type SYMBOL) (allowed-values S0 S1 S2 P1 P2 P3 CONSUMED) (default S0))
)

(deftemplate sim
  (slot proc-state (allowed-values IDLE WAIT PROC) (default IDLE))
  (slot rb-client-id (type INTEGER))
  (slot goto-target (type SYMBOL))
  (slot holding-puck-id (type INTEGER))
  (slot placed-puck-id (type INTEGER))
)


(defrule sim-enable-sim
  (confval (path "/clips-agent/llsf2013/enable-sim") (type BOOL) (value true))
  =>
  (bind ?client-id (pb-connect "localhost" 4444))
  (assert (sim (rb-client-id ?client-id))
	  (sim-machine (name M1))
	  (sim-machine (name M2))
	  (sim-machine (name M3))
	  (sim-machine (name M4))
	  (sim-machine (name M5))
	  (sim-machine (name M6))
	  (sim-machine (name M7))
	  (sim-machine (name M8))
	  (sim-machine (name M9))
	  (sim-machine (name M10))
	  (sim-machine (name R1))
	  (sim-machine (name R2))
	  (sim-machine (name TST))
	  (sim-machine (name D1))
	  (sim-machine (name D2))
	  (sim-machine (name D3))
  )
)

(defrule sim-connected
  (sim (rb-client-id ?client-id))
  ?cf <- (protobuf-client-connected ?client-id)
  =>
  (retract ?cf)
  (printout t "Simulation connected to refbox" crlf)
)

;(defrule sim-disconnected
;  ?sf <- (sim (rb-client-id ?client-id))
;  ?df <- (protobuf-client-disconnected ?client-id)
;  =>
;  (retract ?df)
;  (modify ?sf (rb-client-id 0))
;)

(defrule sim-recv-PuckInfo
  (protobuf-msg (type "llsf_msgs.PuckInfo") (ptr ?p) (rcvd-via STREAM))
  =>
  (if (any-factp ((?pt sim-puck)) TRUE)
  then
    (foreach ?pp (pb-field-list ?p "pucks")
      (do-for-fact ((?puck sim-puck)) (eq ?puck:id (pb-field-value ?pp "id"))
        (bind ?new-state (pb-field-value ?pp "state"))
        (if (neq ?puck:state ?new-state) then (modify ?puck (state ?new-state)))
      )
    )
  else
    (foreach ?pp (pb-field-list ?p "pucks")
      (assert (sim-puck (id (pb-field-value ?pp "id")) (state (pb-field-value ?pp "state"))))
    )
  )
)

(defrule sim-recv-MachineInfo
  ?pf <- (protobuf-msg (type "llsf_msgs.MachineInfo") (ptr ?p) (rcvd-via STREAM))
  =>
  (retract ?pf)
  (foreach ?m (pb-field-list ?p "machines")
    ;(printout t "Processing machine " (pb-field-value ?m "name") crlf)
    (do-for-fact ((?machine sim-machine))
		 (eq ?machine:name (sym-cat (pb-field-value ?m "name")))

      (bind ?new-puck ?machine:puck-id)
      (if (pb-has-field ?m "puck_under_rfid") then
        (bind ?new-puck (pb-field-value (pb-field-value ?m "puck_under_rfid") "id"))
        ;(printout t "puck under rfid " ?new-puck crlf)
      )

      (bind ?new-lights (create$))
      (progn$ (?l (pb-field-list ?m "lights"))
	(bind ?light (sym-cat (pb-field-value ?l "color") "-" (pb-field-value ?l "state")))
        (bind ?new-lights (append$ ?new-lights ?light))
      )

      (bind ?new-lw (create$))
      (progn$ (?l (pb-field-list ?m "loaded_with"))
        (bind ?new-lw (append$ ?new-lw (pb-field-value ?l "id")))
      )

      (bind ?new-mtype (sym-cat (pb-field-value ?m "type")))

      (if (or (neq ?machine:mtype ?new-mtype) (neq ?machine:lights ?new-lights)
	      (neq ?machine:puck-id ?new-puck) (neq ?machine:loaded-with ?new-lw))
        then
        (printout t ?machine:name "|" ?new-mtype " modified - L " ?new-lights
		  " P " ?new-puck " " ?new-lw crlf)
        (modify ?machine (mtype ?new-mtype) (lights ?new-lights)
		(puck-id ?new-puck) (loaded-with ?new-lw))
      )
    )
  )
)


(defrule sim-get-s0-final
  (declare (salience ?*PRIORITY-SIM*))
  (skill-done (name "get_s0") (status FINAL))
  (sim-puck (state S0) (id ?puck-id))
  (not (sim-machine (puck-id ?puck-id)))
  (not (sim-machine (loaded-with $?lw&:(member$ ?puck-id ?lw))))
  ?sf <- (sim (holding-puck-id 0))
  =>
  (modify ?sf (holding-puck-id ?puck-id))
)

(defrule sim-goto-final-deliver
  (declare (salience ?*PRIORITY-SIM*))
  (skill-done (name "finish_puck_at") (status FINAL))
  (goto-target deliver)
  ?sf <- (sim (rb-client-id ?client-id) (holding-puck-id ?puck-id&~0))
  (sim-machine (name ?name) (mtype DELIVER) (lights GREEN-ON))
  =>
  (printout warn "Placing " ?puck-id " under " ?name crlf)
  (bind ?m (pb-create "llsf_msgs.PlacePuckUnderMachine"))
  (pb-set-field ?m "machine_name" ?name)
  (pb-set-field ?m "puck_id" ?puck-id)
  (pb-send ?client-id ?m)
  (pb-destroy ?m)
  (modify ?sf (proc-state WAIT) (holding-puck-id 0) (placed-puck-id ?puck-id)
	  (goto-target ?name))
)

(defrule sim-goto-final
  (declare (salience ?*PRIORITY-SIM*))
  (skill-done (name "finish_puck_at") (status FINAL))
  (goto-target ?gt)
  ?sf <- (sim (rb-client-id ?client-id) (holding-puck-id ?puck-id&~0))
  =>
  (printout warn "Placing " ?puck-id " under " ?gt crlf)
  (bind ?m (pb-create "llsf_msgs.PlacePuckUnderMachine"))
  (pb-set-field ?m "machine_name" ?gt)
  (pb-set-field ?m "puck_id" ?puck-id)
  (pb-send ?client-id ?m)
  (pb-destroy ?m)
  (modify ?sf (proc-state WAIT) (holding-puck-id 0) (placed-puck-id ?puck-id) (goto-target ?gt))
)

(defrule sim-proc-start
  (declare (salience ?*PRIORITY-SIM*))
  ?sf <- (sim (proc-state WAIT) (goto-target ?gt))
  (sim-machine (name ?gt) (lights GREEN-ON YELLOW-ON))
  =>
  (modify ?sf (proc-state PROC))
)


(defrule sim-proc-final
  (declare (salience ?*PRIORITY-SIM*))
  ?sf <- (sim (proc-state PROC) (goto-target ?gt) (rb-client-id ?client-id)
	      (placed-puck-id ?puck-id&~0))
  (sim-machine (name ?gt) (mtype ~DELIVER)
	       (puck-id ?puck-id) (lights ?light&GREEN-ON|YELLOW-ON))
  =>
  (bind ?m (pb-create "llsf_msgs.RemovePuckFromMachine"))
  (pb-set-field ?m "machine_name" ?gt)
  (pb-set-field ?m "puck_id" ?puck-id)
  (pb-send ?client-id ?m)
  (pb-destroy ?m)
  (modify ?sf (proc-state IDLE) (placed-puck-id 0)
	  (holding-puck-id (if (eq ?light GREEN-ON) then ?puck-id else 0)))
  (assert (lights ?light))
)

(defrule sim-proc-delivered
  (declare (salience ?*PRIORITY-SIM*))
  ?sf <- (sim (proc-state PROC) (goto-target ?gt) (rb-client-id ?client-id)
	      (placed-puck-id ?puck-id&~0))
  (sim-machine (name ?gt) (mtype DELIVER)
	       (puck-id ?puck-id) (lights GREEN-ON YELLOW-ON RED-ON))
  =>
  (bind ?m (pb-create "llsf_msgs.RemovePuckFromMachine"))
  (pb-set-field ?m "machine_name" ?gt)
  (pb-set-field ?m "puck_id" ?puck-id)
  (pb-send ?client-id ?m)
  (pb-destroy ?m)
  (modify ?sf (proc-state IDLE) (placed-puck-id 0) (holding-puck-id 0))
  (assert (lights GREEN-ON YELLOW-ON RED-ON))
)

(defrule sim-proc-fail
  (declare (salience ?*PRIORITY-SIM*))
  ?sf <- (sim (proc-state WAIT) (goto-target ?gt) (rb-client-id ?client-id)
	      (placed-puck-id ?puck-id&~0))
  (sim-machine (puck-id ?puck-id) (lights ?lights&YELLOW-BLINK))
  =>
  (bind ?m (pb-create "llsf_msgs.RemovePuckFromMachine"))
  (pb-set-field ?m "machine_name" ?gt)
  (pb-set-field ?m "puck_id" ?puck-id)
  (pb-send ?client-id ?m)
  (pb-destroy ?m)
  (modify ?sf (proc-state IDLE) (placed-puck-id 0) (holding-puck-id ?puck-id))
  (assert (lights ?lights))
)
