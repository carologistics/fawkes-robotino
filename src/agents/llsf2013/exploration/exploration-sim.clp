;---------------------------------------------------------------------------
;  exploration-sim.clp - Simulation for the exploration agent to play against the refbox
;
;  Created: Sat Jun 16 16:58:22 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de], Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate sim-machine-exp
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

(deftemplate sim-exp
  (slot state (allowed-values IDLE GET-S0 GOTO WAIT-PROC) (default IDLE))
  (slot proc-state (allowed-values IDLE WAIT PROC) (default IDLE))
  (slot rb-client-id (type INTEGER))
  (slot goto-target (type SYMBOL))
  (slot holding-puck-id (type INTEGER))
  (slot placed-puck-id (type INTEGER))
)


(defrule sim-enable-sim-exp
  (confval (path "/clips-agent/llsf2013/enable-sim") (type BOOL) (value true))
  =>
  (bind ?client-id (pb-connect "localhost" 4444))
  (assert (sim-exp (rb-client-id ?client-id))
	  (sim-machine-exp (name M1))
	  (sim-machine-exp (name M2))
	  (sim-machine-exp (name M3))
	  (sim-machine-exp (name M4))
	  (sim-machine-exp (name M5))
	  (sim-machine-exp (name M6))
	  (sim-machine-exp (name M7))
	  (sim-machine-exp (name M8))
	  (sim-machine-exp (name M9))
	  (sim-machine-exp (name M10))
  )
  (assert (simulation-is-running))
)

(defrule sim-start-exp
  (declare (salience ?*PRIORITY_HIGH*))
  (simulation-is-running)
  (start)
  =>
  (printout t "Simulation of exploration phase needs the Robotino position to find the nearest machine. Simulated position is always (0.5 0.5 0.0)." crlf)
  (assert (Position3DInterface (id "Pose") (translation (create$ 0.5 0.5 0.0))))
)

(defrule sim-connected-exp
  (sim-exp (rb-client-id ?client-id))
  ?cf <- (protobuf-client-connected ?client-id)
  =>
  (retract ?cf)
  (printout t "Simulation connected to refbox" crlf)
)

;(defrule sim-disconnected-exp
;  ?sf <- (sim-exp (rb-client-id ?client-id))
;  ?df <- (protobuf-client-disconnected ?client-id)
;  =>
;  (retract ?df)
;  (modify ?sf (rb-client-id 0))
;)
(defrule sim-read-light-pattern
  (declare (salience 10))
  (skill (name "ppgoto") (status FINAL))
  (simulation-is-running)
  (sim-machine-exp (name ?m) (lights $?lights))
  (goalmachine ?m)
  =>
  (assert (RobotinoLightInterface (id "Light_State") (red OFF) (yellow OFF) (green OFF) (ready FALSE)));gess at all
  ;assert facts for modification
  (foreach ?l ?lights
    (assert (sim-machine-light ?m ?l))
  )
)

(defrule match-light-on-interface-redon
  ?f <- (sim-machine-light ? RED-ON)
  ?i <- (RobotinoLightInterface (id "Light_State") (red ?) (yellow ?) (green ?) (ready FALSE))
  =>
  (retract ?f)
  (modify ?i (red ON))
)
(defrule match-light-on-interface-redblink
  ?f <- (sim-machine-light ? RED-BLINK)
  ?i <- (RobotinoLightInterface (id "Light_State") (red ?) (yellow ?) (green ?) (ready FALSE))
  =>
  (retract ?f)
  (modify ?i (red BLINKING))
)
(defrule match-light-on-interface-yellowon
  ?f <- (sim-machine-light ? YELLOW-ON)
  ?i <- (RobotinoLightInterface (id "Light_State") (red ?) (yellow ?) (green ?) (ready FALSE))
  =>
  (retract ?f)
  (modify ?i (yellow ON))
)
(defrule match-light-on-interface-yellowblink
  ?f <- (sim-machine-light ? YELLOW-BLINK)
  ?i <- (RobotinoLightInterface (id "Light_State") (red ?) (yellow ?) (green ?) (ready FALSE))
  =>
  (retract ?f)
  (modify ?i (yellow BLINKING))
)
(defrule match-light-on-interface-greenon
  ?f <- (sim-machine-light ? GREEN-ON)
  ?i <- (RobotinoLightInterface (id "Light_State") (red ?) (yellow ?) (green ?) (ready FALSE))
  =>
  (retract ?f)
  (modify ?i (green ON))
)
(defrule match-light-on-interface-greenblink
  ?f <- (sim-machine-light ? GREEN-BLINK)
  ?i <- (RobotinoLightInterface (id "Light_State") (red ?) (yellow ?) (green ?) (ready FALSE))
  =>
  (retract ?f)
  (modify ?i (green BLINKING))
)

(defrule match-light-on-interface-finished
  ?i <- (RobotinoLightInterface (id "Light_State") (red ?) (yellow ?) (green ?) (ready FALSE))
  (not (sim-machine-light ? ?))
  =>
  (modify ?i (ready TRUE))
)

;use recived via stream in simulation

;(defrule test-refbox-communication-exp
;  (protobuf-msg (type ?type))
;
;  =>
;  (printout t "Recieved Protobuf message from refbox with type " ?type crlf)
;)



(defrule sim-recv-MachineInfo-exp
  ?pf <- (protobuf-msg (type "llsf_msgs.MachineInfo") (ptr ?p) (rcvd-via STREAM))
  =>
  (foreach ?m (pb-field-list ?p "machines")
  (retract ?pf)
    ;(printout t "Processing machine " (pb-field-value ?m "name") crlf)
    (do-for-fact ((?machine sim-machine-exp))
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
        ;(printout t ?machine:name "|" ?new-mtype " modified - L " ?new-lights
	;	  " P " ?new-puck " " ?new-lw crlf)
        (modify ?machine (mtype ?new-mtype) (lights ?new-lights)
		(puck-id ?new-puck) (loaded-with ?new-lw))
      )
    )
  )
)
