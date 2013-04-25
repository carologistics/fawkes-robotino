
;---------------------------------------------------------------------------
;  worldmodel.clp - Robotino agent -- world model update rules
;
;  Created: Sat Jun 16 18:50:53 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; knowledge evaluation request
(defrule wm-recv-MachineInfo
  (protobuf-msg (type "llsf_msgs.MachineInfo") (ptr ?p) (rcvd-via BROADCAST))
  (not (machine-info-received))
  =>
  (assert (machine-info-received))
  (foreach ?m (pb-field-list ?p "machines")
    (do-for-fact ((?machine machine))
      (eq ?machine:name (sym-cat (pb-field-value ?m "name")))
      (modify ?machine (mtype (sym-cat (pb-field-value ?m "type")))
	      (output (sym-cat (pb-field-value ?m "output"))))
    )
  )
)

(defrule wm-get-s0-final
  (declare (salience ?*PRIORITY-WM*))
  (state GET-S0-FINAL)
  ?hf <- (holding NONE)
  =>
  (retract ?hf)
  (assert (holding S0))
)

(defrule wm-goto-light
  (declare (salience ?*PRIORITY-WM*))
  (state GOTO-FINAL)
  (not (lights))
  (RobotinoLightInterface (id "Light detected")
			  (ready TRUE) (visibility_history ?vh&:(> ?vh 5))
			  (red ?red) (green ?green) (yellow ?yellow))
  =>
  (bind ?lights (create$))
  (if (neq ?red OFF) then (bind ?lights (create$ ?lights (sym-cat RED- ?red))))
  (if (neq ?green OFF) then (bind ?lights (create$ ?lights (sym-cat GREEN- ?green))))
  (if (neq ?yellow OFF) then (bind ?lights (create$ ?lights (sym-cat YELLOW- ?yellow))))
  (assert (lights ?lights))
)

(defrule wm-goto-proc-complete
  (declare (salience ?*PRIORITY-WM*))
  (state GOTO-FINAL)
  ?tf <- (goto-target ?name)
  ?hf <- (holding ?any)
  ?lf <- (lights GREEN-ON)
  ?mf <- (machine (name ?name) (mtype ?mtype) (output ?output))
  =>
  (retract ?tf ?hf ?lf)
  (assert (holding ?output))
  (printout t "Production completed at " ?name "|" ?mtype crlf) 
  (modify ?mf (loaded-with))
)

(defrule wm-proc-inprogress
  (declare (salience ?*PRIORITY-WM*))
  (state GOTO-FINAL)
  ?tf <- (goto-target ?name)
  ?hf <- (holding ?was-holding)
  ?lf <- (lights YELLOW-ON)
  ?mf <- (machine (name ?name) (mtype ?mtype) (loaded-with $?lw))
  =>
  (retract ?hf ?lf ?tf)
  (assert (holding NONE))
  (printout t "Production in progress at " ?name "|" ?mtype crlf) 
  (modify ?mf (loaded-with (create$ ?lw ?was-holding)))
)

(defrule wm-proc-invalid
  (declare (salience ?*PRIORITY-WM*))
  (state GOTO-FINAL)
  ?tf <- (goto-target ?name)
  ?lf <- (lights YELLOW-BLINK)
  (machine (name ?name) (mtype ?mtype))
  =>
  (printout t "Production invalid at " ?name "|" ?mtype crlf) 
  (retract ?lf ?tf)
)

(defrule wm-proc-delivered
  (declare (salience ?*PRIORITY-WM*))
  (state GOTO-FINAL)
  ?tf <- (goto-target deliver)
  ?hf <- (holding ?was-holding)
  ?lf <- (lights GREEN-ON YELLOW-ON RED-ON)
  =>
  (retract ?hf ?lf ?tf)
  (assert (holding NONE))
  (printout t "Delivered " ?was-holding crlf)
)

(defrule wm-proc-wtf
  (declare (salience ?*PRIORITY-WM*))
  (state GOTO-FINAL)
  ?tf <- (goto-target ?name)
  ?lf <- (lights ~YELLOW-BLINK&~GREEN-ON&~YELLOW-ON)
  (machine (name ?name) (mtype ?mtype))
  =>
  (printout t "Unknown light code at " ?name "|" ?mtype crlf) 
  (retract ?lf)
)
