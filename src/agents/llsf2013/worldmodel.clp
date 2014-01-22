
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
  (printout t "***** Received MachineInfo *****" crlf)
  (assert (machine-info-received))
  (foreach ?m (pb-field-list ?p "machines")
    (do-for-fact ((?machine machine))
      (eq ?machine:name (sym-cat (pb-field-value ?m "name")))
      (printout t "Machine " (pb-field-value ?m "name") " is of type "
		(pb-field-value ?m "type") crlf)
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

(defrule wm-store-lights
  (declare (salience ?*PRIORITY-WM*))
  ?rf <- (RobotinoLightInterface (id "Light determined") (ready TRUE)
				 (red ?red) (green ?green) (yellow ?yellow))
  =>
  (retract ?rf)
  ; remove all possibly existing last-lights facts
  (delayed-do-for-all-facts ((?ll last-lights)) TRUE (retract ?ll))
  (bind ?lights (create$))
  (if (neq ?red OFF) then (bind ?lights (create$ ?lights (sym-cat RED- ?red))))
  (if (neq ?green OFF) then (bind ?lights (create$ ?lights (sym-cat GREEN- ?green))))
  (if (neq ?yellow OFF) then (bind ?lights (create$ ?lights (sym-cat YELLOW- ?yellow))))
  (assert (last-lights ?lights))
  (printout t "***** Lights 5 " ?lights crlf)
)

(defrule wm-goto-failed
  (declare (salience ?*PRIORITY-WM-LOW*))
  (state GOTO-FAILED)
  ?tf <- (goto-target ?name)
  ?hf <- (holding ?)
  =>
  (retract ?tf ?hf)
  (assert (holding NONE))
  (printout t "Production failed at " ?name crlf) 
)

(defrule wm-goto-failed-p3
  (declare (salience ?*PRIORITY-WM-MID*))
  (state GOTO-FINAL)
  ?tf <- (goto-target ?name)
  ?hf <- (holding ?)
  ?mf <- (machine (name ?name) (mtype T5))
  =>
  (retract ?tf ?hf)
  (assert (holding P3))
)

(defrule wm-goto-failed-p1p2
  (declare (salience ?*PRIORITY-WM-MID*))
  (state GOTO-FINAL)
  ?tf <- (goto-target ?name)
  ?hf <- (holding ?)
  ?mf <- (machine (name ?name) (mtype T3|T4) (output ?output) (loaded-with $?lw&:(> (length$ ?lw) 1)) )
  =>
  (retract ?tf ?hf)
  (assert (holding ?output))
  (modify ?mf (loaded-with))
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
  ?hf <- (holding ?)
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
  ?lf <- (lights YELLOW-BLINKING)
  ?mf <- (machine (name ?name) (mtype ?mtype))
  ?hf <- (holding ?)
  =>
  (printout t "Production invalid at " ?name "|" ?mtype crlf) 
  (retract ?lf ?tf ?hf)
  (assert (holding NONE))
  (if (not (or (eq ?mtype T5) (eq ?mtype T1)))
    then
    ;forget machine and choose an other one
    (strat-allow-all ?mtype)
  )
  (delayed-do-for-all-facts ((?machine machine)) (eq ?machine:name ?name) 
    (modify ?machine (allowed FALSE))
  )
)

(defrule wm-proc-delivered
  (declare (salience ?*PRIORITY-WM*))
  (state GOTO-FINAL)
  ?tf <- (goto-target deliver)
  ?hf <- (holding ?was-holding)
  ;?lf <- (lights $?)
  =>
  (retract ?hf ?tf)
  (assert (holding NONE))
  (printout t "Delivered " ?was-holding crlf)
)

(defrule wm-proc-wtf
  (declare (salience ?*PRIORITY-WM-LOW*))
  (state GOTO-FINAL)
  ?tf <- (goto-target ?name)
  ?lf <- (lights $?)
  ?hf <- (holding ?)
  ?mf <- (machine (name ?name) (mtype ?mtype))
  =>
  (printout warn "WTF? Unhandled light code at " ?name "|" ?mtype crlf) 
  (retract ?lf ?tf ?hf)
  (assert (holding NONE))
  (if (not (or (eq ?mtype T5) (eq ?mtype T1)))
    then
    ;forget machine and choose an other one
    (strat-allow-all ?mtype)
    (modify ?mf (allowed FALSE))
  )
)
