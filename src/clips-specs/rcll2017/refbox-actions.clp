;---------------------------------------------------------------------------
;  custom-actions.clp - Actions that interact with the RefBox
;
;  Created:: Mon 10 Jan 2017 13:21:21 CET
;  Copyright  2017  Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file in the doc directory.
;---------------------------------------------------------------------------

(defglobal
  ; network sending periods; seconds
  ?*BEACON-PERIOD* = 1.0
  ?*PREPARE-PERIOD* = 1.0
  ?*ABORT-PREPARE-PERIOD* = 10.0

)

(defrule action-send-beacon-signal
	(time $?now)
  ?bs <- (wm-fact (key refbox beacon seq) (value ?seq))
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (status PENDING)
                      (action-name send-beacon) (executable TRUE)
                      (param-names $?param-names)
                      (param-values $?param-values))
  	(wm-fact (id "/config/rcll/team-name")  (value ?team-name) )
  	(wm-fact (id "/config/rcll/robot-name")  (value ?robot-name) )
  	(wm-fact (id "/config/rcll/robot-number")  (value ?robot-number) )
  	(wm-fact (id "/refbox/team-color") (value ?team-color&:(neq ?team-color nil)))
  	(wm-fact (id "/refbox/comm/peer-id/public") (value ?peer) (type INT))
  	(Position3DInterface (id "Pose") (translation $?trans) (rotation $?ori) (time $?ptime))
  	=>
  (modify ?bs (value (+ ?seq 1)))
	(if (debug 3) then (printout t "Sending beacon" crlf))
	(bind ?beacon (pb-create "llsf_msgs.BeaconSignal"))
	(bind ?beacon-time (pb-field-value ?beacon "time"))
	(pb-set-field ?beacon-time "sec" (nth$ 1 ?now))
	(pb-set-field ?beacon-time "nsec" (* (nth$ 2 ?now) 1000))
	(pb-set-field ?beacon "time" ?beacon-time) ; destroys ?beacon-time!
	(pb-set-field ?beacon "seq" ?seq)
	(pb-set-field ?beacon "team_name" ?team-name)
	(pb-set-field ?beacon "peer_name" ?robot-name)
	(pb-set-field ?beacon "team_color" ?team-color)
	(pb-set-field ?beacon "number" ?robot-number)

	(bind ?beacon-pose (pb-field-value ?beacon "pose"))
	(pb-set-field ?beacon-pose "x" (nth$ 1 ?trans))
	(pb-set-field ?beacon-pose "y" (nth$ 2 ?trans))
	(pb-set-field ?beacon-pose "ori" (tf-yaw-from-quat ?ori))
	(bind ?beacon-pose-time (pb-field-value ?beacon-pose "timestamp"))
	(pb-set-field ?beacon-pose-time "sec" (nth$ 1 ?ptime))
	(pb-set-field ?beacon-pose-time "nsec" (* (nth$ 2 ?ptime) 1000))
	(pb-set-field ?beacon-pose "timestamp" ?beacon-pose-time)
	(pb-set-field ?beacon "pose" ?beacon-pose)

	(pb-broadcast ?peer ?beacon)
	(pb-destroy ?beacon)
	(modify ?pa (status FINAL))
)


(defrule refbox-action-prepare-mps-start
	(time $?now)
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (status PENDING)
	                      (action-name prepare-bs|prepare-cs|prepare-ds|prepare-rs)
	                      (executable TRUE)
	                      (param-names $?param-names)
	                      (param-values $?param-values))
	(wm-fact (key refbox team-color) (value ?team-color&:(neq ?team-color nil)))
	(wm-fact (key refbox comm peer-id private) (value ?peer-id))
	=>
	(bind ?mps (nth$ 1 ?param-values))
	(bind ?instruction_info (rest$ ?param-values))
	(printout t "Action Prepare " ?mps " Started"  crlf)
	(assert (metadata-prepare-mps ?mps ?team-color ?peer-id ?instruction_info))
	(assert (timer (name prepare-mps-send-timer) (time ?now) (seq 1)))
	(assert (timer (name prepare-mps-abort-timer) (time ?now) (seq 1)))
	(modify ?pa (status RUNNING))
)

(defrule refbox-action-mps-prepare-send-signal
	(time $?now)
	?st <- (timer (name prepare-mps-send-timer) 
					(time $?t&:(timeout ?now ?t ?*PREPARE-PERIOD*))
					(seq ?seq))
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (status RUNNING)
	                      (action-name prepare-bs|prepare-cs|prepare-ds|prepare-rs)
	                      (executable TRUE)
	                      (param-names $?param-names)
	                      (param-values $?param-values))
	(metadata-prepare-mps ?mps ?team-color ?peer-id $?instruction_info)
	(wm-fact (key domain fact mps-type args? m ?mps t ?mps-type) (value TRUE))
	=>
	(bind ?machine-instruction (pb-create "llsf_msgs.PrepareMachine"))
	(pb-set-field ?machine-instruction "team_color" ?team-color)
	(pb-set-field ?machine-instruction "machine" (str-cat ?mps))
	
	(switch ?mps-type
    (case BS
      then
		(bind ?bs-inst (pb-create "llsf_msgs.PrepareInstructionBS"))
	  	(pb-set-field ?bs-inst "side" (nth$ 1 ?instruction_info))
	  	(pb-set-field ?bs-inst "color" (nth$ 2 ?instruction_info))
	  	(pb-set-field ?machine-instruction "instruction_bs" ?bs-inst)
	  	)
    (case CS
      then
      	(bind ?cs-inst (pb-create "llsf_msgs.PrepareInstructionCS"))
  		(pb-set-field ?cs-inst "operation" (nth$ 1 ?instruction_info))
  		(pb-set-field ?machine-instruction "instruction_cs" ?cs-inst)
  		)
    (case DS
      then
    	(bind ?ds-inst (pb-create "llsf_msgs.PrepareInstructionDS"))
  		(bind ?gate-sym (nth$ 1 ?instruction_info))
  		(if (eq ?gate-sym GATE-1) then (bind ?gate 1) )
  		(if (eq ?gate-sym GATE-2) then (bind ?gate 2) )
  		(if (eq ?gate-sym GATE-3) then (bind ?gate 3) )
  		(pb-set-field ?ds-inst "gate" ?gate)
  		(pb-set-field ?machine-instruction "instruction_ds" ?ds-inst)
  		)
    (case RS
      then
        (bind ?rs-inst (pb-create "llsf_msgs.PrepareInstructionRS"))
    	(pb-set-field ?rs-inst "ring_color" (nth$ 1 ?instruction_info) )
  		(pb-set-field ?machine-instruction "instruction_rs" ?rs-inst)
  		)
  	)

  	(pb-broadcast ?peer-id ?machine-instruction)
  	(pb-destroy ?machine-instruction)
  	(printout t "Sent Prepare Msg for " ?mps " with " ?instruction_info  crlf)

	(modify ?st (time ?now) (seq (+ ?seq 1)))
)

(defrule refbox-action-bs-prepare-final
	"Finalize the prepare action if the desired machine state was reached"
	(time $?now)
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (status RUNNING)
	                      (action-name prepare-bs) (executable TRUE)
	                      (param-names m side bc)
	                      (param-values ?mps ?side ?base-color))
	?st <- (timer (name prepare-bs-send-timer))
	?at <- (timer (name prepare-bs-abort-timer))
	?md <- (metadata-prepare-bs $?date)
	(wm-fact (key domain fact mps-state args? m ?mps s READY-AT-OUTPUT|PROCESSING))
	=>
	(retract ?st ?md)
	(modify ?pa (status EXECUTED))
)


(defrule refbox-action-bs-prepare-abort
	"Abort preparing and fail the action if took too long"
	(time $?now)
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (status RUNNING)
	                      (action-name prepare-bs) (executable TRUE)
	                      (param-names m side bc)
	                      (param-values ?mps ?side ?base-color))
	?at <- (timer (name prepare-bs-abort-timer) (time $?t&:(timeout ?now ?t ?*ABORT-PREPARE-PERIOD*)) (seq ?seq))
	?st <- (timer (name prepare-bs-send-timer))
	?md <- (metadata-prepare-bs $?date)
	(not (wm-fact (key domain fact mps-state args? m ?mps s READY-AT-OUTPUT|PROCESSING)))
	=>
	(retract ?st ?md ?at)
	(modify ?pa (status FAILED))
)