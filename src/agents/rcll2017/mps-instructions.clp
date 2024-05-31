;---------------------------------------------------------------------------
;  mps-instruction.clp - Agent rules/funcitons to instruct mps machines
;
;  Created: Wed Apr 22 13:57:58 2015
;  Copyright  2015 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deffunction mps-instruction-get-prepare-machine-msg (?machine ?team-color)
  (bind ?instruction (pb-create "llsf_msgs.PrepareMachine"))
  (pb-set-field ?instruction "team_color" ?team-color)
  (pb-set-field ?instruction "machine" ?machine)
  (return ?instruction)
)

(deffunction mps-instruction-base-station (?machine ?team-color ?base-color ?side)
  (bind ?instruction (mps-instruction-get-prepare-machine-msg ?machine ?team-color))
  (bind ?bs-inst (pb-create "llsf_msgs.PrepareInstructionBS"))
  (pb-set-field ?bs-inst "side" ?side)
  (pb-set-field ?bs-inst "color" (sym-cat BASE_ ?base-color))
  (pb-set-field ?instruction "instruction_bs" ?bs-inst)
  (printout t ?base-color crlf)
  (return ?instruction)
)

(deffunction mps-instruction-delivery-station (?machine ?team-color ?gate)
  (bind ?instruction (mps-instruction-get-prepare-machine-msg ?machine ?team-color))
  (bind ?ds-inst (pb-create "llsf_msgs.PrepareInstructionDS"))
  (pb-set-field ?ds-inst "gate" ?gate)
  (pb-set-field ?instruction "instruction_ds" ?ds-inst)
  (return ?instruction)
)

(deffunction mps-instruction-ring-station (?machine ?team-color ?ring-color)
  (bind ?instruction (mps-instruction-get-prepare-machine-msg ?machine ?team-color))
  (bind ?rs-inst (pb-create "llsf_msgs.PrepareInstructionRS"))
  (pb-set-field ?rs-inst "ring_color" (sym-cat RING_ ?ring-color))
  (pb-set-field ?instruction "instruction_rs" ?rs-inst)
  (return ?instruction)
)

(deffunction mps-instruction-cap-station (?machine ?team-color ?operation)
  (if (not (member$ ?operation (create$ MOUNT_CAP RETRIEVE_CAP))) then
    (printout error "Cap-station operation " ?operation " is not valid!" crlf)
    (return)
  )
  (bind ?instruction (mps-instruction-get-prepare-machine-msg ?machine ?team-color))
  (bind ?cs-inst (pb-create "llsf_msgs.PrepareInstructionCS"))
  (pb-set-field ?cs-inst "operation" ?operation)
  (pb-set-field ?instruction "instruction_cs" ?cs-inst)
  (return ?instruction)
)


(defrule mps-instruction-send-instructions-periodically
  "Periodically send instructions to mps, so they know what to produce."
  (time $?now)
  ?mps-inst <- (mps-instruction (machine ?machine) (seq ?seq)
                                (timer $?t&:(timeout ?now ?t ?*MPS_INSTRUCTION-PERIOD*))
                                (base-color ?base-color) (gate ?gate)
                                (ring-color ?ring-color) (cs-operation ?cs-op)
                                (side ?side) (lock ?lock))
  ;eigther no lock needed or already aquired
  (or (test (eq ?lock NONE))
      (wait-for-lock (res ?lock) (state use)))
  (machine (name ?machine) (mtype ?mtype))
  (team-color ?team-color)
  (peer-id private ?peer)
  =>
  (switch ?mtype
    (case BS
      then
      (bind ?instruction (mps-instruction-base-station ?machine ?team-color ?base-color ?side)))
    (case CS
      then
      (bind ?instruction (mps-instruction-cap-station ?machine ?team-color ?cs-op)))
    (case DS
      then
      (bind ?instruction (mps-instruction-delivery-station ?machine ?team-color ?gate)))
    (case RS
      then
      (bind ?instruction (mps-instruction-ring-station ?machine ?team-color ?ring-color)))
  )
  (pb-broadcast ?peer ?instruction)
  (pb-destroy ?instruction)
  (printout t "Sent mps-instruction for " ?machine crlf)
  (modify ?mps-inst (timer ?now) (seq (+ 1 ?seq)))
)

(defrule mps-instruction-stop-sending-when-mps-is-prepared-instrucitons
  "Stop instructing the mps, if it is already prepared."
  (time $?now)
  ?mps-inst <- (mps-instruction (machine ?machine)
                                (seq ?seq&:(>= ?seq ?*MIN-TIMES-TO-SEND-MPS-INSTRUCTIONS*)))
  (machine (name ?machine) (state PROCESSING|PREPARED|READY-AT-OUTPUT))
  =>
  (printout t "Mps " ?machine " successfully instructed" crlf)
  (retract ?mps-inst)
)

(defrule mps-instruction-stop-sending-when-mps-is-broken
  "Stop instructing the mps, when the MPS is/became broken"
  (time $?now)
  ?mps-inst <- (mps-instruction (machine ?machine)
                                (seq ?seq&:(>= ?seq ?*MIN-TIMES-TO-SEND-MPS-INSTRUCTIONS*)))
  (machine (name ?machine) (state BROKEN))
  =>
  (printout error "Mps " ?machine " broke during instructed" crlf)
  (retract ?mps-inst)
)

(deffunction mps-instruction-reset-machine (?machine ?team-color)
 "Create ResetMachine Message"
  (bind ?reset-msg (pb-create "llsf_msgs.ResetMachine"))
  (pb-set-field ?reset-msg "team_color" ?team-color)
  (pb-set-field ?reset-msg "machine" ?machine)
(return ?reset-msg)
)

(defrule mps-instruction-send-reset-periodicaly
  "Periodically send reset to mps, until machine reseted."
  (time $?now)
  ?mps-reset <- (mps-reset (machine ?machine) (seq ?seq)
                           (timer $?t&:(timeout ?now ?t ?*MPS_RESET-PERIOD*))
			                     (lock ?lock))
  (or (test (eq ?lock NONE))
      (wait-for-lock (res ?lock) (state use)))
  (team-color ?team-color)
  (peer-id private ?peer)
  =>
  (bind ?instruction (mps-instruction-reset-machine ?machine ?team-color))
  (pb-broadcast ?peer ?instruction)
  (pb-destroy ?instruction)
  (printout t "Sent mps-reset for " ?machine crlf)
  (modify ?mps-reset (timer ?now) (seq (+ 1 ?seq)))
)

(defrule mps-instruction-stop-send-reset-message
  "Stop reset the mps, if machine is reseted (state BROKEN)."
  (time $?now)
  ?mps-reset <- (mps-reset (machine ?machine)
                                (seq ?seq&:(>= ?seq ?*MIN-TIMES-TO-SEND-MPS-RESET*)))
  (machine (name ?machine) (state BROKEN))
  =>
  (printout t "Mps " ?machine " successfully reseted" crlf)
  (retract ?mps-reset)
)
