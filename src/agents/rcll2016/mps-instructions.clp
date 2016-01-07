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

(deffunction mps-instruction-base-station (?machine ?team-color ?base-color)
  (bind ?instruction (mps-instruction-get-prepare-machine-msg ?machine ?team-color))
  (bind ?bs-inst (pb-create "llsf_msgs.PrepareInstructionBS"))
  (pb-set-field ?bs-inst "side" INPUT)
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
                                (lock ?lock))
  ;eigther no lock needed or already aquired
  (or (machine (name ?unused&:(eq ?lock NONE)))
      (wait-for-lock (res ?lock) (state use)))
  (machine (name ?machine) (mtype ?mtype))
  (team-color ?team-color)
  (peer-id private ?peer)
  =>
  (switch ?mtype
    (case BS
      then
      (bind ?instruction (mps-instruction-base-station ?machine ?team-color ?base-color)))
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
