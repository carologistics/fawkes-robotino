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

(deffunction mps-instruction-base-station (?peer ?machine ?team-color ?base-color)
  (bind ?instruction (mps-instruction-get-prepare-machine-msg ?machine ?team-color))
  (bind ?bs-inst (pb-create "llsf_msgs.PrepareInstructionBS"))
  (pb-set-field ?bs-inst "side" OUTPUT)
  (pb-set-field ?bs-inst "color" (sym-cat BASE_ ?base-color))
  (pb-set-field ?instruction "instruction_bs" ?bs-inst)
  (pb-broadcast ?peer ?instruction)
  (pb-destroy ?instruction)
  (printout t "Send instruction-base-station" crlf)
)

(deffunction mps-instruction-delivery-station (?peer ?machine ?team-color ?gate)
  (bind ?instruction (mps-instruction-get-prepare-machine-msg ?machine ?team-color))
  (bind ?ds-inst (pb-create "llsf_msgs.PrepareInstructionDS"))
  (pb-set-field ?ds-inst "gate" ?gate)
  (pb-set-field ?instruction "instruction_ds" ?ds-inst)
  (pb-broadcast ?peer ?instruction)
  (pb-destroy ?instruction)
  (printout t "Send instruction-delivery-station" crlf)
)

(deffunction mps-instruction-ring-station (?peer ?machine ?team-color ?ring-color)
  (bind ?instruction (mps-instruction-get-prepare-machine-msg ?machine ?team-color))
  (bind ?rs-inst (pb-create "llsf_msgs.PrepareInstructionRS"))
  (pb-set-field ?rs-inst "ring_color" (sym-cat RING_ ?ring-color))
  (pb-set-field ?instruction "instruction_rs" ?rs-inst)
  (pb-broadcast ?peer ?instruction)
  (pb-destroy ?instruction)
  (printout t "Send instruction-ring-station" crlf)
)

(deffunction mps-instruction-cap-station (?peer ?machine ?team-color ?operation)
  (if (not (member$ ?operation (create$ MOUNT_CAP RETRIEVE_CAP))) then
    (printout error "Cap-station operation " ?operation " is not valid!" crlf)
    (return)
  )
  (bind ?instruction (mps-instruction-get-prepare-machine-msg ?machine ?team-color))
  (bind ?cs-inst (pb-create "llsf_msgs.PrepareInstructionCS"))
  (pb-set-field ?cs-inst "operation" ?operation)
  (pb-set-field ?instruction "instruction_cs" ?cs-inst)
  (pb-broadcast ?peer ?instruction)
  (pb-destroy ?instruction)
  (printout t "Send instruction-cap-station" crlf)
)