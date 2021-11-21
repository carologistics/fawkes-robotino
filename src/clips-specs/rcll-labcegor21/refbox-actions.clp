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
  ?*ABORT-PREPARE-PERIOD* = 30.0

)

(defrule action-send-beacon-signal
  (time $?now)
  ?bs <- (wm-fact (key refbox beacon seq) (value ?seq))
  ?pa <- (plan-action (plan-id ?plan-id) (id ?id) (state PENDING)
                      (action-name send-beacon) (executable TRUE)
                      (param-names $?param-names)
                      (param-values $?param-values))
  (wm-fact (key config agent team)  (value ?team-name) )
  (wm-fact (id "/refbox/team-color") (value ?team-color&:(neq ?team-color nil)))
  (wm-fact (id "/refbox/comm/peer-id/public") (value ?peer) (type INT))
  =>
  (do-for-all-facts ((?robot wm-fact))
    (wm-key-prefix ?robot:key (create$ central agent robot))
    (modify ?bs (value (+ ?seq 1)))
    (bind ?robot-name (wm-key-arg ?robot:key r))
    (bind ?name-length (str-length (str-cat ?robot-name)))
    (bind ?robot-number (string-to-field (sub-string ?name-length ?name-length (str-cat ?robot-name))))
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

    (bind ?trans (create$ 0 0))
    (bind ?ori (create$ 0 0 0 1))
    (bind ?ptime ?now)
    (if (not (do-for-fact ((?pose Position3DInterface)) (eq ?pose:id (remote-if-id ?robot-name "Pose"))
                          (bind ?trans ?pose:translation)
                          (bind ?ptime ?pose:time)))
     then
      ; We do not have a correct Pose, fake it using the position of the machine we're at
      (do-for-fact ((?at wm-fact) (?node navgraph-node))
                   (and (wm-key-prefix ?at:key (create$ domain fact at args? r (sym-cat ?robot-name)))
                        (eq ?node:name (wm-fact-to-navgraph-node ?at:key)))
                   (bind ?trans ?node:pos)
      )
    )
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
  )
  (modify ?pa (state FINAL))
)


(defrule refbox-action-reset-mps-start
  (time $?now)
  ?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id) (id ?id)
                      (state PENDING)
                      (action-name ?action&reset-mps)
                      (executable TRUE)
                      (param-names $?param-names)
                      (param-values $?param-values))
  (wm-fact (key refbox team-color) (value ?team-color&:(neq ?team-color nil)))
  (wm-fact (key refbox comm peer-id private) (value ?peer-id))
  =>
  (bind ?mps (nth$ 1 ?param-values))
  (bind ?instruction_info (rest$ ?param-values))
  (printout t "Executing " ?action ?param-values crlf)
  (assert (metadata-reset-mps ?mps ?team-color ?peer-id ?instruction_info))
  (assert (timer (name (sym-cat reset- ?goal-id - ?plan-id - ?id -send-timer))
                 (time ?now) (seq 1)))
  (assert (timer (name (sym-cat reset- ?goal-id - ?plan-id - ?id -abort-timer))
                 (time ?now) (seq 1)))
  (modify ?pa (state RUNNING))
)


(defrule refbox-action-prepare-mps-start
  (time $?now)
  ?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id) (id ?id)
                      (state PENDING)
                      (action-name ?action&prepare-bs|
                                           prepare-cs|
                                           prepare-ds|
                                           prepare-rs|
                                           prepare-ss)
                      (executable TRUE)
                      (param-names $?param-names)
                      (param-values $?param-values))
  (wm-fact (key refbox team-color) (value ?team-color&:(neq ?team-color nil)))
  (wm-fact (key refbox comm peer-id private) (value ?peer-id))
  =>
  (bind ?mps (nth$ 1 ?param-values))
  (bind ?instruction_info (rest$ ?param-values))
  (printout t "Executing " ?action ?param-values crlf)
  (assert (metadata-prepare-mps ?mps ?team-color ?peer-id ?instruction_info))
  (assert (timer (name (sym-cat prepare- ?goal-id - ?plan-id
                                - ?id -send-timer))
          (time ?now) (seq 1)))
  (assert (timer (name (sym-cat prepare- ?goal-id - ?plan-id
                                - ?id -abort-timer))
          (time ?now) (seq 1)))
  (modify ?pa (state RUNNING))
)

(defrule refbox-action-reset-mps-send-signal
  (time $?now)
  ?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id) (id ?id)
                      (state RUNNING)
                      (action-name reset-mps)
                      (executable TRUE)
                      (param-names $?param-names)
                      (param-values $?param-values))
  ?st <- (timer (name ?n&:(eq ?n (sym-cat reset- ?goal-id - ?plan-id
                                          - ?id -send-timer)))
                (time $?t&:(timeout ?now ?t ?*PREPARE-PERIOD*))
                (seq ?seq))
  (domain-obj-is-of-type ?mps&:(eq ?mps (plan-action-arg m
                                                         ?param-names
                                                         ?param-values))
                         mps)
  (metadata-reset-mps ?mps ?team-color ?peer-id $?instruction_info)
  (wm-fact (key domain fact mps-type args? m ?mps t ?mps-type) (value TRUE))
  =>
  (bind ?machine-instruction (pb-create "llsf_msgs.ResetMachine"))
  (pb-set-field ?machine-instruction "team_color" ?team-color)
  (pb-set-field ?machine-instruction "machine" (str-cat ?mps))

  (pb-broadcast ?peer-id ?machine-instruction)
  (pb-destroy ?machine-instruction)
  (printout t "Sent Reset Msg for " ?mps  crlf)

  (modify ?st (time ?now) (seq (+ ?seq 1)))
)

(defrule refbox-action-mps-prepare-send-signal
  (time $?now)
  ?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id) (id ?id)
                      (state RUNNING)
                      (action-name prepare-bs|
                                   prepare-cs|
                                   prepare-ds|
                                   prepare-rs|
                                   prepare-ss)
                      (executable TRUE)
                      (param-names $?param-names)
                      (param-values $?param-values))
  (domain-obj-is-of-type ?mps&:(eq ?mps (plan-action-arg m
                                                         ?param-names
                                                         ?param-values))
                         mps)
  ?st <- (timer (name ?n&:(eq ?n (sym-cat prepare- ?goal-id - ?plan-id
                                          - ?id -send-timer)))
                (time $?t&:(timeout ?now ?t ?*PREPARE-PERIOD*))
                (seq ?seq))
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
        (bind ?order (nth$ 1 ?instruction_info))
        (bind ?order-id (float (string-to-field (sub-string 2 (length$ (str-cat ?order)) (str-cat ?order)))))
        (pb-set-field ?ds-inst "order_id" ?order-id)
        (pb-set-field ?machine-instruction "instruction_ds" ?ds-inst)
    )
    (case SS
      then
       (bind ?ss-inst (pb-create "llsf_msgs.PrepareInstructionSS"))
                (bind ?instruction (nth$ 2 ?instruction_info))
                (pb-set-field ?ss-inst "operation" ?instruction)
                (pb-set-field ?machine-instruction "instruction_ss" ?ss-inst)
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


(defrule refbox-action-reset-mps-final
  "Finalize the prepare action if the desired machine state was reached"
  (time $?now)
  ?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id) (id ?id)
                      (state RUNNING)
                      (action-name reset-mps)
                      (param-names $?param-names)
                      (param-values $?param-values))
  (domain-obj-is-of-type ?mps&:(eq ?mps (plan-action-arg m
                                                         ?param-names
                                                         ?param-values))
                         mps)
  ?st <- (timer (name ?nst&:(eq ?nst (sym-cat reset- ?goal-id - ?plan-id
                                      - ?id -send-timer))))
  ?at <- (timer (name ?nat&:(eq ?nat (sym-cat reset- ?goal-id - ?plan-id
                                      - ?id -abort-timer))))
  (wm-fact (key domain fact mps-state args? m ?mps s BROKEN))
  =>
  (printout t "Action Reset " ?mps " is final" crlf)
  (retract ?st ?at)
  (modify ?pa (state EXECUTION-SUCCEEDED))
)

(defrule refbox-action-prepare-mps-final
  "Finalize the prepare action if the desired machine state was reached"
  (time $?now)
  ?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id) (id ?id)
                      (state RUNNING)
                      (action-name prepare-bs|
                                   prepare-cs|
                                   prepare-ds|
                                   prepare-rs|
                                   prepare-ss)
                      (param-names $?param-names)
                      (param-values $?param-values))
  (domain-obj-is-of-type ?mps&:(eq ?mps (plan-action-arg m
                                                         ?param-names
                                                         ?param-values))
                         mps)
  ?st <- (timer (name ?nst&:(eq ?nst
                               (sym-cat prepare- ?goal-id - ?plan-id
                                        - ?id -send-timer))))
  ?at <- (timer (name ?nat&:(eq ?nat
                               (sym-cat prepare- ?goal-id - ?plan-id
                                        - ?id -abort-timer))))
  ?md <- (metadata-prepare-mps ?mps $?date)
  (wm-fact (key domain fact mps-state args? m ?mps s READY-AT-OUTPUT|
                                                     WAIT-IDLE|
                                                     PROCESSING|
                                                     PROCESSED|
                                                     PREPARED))
  =>
  (printout t "Action Prepare " ?mps " is final" crlf)
  (retract ?st ?at ?md)
  (modify ?pa (state EXECUTION-SUCCEEDED))
)

(defrule refbox-action-reset-mps-abort
  "Abort preparing and fail the action if took too long"
  (time $?now)
  ?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id) (id ?id)
                      (state RUNNING)
                      (action-name reset-mps)
                      (param-names $?param-names)
                      (param-values $?param-values))
  (domain-obj-is-of-type ?mps&:(eq ?mps (plan-action-arg m
                                                         ?param-names
                                                         ?param-values))
                         mps)
  ?at <- (timer (name ?nat&:(eq ?nat
                               (sym-cat reset- ?goal-id - ?plan-id
                                        - ?id -abort-timer)))
			       (time $?t&:(timeout ?now ?t ?*ABORT-PREPARE-PERIOD*))
			       (seq ?seq))
  ?st <- (timer (name ?nst&:(eq ?nst
                                (sym-cat reset- ?goal-id - ?plan-id
                                         - ?id -send-timer))))
  (not (wm-fact (key domain fact mps-state args? m ?mps s BROKEN)))
  =>
  (printout t "Action Reset " ?mps " is Aborted" crlf)
  (retract ?st ?at)
  (modify ?pa (state EXECUTION-FAILED))
)

(defrule refbox-action-prepare-mps-abort-on-broken
  "Abort preparing if the mps got broken"
  ?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id) (id ?id)
                      (state RUNNING)
                      (action-name prepare-bs|
                                   prepare-cs|
                                   prepare-ds|
                                   prepare-rs|
                                   prepare-ss)
                      (param-names $?param-names)
                      (param-values $?param-values))
  (domain-obj-is-of-type ?mps&:(eq ?mps (plan-action-arg m
                                                         ?param-names
                                                         ?param-values))
                         mps)
  ?st <- (timer (name ?nst&:(eq ?nst
                               (sym-cat prepare- ?goal-id - ?plan-id
                                        - ?id -send-timer))))
  ?at <- (timer (name ?nat&:(eq ?nat
                               (sym-cat prepare- ?goal-id - ?plan-id
                                        - ?id -abort-timer))))
  ?md <- (metadata-prepare-mps ?mps $?date)
  (wm-fact (key domain fact mps-state args? m ?mps s BROKEN))
  =>
  (printout t "Action Prepare " ?mps " is Aborted because mps is broken" crlf)
  (retract ?st ?md ?at)
  (modify ?pa (state EXECUTION-FAILED))
)

(defrule refbox-action-prepare-mps-abort
  "Abort preparing and fail the action if took too long"
  (time $?now)
  ?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id) (id ?id)
                      (state RUNNING)
                      (action-name prepare-bs|
                                   prepare-cs|
                                   prepare-ds|
                                   prepare-rs|
                                   prepare-ss)
                      (param-names $?param-names)
                      (param-values $?param-values))
  (domain-obj-is-of-type ?mps&:(eq ?mps (plan-action-arg m
                                                         ?param-names
                                                         ?param-values))
                         mps)
  ?at <- (timer (name ?nat&:(eq ?nat
                                (sym-cat prepare- ?goal-id - ?plan-id
                                         - ?id -abort-timer)))
	        (time $?t&:(timeout ?now ?t ?*ABORT-PREPARE-PERIOD*))
                (seq ?seq))
  ?st <- (timer (name ?nst&:(eq ?nst
                                (sym-cat prepare- ?goal-id - ?plan-id
                                         - ?id -send-timer))))
  ?md <- (metadata-prepare-mps ?mps $?date)
  (not (wm-fact (key domain fact mps-state args? m ?mps s READY-AT-OUTPUT|
                                                          PROCESSING|
                                                          PROCESSED|
                                                          PREPARED)))
  =>
  (printout t "Action Prepare " ?mps " is Aborted" crlf)
  (retract ?st ?md ?at)
  (modify ?pa (state EXECUTION-FAILED))
)
