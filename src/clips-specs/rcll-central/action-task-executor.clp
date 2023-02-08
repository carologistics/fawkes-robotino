(defrule action-task-connect-receiver-of-sim
  "Enable peer connection to the simulator"
  ; (declare (salience ?*PRIORITY-LOW*))
  (executive-init)
  (wm-fact (id "/config/rcll/sim-peer-address") (value ?peer-address))
  (wm-fact (id "/config/rcll/sim-peer-recv-port") (value ?peer-recv-port))
  (not (wm-fact (id "/simulator/comm/sim-peer-enabled") (value TRUE)))
  =>
  (printout t "Enabling local peer (public)" crlf)
  (bind ?peer-id (pb-peer-create-local ?peer-address ?peer-recv-port ?peer-recv-port))
  (assert (wm-fact (id "/simulator/comm/peer-enabled") (value TRUE) (type BOOL))
          (wm-fact (id "/simulator/comm/peer-id/public") (value ?peer-id) (type INT))
   )
)

(defrule action-task-disconnect-from-sim
  "Disable the local peer connection on finalize"
  (executive-finalize)
  ?pe <- (wm-fact (id "/simulator/comm/peer-enabled") (value TRUE))
  (wm-fact (id "/simulator/comm/peer-id/public") (value ?peer-id) (type INT))
  =>
  (printout t "Closing local peer (public)" crlf)
  (pb-peer-destroy ?peer-id)
  (modify ?pe (value FALSE))
)

(defrule action-task-register
  (not (action-task-executor-enable))
  =>
  (assert (action-task-executor-enable (name move))
          (action-task-executor-enable (name go-wait))
          (action-task-executor-enable (name wp-get-shelf))
          (action-task-executor-enable (name wp-get))
          (action-task-executor-enable (name wp-put-slide-cc))
          (action-task-executor-enable (name wp-put))
  )
)

(defrule action-task-open-robot-peer
  (declare (salience 1000))
  ?pf <- (protobuf-msg (type "llsf_msgs.BeaconSignal") (ptr ?p) (rcvd-from ?host ?robot-rcvd) (client-id ?client-id))
  (wm-fact (id "/simulator/comm/peer-id/public") (value ?client-id) (type INT))
  (wm-fact (id "/refbox/team-color") (value ?team-color&:(eq (pb-field-value ?p "team_color") ?team-color)))
  (wm-fact (key central agent robot args? r ?robot&:(str-index (str-cat (pb-field-value ?p "number")) ?robot)))
  (not (wm-fact (key simulator comm peer-id ?robot)))
  =>
  (printout error "Created send peer for robot " ?robot crlf)
  (bind ?peer-id (pb-peer-create-local ?host ?robot-rcvd 2018))
  (assert (wm-fact (key simulator comm peer-enabled ?robot) (value TRUE) (type BOOL))
          (wm-fact (key simulator comm peer-id ?robot) (value ?peer-id) (type INT))
   )
  (pb-destroy ?p)
  (retract ?pf)
)

(defrule action-task-set-on-waiting
  (declare (salience 1000))
  ?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id) (state PENDING)
                      (action-name ?action-name) (executable TRUE))
  (action-task-executor-enable (name ?action-name))
  (wm-fact (id "/simulator/comm/peer-enabled") (value TRUE))
  ;(skiller-control (skiller ?skiller) (acquired TRUE))
  =>
  (modify ?pa (state WAITING))
)

(defrule action-task-send-command
  ?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id)
           (state WAITING) (action-name ?action-name))
  (action-task-executor-enable (name ?action-name))
  ?at <- (refbox-agent-task (task-id ?task-seq) (robot ?robot))
  (wm-fact (key refbox robot task seq args? r ?robot) (value ?task-seq))
  (wm-fact (key simulator comm peer-enabled ?robot) (value TRUE) (type BOOL))
  (wm-fact (key simulator comm peer-id ?robot) (value ?peer-id) (type INT))
  (wm-fact (id "/refbox/team-color") (value ?team-color&:(neq ?team-color nil)))
  =>
  (bind ?task-msg (create-task-msg ?at ?team-color))
  (if ?task-msg
   then
    (pb-send ?peer-id ?task-msg)
    (pb-destroy ?task-msg)
    (modify ?pa (state RUNNING))
   else
    (modify ?pa (state FAILED) (error-msg (str-cat "Failed to create agent-task message for " ?action-name)))
  )
)

(defrule agent-task-recv-AgentTask-for-running-action
  ?pf <- (protobuf-msg (type "llsf_msgs.AgentTask") (ptr ?task-msg))
  ?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id)
           (state RUNNING) (action-name ?action-name))
  (action-task-executor-enable (name ?action-name))
  (refbox-agent-task (task-id ?task-seq) (robot ?robot)
    (goal-id ?goal-id) (plan-id ?plan-id) (action-id ?id)
    (outcome ?outcome)
  )
  (wm-fact (key refbox robot task seq args? r ?robot) (value ?task-seq))
  =>
  (bind ?task (pb-field-value ?task-msg "task_id"))
  (bind ?robot-num (pb-field-value ?task-msg "robot_id"))
  (bind ?team-col (pb-field-value ?task-msg "team_color"))
  (bind ?outcome EXECUTION-FAILED)
  (if (pb-has-field ?task-msg "cancelled") then
    (bind ?cancelled (pb-field-value ?task-msg "cancelled"))
    (if ?cancelled then (printout warn "Agent Task for " ?action-name " got cancelled" crlf))
  )
  (if (pb-has-field ?task-msg "successful") then
    (bind ?successful (pb-field-value ?task-msg "successful"))
    (if ?successful then (bind ?outcome EXECUTION-SUCCEEDED))
  )
  (modify ?pa (state ?outcome))
  (printout t "Agent task " ?task " of robot " ?robot-num  " outcome " ?outcome crlf)
  ; TODO: terminate the action that is done
  (retract ?pf)
)

(defrule agent-task-recv-AgentTask-no-action
  (declare (salience ?*SALIENCE-LOW*))
  ?pf <- (protobuf-msg (type "llsf_msgs.AgentTask") (ptr ?task-msg))
  =>
  (printout error "received AgentTask message without action" crlf)
)
