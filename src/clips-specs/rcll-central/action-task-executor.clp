(deftemplate protobuf-peer
  (slot name (type SYMBOL))
  (slot peer-id (type INTEGER))
)

(defrule action-task-connect-receiver-of-sim
  "Enable peer connection to the simulator"
  (confval (path "/rcll-simulator/enabled") (value TRUE))
  (confval (path "/rcll-simulator/host") (value ?peer-address))
  (confval (path "/rcll-simulator/robot-recv-ports") (is-list TRUE) (list-value $?recv-ports))
  (confval (path "/rcll-simulator/robot-send-ports") (is-list TRUE) (list-value $?send-ports))
  (not (protobuf-peer (name robot1)))
  (not (executive-finalize))
  =>
  (printout info "Enabling robot simulation peers" crlf)
  (if (<> (length$ ?recv-ports) (length$ ?send-ports)) then
    (printout error "Expected number or recv ports to be equal to send ports for simulator robots (" (length$ ?recv-ports) " != "(length$ ?send-ports) ")" crlf)
   else
    (loop-for-count (?i (length$ ?recv-ports)) do
      (bind ?peer-id (pb-peer-create-local ?peer-address (nth$ ?i ?send-ports)(nth$ ?i ?recv-ports)))
      (assert (protobuf-peer (name (sym-cat "robot" ?i)) (peer-id ?peer-id)))
    )
  )
)

(defrule action-task-disconnect-from-sim
  "Disable the local peer connection on finalize"
  (executive-finalize)
  ?pp <- (protobuf-peer (name ?robot) (peer-id ?peer-id))
  =>
  (printout t "Closing local simulation peer "  ?robot crlf)
  (pb-peer-destroy ?peer-id)
  (retract ?pp)
)

(defrule action-task-register
  (not (action-task-executor-enable))
  (confval (path "/rcll-simulator/enabled") (value TRUE))
  =>
  (assert (action-task-executor-enable (name move))
          (action-task-executor-enable (name go-wait))
          (action-task-executor-enable (name enter-field))
          (action-task-executor-enable (name explore-and-turn))
          (action-task-executor-enable (name wp-get-shelf))
          (action-task-executor-enable (name wp-get))
          (action-task-executor-enable (name wp-put-slide-cc))
          (action-task-executor-enable (name wp-put))
  )
)

(defrule action-task-set-on-waiting
" Override skiller invocation for actions with an action-task mapping by
  putting them in state WAITING before the skiller is invoked.
"
  (declare (salience ?*SALIENCE-HIGH*))
  ?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id) (state PENDING)
                      (action-name ?action-name) (executable TRUE))
  (action-task-executor-enable (name ?action-name))
  =>
  (modify ?pa (state WAITING))
)

(defrule action-task-skip-non-mapped-actions
" We dont sent agent tasks for enter-field or explore-and-turn
"
  (declare (salience ?*SALIENCE-LOW*))
  ?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id) (state WAITING)
                      (action-name ?action-name&:(member$ ?action-name (create$ explore-and-turn enter-field))))
  (action-task-executor-enable (name ?action-name))
  =>
  (modify ?pa (state EXECUTION-SUCCEEDED))
)

(defrule action-task-skip-trivial-move
" We dont sent agent tasks for trivial moves...
"
  (declare (salience ?*SALIENCE-LOW*))
  ?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id) (state WAITING)
                      (action-name move) (param-values ?robot ?source ?source-side ?source ?source-side))
  (action-task-executor-enable (name move))
  =>
  (modify ?pa (state EXECUTION-SUCCEEDED))
)

(defrule action-task-skip-trivial-go-wait
" We dont sent agent tasks for trivial moves...
"
  (declare (salience ?*SALIENCE-LOW*))
  ?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id) (state WAITING)
                      (action-name go-wait) (param-values ?robot ?source ?source-side ?source))
  (action-task-executor-enable (name go-wait))
  =>
  (modify ?pa (state EXECUTION-SUCCEEDED))
)

(defrule action-task-send-command
" Create an AgentTask protobuf message and send it to the simulator peer.
"
  ?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id)
           (state WAITING) (action-name ?action-name))
  (action-task-executor-enable (name ?action-name))
  ?at <- (refbox-agent-task (task-id ?task-seq) (robot ?robot) (goal-id ?goal-id) (plan-id ?plan-id) (action-id ?id))
  (wm-fact (key refbox robot task seq args? r ?robot) (value ?task-seq))
  (protobuf-peer (name ?robot) (peer-id ?peer-id))
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
  (test (eq (string-to-field (sub-string (str-length ?robot) (str-length ?robot) ?robot))
  (pb-field-value ?task-msg "robot_id")))
  (wm-fact (key refbox robot task seq args? r ?robot) (value ?task-seq))
  =>
  (bind ?task (pb-field-value ?task-msg "task_id"))
  (if (eq ?task ?task-seq) then
  (bind ?robot-num (pb-field-value ?task-msg "robot_id"))
  (bind ?team-col (pb-field-value ?task-msg "team_color"))
  (bind ?outcome UNKNOWN)
  (if (pb-has-field ?task-msg "cancelled") then
    (bind ?cancelled (pb-field-value ?task-msg "cancelled"))
    (if ?cancelled then (printout warn "Agent Task for " ?action-name " got cancelled" crlf))
  )
  (if (pb-has-field ?task-msg "successful") then
    (bind ?successful (pb-field-value ?task-msg "successful"))
    (if ?successful then
      (bind ?outcome EXECUTION-SUCCEEDED)
     else
      (bind ?outcome EXECUTION-FAILED)
    )
  )
  (if (neq ?outcome UNKNOWN) then
    (modify ?pa (state ?outcome))
  )
   else (if (> ?task ?task-seq) then
     (bind ?robot-num (pb-field-value ?task-msg "robot_id"))
     (bind ?team-col (pb-field-value ?task-msg "team_color"))
     (printout warn "Received feedback for futur task!" crlf)
	 (printout t ?goal-id " " ?plan-id " " ?id " " ?action-name crlf)
	 (printout t ?task-seq " " ?robot " " ?outcome crlf)
	 (printout t  ?task " " ?robot-num " " ?team-col crlf)
	 )
  )
  (retract ?pf)
)

(defrule agent-task-recv-AgentTask-no-action
  (declare (salience ?*SALIENCE-LOW*))
  ?pf <- (protobuf-msg (type "llsf_msgs.AgentTask") (ptr ?task-msg))
  =>
  (retract ?pf)
)
