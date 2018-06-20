(defrule action-selection-exploration-select
	(goal (id EXPLORATION) (mode DISPATCHED))
	?pa <- (plan-action (plan-id EXPLORATION-PLAN) (goal-id EXPLORATION)
			(id ?id) (status FORMULATED)
			(action-name ?action-name)
			(param-values $?param-values))
	(goal (id EXPLORATION) (mode DISPATCHED))
	(not (plan-action (goal-id EXPLORATION) (status FAILED|PENDING|WAITING|RUNNING)))
	(not (plan (id EXPLORE-ZONE) (goal-id EXPLORATION)))
	(not (plan-action (plan-id EXPLORATION-PLAN) (goal-id EXPLORATION) (status FORMULATED) (id ?oid&: (< ?oid ?id))))
	=>
	(printout t "Selected next Exploration move action" ?action-name ?param-values crlf)
	(modify ?pa (status PENDING))
)

(defrule action-selection-exploration-done
	(not (plan-action (goal-id EXPLORATION) (plan-id EXPLORATION-PLAN) (status ?status&~FINAL&~FAILED)))
	?g <- (goal (id EXPLORATION) (mode DISPATCHED))
	=>
	(modify ?g (mode FINISHED) (outcome COMPLETED))
)

(defrule action-seleection-explorezone-release-locks
	(goal (id EXPLORATION) (mode DISPATCHED))
	?p <- (plan (id EXPLORE-ZONE) (goal-id EXPLORATION))
	(not (plan-action (id ?id) (plan-id EXPLORE-ZONE) (goal-id EXPLORATION) (status ?s&~FINAL&~FORMULATED&~FAILED)))
	?rl <- (plan-action (plan-id EXPLORE-ZONE) (action-name release-locks) (status FORMULATED))
	=>
	(modify ?rl (status PENDING))
)


(defrule action-selection-explorezone-done
	?p <- (plan (id EXPLORE-ZONE) (goal-id EXPLORATION))
	(goal (id EXPLORATION) (mode DISPATCHED))
	(or (not (plan-action (id ?id) (plan-id EXPLORE-ZONE) (goal-id EXPLORATION) (status ?s&~FINAL)))
					 (plan-action (id ?id) (plan-id EXPLORE-ZONE) (goal-id EXPLORATION) (status ?s&FAILED))
	)
	(or (plan-action (plan-id EXPLORE-ZONE) (action-name release-locks) (status FINAL))
		  (not (plan-action (plan-id EXPLORE-ZONE) (action-name release-locks))))
	=>
	(do-for-all-facts ((?pa plan-action)) (eq ?pa:plan-id EXPLORE-ZONE)
		(retract ?pa)
	)
	(retract ?p)
)

(defrule action-selection-exploration-failed
	?p <- (plan (id EXPLORATION-PLAN) (goal-id EXPLORATION))
	(goal (id EXPLORATION) (mode DISPATCHED))
	?pa <- (plan-action (plan-id EXPLORATION-PLAN) (goal-id EXPLORATION) (id ?id)
			(action-name move-node) (param-values ?r ?node) (status FAILED))
	(Position3DInterface (id "Pose") (translation $?r-pose))
	(navgraph-node (name ?node) (pos $?node-pos))
	(not (plan (id EXPLORE-ZONE) (goal-id EXPLORATION)))
	?panext <- (plan-action (plan-id EXPLORATION-PLAN) (goal-id EXPLORATION) (id ?oid&: (> ?oid ?id)) (status FORMULATED))
	(not (plan-action (plan-id EXPLORATION-PLAN) (id ?hid&: (and (< ?hid ?oid) (> ?hid ?id))) (status FORMULATED)))
	=>
	(if (< (distance-mf ?node-pos ?r-pose) 1.5) then
		(printout t "EXP Go to next node" crlf)
		(modify ?panext (status PENDING))
		(modify ?pa (status FINAL))
		else
		(printout t "EXP Retry node" crlf)
		(modify ?pa (status FORMULATED))
	)
)

(deffunction parent-actions-finished (?goal-id ?plan-id $?actions-ids)
	"check if all actions in the list of ids are finished successfully"
	(foreach ?a ?actions-ids
		(bind ?result
			(do-for-fact ((?wm-fact wm-fact))
				(and
					(wm-key-prefix ?wm-fact:key (create$ plan-action ?goal-id ?plan-id ?a status))
					(eq (string-to-field (str-cat ?wm-fact:value)) FINAL)
				)
				(printout t "Dependency of action with id " ?a " was detected" crlf) ; TODO Why is this printout crucial for the correct execution?
			)
		)
		(if (not ?result) then
			(return FALSE)
		)
	)
	(return TRUE)
)

(defrule action-update-status
	"update information abount plan-actions status to wm-fact base"
	(plan-action (id ?id) (plan-id ?plan-id) (status ?status-new))
	?wmf <- (wm-fact (key plan-action ?goal-id ?plan-id ?sym-id&:(eq (string-to-field (str-cat ?sym-id)) ?id) status) (value ?status-old&:(not (eq (string-to-field ?status-old) (string-to-field (str-cat ?status-new))))))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(goal (id ?goal-id) (mode DISPATCHED))
	=>
	(printout t "Update status for action with id " ?sym-id " [" ?status-old " -> " ?status-new "]"  crlf)
	(modify ?wmf (value (str-cat ?status-new)))
)

(defrule action-selection-parents-id-match
	"R-1 evaluates the status and marks plan-actions as executable"
	; Only R-1 should run the evaluation
	(wm-fact (key config rcll robot-name) (value "R-1"))

	(goal (id ?goal-id&COMPLEXITY) (mode DISPATCHED))
	(plan (id ?plan-id&SMT-PLAN) (goal-id ?goal-id))
	(wm-fact (key plan-action ?goal-id ?plan-id ?id-o status))

	(wm-fact (key plan-action ?goal-id ?plan-id ?id dep) (values $?parents-ids))
	(wm-fact (key plan-action ?goal-id ?plan-id ?id actor))
	(wm-fact (key plan-action ?goal-id ?plan-id ?id status))
	(wm-fact (key plan-action ?goal-id ?plan-id ?id action))
	(not (wm-fact (key plan-action ?goal-id ?plan-id ?id dep-match)))
	(test (parent-actions-finished ?goal-id ?plan-id ?parents-ids))
=>
	(printout t "Action with id " ?id " matches it's parents-ids" $?parents-ids crlf)
	(assert 
		(wm-fact (key plan-action ?goal-id ?plan-id ?id dep-match))
	)
)

(defrule action-selection-select-parallel
	"select earliest action if no other is chosen and if all actions indicated by parents-ids are finished"
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (status FORMULATED)
					  (action-name ?action-name)
					  (param-values $?param-values))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(goal (id ?goal-id) (mode DISPATCHED))

	(not (plan-action (plan-id ?plan-id) (status PENDING|WAITING|RUNNING|FAILED)))
	(not (plan-action (plan-id ?plan-id) (status FORMULATED) (id ?oid&:(< ?oid ?id))))

	(wm-fact (key plan-action ?goal-id ?plan-id ?sym-id&:(eq (string-to-field (str-cat ?sym-id)) ?id) dep-match))
	=>
	(printout t "Select next action " ?action-name ?param-values " with id " ?sym-id crlf)
	(modify ?pa (status PENDING))
)

(defrule action-selection-done
	(plan (id ?plan-id) (goal-id ?goal-id))
	?g <- (goal (id ?goal-id) (mode DISPATCHED) (type ACHIEVE))
	(not (plan-action (plan-id ?plan-id) (status ~FINAL)))
	=>
	(modify ?g (mode FINISHED) (outcome COMPLETED))
)

(defrule action-selection-failed
	(plan (id ?plan-id) (goal-id ?goal-id))
	?g <- (goal (id ?goal-id& :(neq ?goal-id EXPLORATION)) (mode DISPATCHED))
	(plan-action (goal-id ?goal-id) (plan-id ?plan-id) (status FAILED))
	=>
	(modify ?g (mode FINISHED) (outcome FAILED))
)


;---------------------------------------------------------------------------
;  Add plan action out of wm-facts plan after wm-fact sync
;  TODO
;    - Add filter for correct robot (compare ?action-specific-actor with own name) in order to add only own actions
;    - Add filter for R-1 for exogenous actions
;---------------------------------------------------------------------------

(defrule production-add-plan-action-enter-field
	; If wm-fact plan enter-field is found assert the corresponding plan-action enter-field
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "enter-field" ?robot-name&:(eq ?robot-name (cx-identity)) ?team-color)
	)
	=>
	(printout t "Enter-field action translated from wm-fact-base by " (cx-identity) " and " ?robot-name crlf)
	(assert
		 (plan-action
			(id ?next-step-id)
			(plan-id ?plan-id)
			(goal-id ?goal-id)
                        (duration 4.0)
			(action-name enter-field)
			(param-names r team-color)
			(param-values (string-to-field ?robot-name) (string-to-field ?team-color)))
	)
)

(defrule production-add-plan-action-move
	; If wm-fact plan move is found assert the corresponding plan-action move
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "move" ?robot-name&:(eq ?robot-name (cx-identity)) ?from ?from-side ?to ?to-side)
	)
	=>
	(printout t "Move action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name move)
			(param-names r from from-side to to-side)
			(param-values (string-to-field ?robot-name) (string-to-field ?from) (string-to-field ?from-side) (string-to-field ?to) (string-to-field ?to-side))
		)
	)
)

(defrule production-add-plan-action-wp-get-shelf
	; If wm-fact plan wp-get-shelf is found assert the corresponding plan-action wp-get-shelf
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "wp-get-shelf" ?robot-name&:(eq ?robot-name (cx-identity)) ?wp ?mps ?shelf)
	)
	=>
	(printout t "Wp-get-shelf action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name wp-get-shelf)
			(param-names r cc m spot)
			(param-values (string-to-field ?robot-name) (string-to-field ?wp) (string-to-field ?mps) (string-to-field ?shelf))
		)
	)
)

(defrule production-add-plan-action-wp-get
	; If wm-fact plan wp-get is found assert the corresponding plan-action wp-get
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "wp-get" ?robot-name&:(eq ?robot-name (cx-identity)) ?wp ?mps ?side)
	)
	=>
	(printout t "Wp-get action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name wp-get)
			(param-names r wp m side)
			(param-values (string-to-field ?robot-name) (string-to-field ?wp) (string-to-field ?mps) (string-to-field ?side))
		)
	)
)

(defrule production-add-plan-action-wp-put
	; If wm-fact plan wp-put is found assert the corresponding plan-action wp-put
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "wp-put" ?robot-name&:(eq ?robot-name (cx-identity)) ?wp ?mps)
	)
	=>
	(printout t "Wp-put action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name wp-put)
			(param-names r wp m)
			(param-values (string-to-field ?robot-name) (string-to-field ?wp) (string-to-field ?mps))
		)
	)
)

(defrule production-add-plan-action-wp-put-slide-cc
	; If wm-fact plan wp-put-slide-cc is found assert the corresponding plan-action wp-put-slide-cc
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "wp-put-slide-cc" ?robot-name&:(eq ?robot-name (cx-identity)) ?wp ?mps ?rs-before ?rs-after)
	)
	=>
	(printout t "Wp-put-slide-cc action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name wp-put-slide-cc)
			(param-names r wp m rs-before rs-after)
			(param-values (string-to-field ?robot-name) (string-to-field ?wp) (string-to-field ?mps) (string-to-field ?rs-before) (string-to-field ?rs-after))
		)
	)
)

(defrule production-add-plan-action-wp-discard
	; If wm-fact plan wp-discard is found assert the corresponding plan-action wp-discard
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "wp-discard" ?robot-name&:(eq ?robot-name (cx-identity)) ?wp)
	)
	=>
	(printout t "Wp-discard action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name wp-discard)
			(param-names r cc)
			(param-values (string-to-field ?robot-name) (string-to-field ?wp))
		)
	)
)

(defrule production-add-plan-action-prepare-bs
	; If wm-fact plan prepare-bs is found assert the corresponding plan-action prepare-bs
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id actor)
		(value ?robot-name&:(eq ?robot-name (cx-identity)))
	)
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "prepare-bs" ?mps ?side ?goal-base-color)
	)
	=>
	(assert
	(printout t "Prepare-bs action translated from wm-fact-base by " (cx-identity) crlf)
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name prepare-bs)
			(param-names m side bc)
			(param-values (string-to-field ?mps) (string-to-field ?side) (string-to-field ?goal-base-color))
		)
	)
)

(defrule production-add-plan-action-bs-dispense
	; If wm-fact plan bs-dispense is found assert the corresponding plan-action bs-dispense
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "bs-dispense" ?robot-name&:(eq ?robot-name (cx-identity)) ?mps ?side ?wp ?goal-base-color)
	)
	=>
	(printout t "Bs-dispense action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name bs-dispense)
			(param-names r m side wp basecol)
			(param-values (string-to-field ?robot-name) (string-to-field ?mps) (string-to-field ?side) (string-to-field ?wp) (string-to-field ?goal-base-color))
		)
	)
)

(defrule production-add-plan-action-prepare-ds
	; If wm-fact plan prepare-ds is found assert the corresponding plan-action prepare-ds
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id actor)
		(value ?robot-name&:(eq ?robot-name (cx-identity)))
	)
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "prepare-ds" ?mps ?gate)
	)
	=>
	(printout t "Prepare-ds action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name prepare-ds)
			(param-names m gate)
			(param-values (string-to-field ?mps) (string-to-field ?gate))
		)
	)
)

(defrule production-add-plan-action-fulfill-order-c0
	; If wm-fact plan fulfill-order-c0 is found assert the corresponding plan-action fulfill-order-c0
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id actor)
		(value ?robot-name&:(eq ?robot-name (cx-identity)))
	)
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "fulfill-order-c0" ?order-id ?wp ?mps ?gate ?base-color ?cap-color)
	)
	=>
	(printout t "Fulfill-c0 action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name fulfill-order-c0)
			(param-names ord wp m g basecol capcol)
			(param-values (string-to-field ?order-id) (string-to-field ?wp) (string-to-field ?mps) (string-to-field ?gate) (string-to-field ?base-color) (string-to-field ?cap-color))
		)
	)
)

(defrule production-add-plan-action-fulfill-order-c1
	; If wm-fact plan fulfill-order-c1 is found assert the corresponding plan-action fulfill-order-c1
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id actor)
		(value ?robot-name&:(eq ?robot-name (cx-identity)))
	)
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "fulfill-order-c1" ?order-id ?wp ?mps ?gate ?base-color ?cap-color ?ring1-color)
	)
	=>
	(printout t "Fulfill-c1 action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name fulfill-order-c1)
			(param-names ord wp m g basecol capcol ring1col)
			(param-values (string-to-field ?order-id) (string-to-field ?wp) (string-to-field ?mps) (string-to-field ?gate) (string-to-field ?base-color) (string-to-field ?cap-color) (string-to-field ?ring1-color))
		)
	)
)

(defrule production-add-plan-action-fulfill-order-c2
	; If wm-fact plan fulfill-order-c2 is found assert the corresponding plan-action fulfill-order-c2
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id actor)
		(value ?robot-name&:(eq ?robot-name (cx-identity)))
	)
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "fulfill-order-c2" ?order-id ?wp ?mps ?gate ?base-color ?cap-color ?ring1-color ?ring2-color)
	)
	=>
	(printout t "Fulfill-c2 action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name fulfill-order-c2)
			(param-names ord wp m g basecol capcol ring1col ring2col)
			(param-values (string-to-field ?order-id) (string-to-field ?wp) (string-to-field ?mps) (string-to-field ?gate) (string-to-field ?base-color) (string-to-field ?cap-color) (string-to-field ?ring1-color) (string-to-field ?ring2-color))
		)
	)
)

(defrule production-add-plan-action-fulfill-order-c3
	; If wm-fact plan fulfill-order-c3 is found assert the corresponding plan-action fulfill-order-c3
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id actor)
		(value ?robot-name&:(eq ?robot-name (cx-identity)))
	)
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "fulfill-order-c3" ?order-id ?wp ?mps ?gate ?base-color ?cap-color ?ring1-color ?ring2-color ?ring3-color)
	)
	=>
	(printout t "Fulfill-c3 action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name fulfill-order-c3)
			(param-names ord wp m g basecol capcol ring1col ring2col ring3col)
			(param-values (string-to-field ?order-id) (string-to-field ?wp) (string-to-field ?mps) (string-to-field ?gate) (string-to-field ?base-color) (string-to-field ?cap-color) (string-to-field ?ring1-color) (string-to-field ?ring2-color) (string-to-field ?ring3-color))
		)
	)
)

(defrule production-add-plan-action-prepare-cs
	; If wm-fact plan prepare-cs is found assert the corresponding plan-action prepare-cs
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id actor)
		(value ?robot-name&:(eq ?robot-name (cx-identity)))
	)
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "prepare-cs" ?mps ?operation)
	)
	=>
	(printout t "Prepare-cs action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name prepare-cs)
			(param-names m op)
			(param-values (string-to-field ?mps) (string-to-field ?operation))
		)
	)
)

(defrule production-add-plan-action-cs-retrieve-cap
	; If wm-fact plan cs-retrieve-cap is found assert the corresponding plan-action cs-retrieve-cap
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id actor)
		(value ?robot-name&:(eq ?robot-name (cx-identity)))
	)
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "cs-retrieve-cap" ?mps ?wp ?cap-color)
	)
	=>
	(printout t "Cs-retrieve-cap action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name cs-retrieve-cap)
			(param-names m cc capcol)
			(param-values (string-to-field ?mps) (string-to-field ?wp) (string-to-field ?cap-color))
		)
	)
)

(defrule production-add-plan-action-cs-mount-cap
	; If wm-fact plan cs-mount-cap is found assert the corresponding plan-action cs-mount-cap
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id actor)
		(value ?robot-name&:(eq ?robot-name (cx-identity)))
	)
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "cs-mount-cap" ?mps ?wp ?cap-color)
	)
	=>
	(printout t "Cs-mount-cap action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name cs-mount-cap)
			(param-names m wp capcol)
			(param-values (string-to-field ?mps) (string-to-field ?wp) (string-to-field ?cap-color))
		)
	)
)

(defrule production-add-plan-action-prepare-rs
	; If wm-fact plan prepare-rs is found assert the corresponding plan-action prepare-rs
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id actor)
		(value ?robot-name&:(eq ?robot-name (cx-identity)))
	)
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "prepare-rs" ?mps ?goal-ring-color ?rs-before ?rs-after ?r-req)
	)
	=>
	(printout t "Prepare-rs action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name prepare-rs)
			(param-names m rc rs-before rs-after r-req)
			(param-values (string-to-field ?mps) (string-to-field ?goal-ring-color) (string-to-field ?rs-before) (string-to-field ?rs-after) (string-to-field ?r-req))
		)
	)
)

(defrule production-add-plan-action-rs-mount-ring1
	; If wm-fact plan rs-mount-ring1 is found assert the corresponding plan-action rs-mount-ring1
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id actor)
		(value ?robot-name&:(eq ?robot-name (cx-identity)))
	)
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "rs-mount-ring1" ?mps ?wp ?ring-color ?rs-before ?rs-after ?r-req)
	)
	=>
	(printout t "Rs-mount-ring1 action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name rs-mount-ring1)
			(param-names m wp col rs-before rs-after r-req)
			(param-values (string-to-field ?mps) (string-to-field ?wp) (string-to-field ?ring-color) (string-to-field ?rs-before) (string-to-field ?rs-after) (string-to-field ?r-req))
		)
	)
)

(defrule production-add-plan-action-rs-mount-ring2
	; If wm-fact plan rs-mount-ring2 is found assert the corresponding plan-action rs-mount-ring2
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id actor)
		(value ?robot-name&:(eq ?robot-name (cx-identity)))
	)
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "rs-mount-ring2" ?mps ?wp ?ring-color ?col1 ?rs-before ?rs-after ?r-req)
	)
	=>
	(printout t "Rs-mount-ring2 action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name rs-mount-ring2)
			(param-names m wp col col1 rs-before rs-after r-req)
			(param-values (string-to-field ?mps) (string-to-field ?wp) (string-to-field ?ring-color) (string-to-field ?col1) (string-to-field ?rs-before) (string-to-field ?rs-after) (string-to-field ?r-req))
		)
	)
)

(defrule production-add-plan-action-rs-mount-ring3
	; If wm-fact plan rs-mount-ring3 is found assert the corresponding plan-action rs-mount-ring3
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id actor)
		(value ?robot-name&:(eq ?robot-name (cx-identity)))
	)
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "rs-mount-ring3" ?mps ?wp ?ring-color ?col1 ?col2 ?rs-before ?rs-after ?r-req)
	)
	=>
	(printout t "Rs-mount-ring3 action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name rs-mount-ring3)
			(param-names m wp col col1 col2 rs-before rs-after r-req)
			(param-values (string-to-field ?mps) (string-to-field ?wp) (string-to-field ?ring-color) (string-to-field ?col1) (string-to-field ?col2) (string-to-field ?rs-before) (string-to-field ?rs-after) (string-to-field ?r-req))
		)
	)
)
