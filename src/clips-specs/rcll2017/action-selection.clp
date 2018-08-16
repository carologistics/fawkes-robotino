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
	(modify ?wmf (value (str-cat ?status-new)))
)

(defrule action-selection-parents-id-match
	"R-1 evaluates the status and marks plan-actions as executable even if R-1 terminated already its plan"
	; Only R-1 should run the evaluation
	(wm-fact (key config rcll robot-name) (value "R-1"))

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

(defrule action-selection-select-parallel-others
	"select earliest action if no other is chosen and if all actions indicated by parents-ids are finished"
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (status FORMULATED)
					  (action-name ?action-name&send-beacon
	            |enter-field
	            |move
							|wp-get
							|wp-get-shelf
							|wp-put
							|wp-put-slide-cc
							|prepare-bs
							|prepare-cs
							|bs-dispense
							|cs-mount-cap
							|cs-retrieve-cap
							|cs-retrieve-cap
							|prepare-rs
							|rs-mount-ring1
							|rs-mount-ring2
	            |rs-mount-ring3
							|fulfill-order-c0
	            |fulfill-order-c1
	            |fulfill-order-c2
	            |fulfill-order-c3)
					  (param-values $?param-values))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(goal (id ?goal-id) (mode DISPATCHED))

	(not (plan-action (plan-id ?plan-id) (status PENDING|WAITING|RUNNING|FAILED)))
	(not (plan-action (plan-id ?plan-id) (status FORMULATED) (id ?oid&:(< ?oid ?id))))

	; Dependencies of action are matched
	(wm-fact (key plan-action ?goal-id ?plan-id ?sym-id&:(eq (string-to-field (str-cat ?sym-id)) ?id) dep-match))
	=>
	(printout t "Select next action " ?action-name ?param-values " with id " ?sym-id " of plan " ?plan-id " for goal " ?goal-id crlf)
	(modify ?pa (status PENDING))
)

(defrule action-selection-select-parallel-prepare-ds
	"select earliest action if no other is chosen and if all actions indicated by parents-ids are finished"
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (status FORMULATED)
					  (action-name ?action-name&prepare-ds)
					  (param-values $?param-values))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(goal (id ?goal-id) (mode DISPATCHED))

	(not (plan-action (plan-id ?plan-id) (status PENDING|WAITING|RUNNING|FAILED)))
	(not (plan-action (plan-id ?plan-id) (status FORMULATED) (id ?oid&:(< ?oid ?id))))

	(wm-fact (key plan-action ?goal-id ?plan-id ?sym-id&:(eq (string-to-field (str-cat ?sym-id)) ?id) dep-match))

	(wm-fact (key plan-action ?goal-id ?plan-id order) (value ?order-id))
	(wm-fact (key refbox game-time) (values ?sec ?sec-2))
	(wm-fact (key refbox order ?sym-order-id&:(eq (string-to-field (str-cat ?order-id)) ?sym-order-id) delivery-begin) (value ?delivery-begin&:(< ?delivery-begin ?sec)))
	=>
	(printout t "Select next action " ?action-name ?param-values " with id " ?sym-id " in time [" ?sec "] with delivery window beginning at " ?delivery-begin crlf)
	(modify ?pa (status PENDING))
)

(defrule action-selection-done
	(plan (id ?plan-id) (goal-id ?goal-id))
	?g <- (goal (id ?goal-id) (mode DISPATCHED) (type ACHIEVE))
	(not (wm-fact (key plan-action ?goal-id ?plan-id ?id status) (value ?status&:(neq (string-to-field (str-cat ?status)) FINAL)  ) ) )
	=>
	(modify ?g (mode FINISHED) (outcome COMPLETED))
)

(defrule action-selection-failed
	(plan (id ?plan-id) (goal-id ?goal-id))
	?g <- (goal (id ?goal-id& :(neq ?goal-id EXPLORATION)) (mode DISPATCHED))
	(wm-fact (key plan-action ?goal-id ?plan-id ?id status) (value ?status&:(eq (string-to-field (str-cat ?status)) FAILED) ) )
	=>
	(modify ?g (mode FINISHED) (outcome FAILED))
)
