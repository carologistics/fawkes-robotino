(defglobal
	?*GOAL_ID_SLOTS* = id
	?*GOAL_META_ID_SLOTS* = goal-id
	?*PLAN_ID_SLOTS* = id
	?*PLAN_ACTION_ID_SLOTS* = (create$ id goal-id plan-id)

	?*SALIENCE_SKIP_TRIGGER_OVERRIDE* = 1
)

(defrule wm-robmem-sync-init-override
	(declare (salience ?*SALIENCE_SKIP_TRIGGER_OVERRIDE*))
	(executive-init)
	(not (executive-finalize))
	(not (wm-robmem-sync-initialized))
	=>
	(assert (wm-robmem-sync-initialized))
)

(defrule wm-robmem-sync-finalize-override
	(declare (salience ?*SALIENCE_SKIP_TRIGGER_OVERRIDE*))
	(executive-finalize)
	?wi <- (wm-robmem-sync-initialized)
	=>
	(retract ?wi)
)

(defrule init-worldmodel-sync
	(executive-init)
	(wm-fact (key cx identity))
	=>
	(wm-robmem-sync-enable "/domain/")
	(wm-robmem-sync-enable "/template/")
	(wm-robmem-sync-enable "/mps-handling/")
	(wm-robmem-sync-enable "/order/meta/wp-for-order")
	(wm-robmem-sync-enable "/strategy/keep-mps-side-free")
	(wm-robmem-sync-enable "/exploration/")
)

(defrule wm-sync-init-goal-to-wm-fact
	?g <- (goal (id ?id))
	?gm <- (goal-meta (goal-id ?id))
	(not (wm-fact (key template fact goal args? id ?id)))
	=>
	(assert-template-wm-fact ?g
	                         ?*GOAL_ID_SLOTS*
	                         (delete-member$ (deftemplate-remaining-slots goal ?*GOAL_ID_SLOTS*)
	                                         meta-fact))
	(assert-template-wm-fact ?gm
	                         ?*GOAL_META_ID_SLOTS*
	                         (deftemplate-remaining-slots goal-meta ?*GOAL_META_ID_SLOTS*))
)

(defrule wm-sync-update-goals-on-mode-change
	?g <- (goal (id ?id) (mode ?mode))
	?gm <- (goal-meta (goal-id ?id))
	?wm <- (wm-fact (key template fact goal args? id ?id)
	                (values $? mode ?other-mode&:(neq ?mode ?other-mode)))
	?wm2 <- (wm-fact (key template fact goal-meta args? goal-id ?id))
	=>
	(retract ?wm)
	(retract ?wm2)
	(assert-template-wm-fact ?g
	                         ?*GOAL_ID_SLOTS*
	                         (delete-member$ (deftemplate-remaining-slots goal ?*GOAL_ID_SLOTS*)
	                                             meta-fact))
	(assert-template-wm-fact ?gm
	                         ?*GOAL_META_ID_SLOTS*
	                         (deftemplate-remaining-slots goal-meta ?*GOAL_META_ID_SLOTS*))
)

(defrule wm-sync-cleanup-goal-wm-fact
	?wm <- (wm-fact (key template fact goal args? id ?id))
	(not (goal (id ?id)))
	=>
	(retract ?wm)
)

(defrule wm-sync-cleanup-goal-meta-wm-fact
	?wm <- (wm-fact (key template fact goal-meta args? goal-id ?id))
	(not (goal-meta (goal-id ?id)))
	=>
	(retract ?wm)
)

(defrule wm-sync-init-plan-action-to-wm-fact
	?pa <- (plan-action (id ?id-int) (goal-id ?goal-id) (plan-id ?plan-id))
	(not (wm-fact (key template fact plan-action
	               args? id ?id&:(eq (sym-cat ?id-int) (sym-cat ?id)) goal-id ?goal-id plan-id ?plan-id)))
	=>
	(assert-template-wm-fact ?pa
	                         ?*PLAN_ACTION_ID_SLOTS*
	                         (delete-member$ (deftemplate-remaining-slots
	                                           plan-action
	                                           ?*PLAN_ACTION_ID_SLOTS*
	                                         )
	                                         skiller))
)

(defrule wm-sync-update-plan-action-on-state-change
	?pa <- (plan-action (id ?id-int) (goal-id ?goal-id) (plan-id ?plan-id)
	                    (state ?state))
	?wm <- (wm-fact (key template fact plan-action
	                 args? id ?id&:(eq (sym-cat ?id-int) (sym-cat ?id))
	                       goal-id ?goal-id
	                       plan-id ?plan-id)
	                (values $? state ?other-state&:(neq ?state ?other-state) $?))
	=>
	(retract ?wm)
	(assert-template-wm-fact ?pa
	                         ?*PLAN_ACTION_ID_SLOTS*
	                         (delete-member$ (deftemplate-remaining-slots
	                                           plan-action
	                                           ?*PLAN_ACTION_ID_SLOTS*
	                                         )
	                                         skiller))
)

(defrule wm-sync-cleanup-plan-action-wm-fact
	?wm <- (wm-fact (key template fact plan-action
	                 args? id ?id
	                       goal-id ?goal-id
	                       plan-id ?plan-id))
	(not (plan-action (id ?id-int&:(eq (sym-cat ?id-int) (sym-cat ?id)))
	                  (goal-id ?goal-id) (plan-id ?plan-id)))
	=>
	(retract ?wm)
)

(defrule wm-sync-plan-to-wm-fact
	?plan <- (plan (id ?id))
	=>
	(do-for-fact ((?wm wm-fact))
		(wm-key-prefix ?wm:key (create$ template fact plan args? id ?id))
		(retract ?wm)
	)
	(assert-template-wm-fact ?plan
	                         ?*PLAN_ID_SLOTS*
	                         (deftemplate-remaining-slots plan ?*PLAN_ID_SLOTS*))
)

(defrule wm-sync-cleanup-plan-wm-fact
	?wm <- (wm-fact (key template fact plan args? id ?id))
	(not (plan (id ?id)))
	=>
	(retract ?wm)
)
