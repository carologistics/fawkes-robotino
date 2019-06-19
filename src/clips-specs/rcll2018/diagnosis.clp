(defrule diag-create-check-workpiece-plans
    ?g <- (goal (id ?id) (mode SELECTED) (class ACTIVE-DIAGNOSIS-SENSE))
    (domain-fact (name self) (param-values ?r))
    (domain-object (name ?mps) (type mps))
    (domain-object (name ?side) (type mps-side))
    (not (plan-action (action-name check-workpiece) (goal-id ?id) (param-values ?r ?mps ?side)))
    =>
    (bind ?plan-id (sym-cat CHECK-WORKPIECE-PLAN- (gensym*)))
    (assert
        (plan (id ?plan-id) (goal-id ?id) (diag-wm-store FALSE))
        (plan-action (goal-id ?id) (plan-id ?plan-id) (action-name check-workpiece) (id 1) (param-names r m side) (param-values ?r ?mps ?side))
    )
)

(defrule diag-create-drive-to-check-workpiece-plans
    ?g <- (goal (id ?id) (mode SELECTED) (class ACTIVE-DIAGNOSIS-SENSE))
    (domain-fact (name self) (param-values ?r))
    (domain-object (name ?mps) (type mps))
    (domain-object (name ?side) (type mps-side))
    (not (plan-action (action-name drive-to-check-workpiece) (goal-id ?id) (param-values ?r ?mps ?side)))
    =>
    (bind ?plan-id (sym-cat DRIVE-TO-CHECK-WORKPIECE-PLAN- (gensym*)))
    (assert
        (plan (id ?plan-id) (goal-id ?id) (diag-wm-store FALSE))
        (plan-action (goal-id ?id) (plan-id ?plan-id) (action-name drive-to-check-workpiece) (id 1) (param-names r m side) (param-values ?r ?mps ?side))
    )
)


(defrule diag-create-gripper-calibrated-plans
    ?g <- (goal (id ?id) (mode SELECTED) (class ACTIVE-DIAGNOSIS-SENSE))
    (not (plan-action (action-name gripper-calibrated) (goal-id ?id) (param-values ?r)))
    (domain-fact (name self) (param-values ?r))
    =>
    (bind ?plan-id (sym-cat GRIPPER-CALIBRATED-PLAN (gensym*)))
    (assert
        (plan (id ?plan-id) (goal-id ?id) (diag-wm-store FALSE))
        (plan-action (goal-id ?id) (plan-id ?plan-id) (action-name gripper-calibrated) (id 1) (param-values ?r))
    )
)


(defrule diag-create-move-base-is-locked-plans
    ?g <- (goal (id ?id) (mode SELECTED) (class ACTIVE-DIAGNOSIS-SENSE))
    (not (plan-action (action-name move-base-is-locked) (goal-id ?id) (param-values ?r)))
    (domain-fact (name self) (param-values ?r))
    =>
    (bind ?plan-id (sym-cat MOVE-BASE-IS-LOCKED-PLAN (gensym*)))
    (assert
        (plan (id ?plan-id) (goal-id ?id) (diag-wm-store FALSE))
        (plan-action (goal-id ?id) (plan-id ?plan-id) (action-name move-base-is-locked) (id 1) (param-values ?r))
    )
)

