(deftemplate move-away-meta (slot goal-id) (slot robot) (slot mps) (slot side))
(defrule assign-move-away-goal
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (not (and (domain-object (name ?robot) (type robot)) (not (domain-fact (name entered-field) (param-values ?robot)))))
    (not (goal (class GOAL-MOVE-AWAY)))
    (machine-used (mps ?s) (order-id ?ord))
    (domain-fact (name at) (param-values ?robot ?s ?side))
    ?rw <- (domain-fact (name robot-waiting) (param-values ?robot))
    ; IDEA: Could also try to match robot at machine that not belongs to the order blocking it (instead of robot-waiting)
    =>
    (bind ?goal-id (gensym*))
    (assert (goal (class GOAL-MOVE-AWAY)
                (id ?goal-id)
                (type ACHIEVE)
                (sub-type SIMPLE)
                (verbosity NOISY) (is-executable TRUE)
    ))
    (assert (move-away-meta (goal-id ?goal-id) (robot ?robot) (mps ?s) (side ?side)))
    (retract ?rw)
)

(defrule select-move-away-goal
    ?g <- (goal (class GOAL-MOVE-AWAY) (mode FORMULATED))
    =>
    (modify ?g (mode SELECTED)) 
)

(defrule expand-move-away-goal
    ?g <- (goal (id ?goal-id) (class GOAL-MOVE-AWAY) (mode SELECTED))
    (move-away-meta (goal-id ?goal-id) (robot ?robot) (mps ?s) (side ?side))

    (domain-fact (name mps-team) (param-values ?ss ?team-color))
    (domain-fact (name mps-type) (param-values ?ss SS))    
    (domain-fact (name mps-side-free) (param-values ?ss ?fs))
    =>
    (plan-assert-sequential MOVE-AWAY-PLAN ?goal-id ?robot
        (plan-assert-action move ?robot ?s ?side ?ss ?fs)
    )
    (printout t ?robot" is moving to " ?ss ?fs " to free up " ?s crlf)
    (modify ?g (mode EXPANDED))
)

(defrule move-away-free-robot
    (goal (id ?goal-id) (class GOAL-MOVE-AWAY) (outcome COMPLETED))
    ?gm <- (move-away-meta (goal-id ?goal-id) (robot ?robot) (mps ?s) (side ?side))
    =>
    (assert (domain-fact (name robot-waiting) (param-values ?robot)))
    (retract ?gm)
)