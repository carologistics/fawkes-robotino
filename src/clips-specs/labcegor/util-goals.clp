; move away goal
(deftemplate move-away-meta (slot goal-id) (slot robot) (slot mps) (slot side))
(defrule assign-move-away-goal
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (not (and (domain-object (name ?robot) (type robot)) (not (domain-fact (name entered-field) (param-values ?robot)))))
    (machine-used (mps ?s) (order-id ?ord))
    (domain-fact (name at) (param-values ?robot ?s ?side))
    ?rw <- (domain-fact (name robot-waiting) (param-values ?robot))
    (not (move-away-meta (goal-id ?goal-id) (robot ?robot) (mps ?s) (side ?side)))
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

; refill shelf goal
(deftemplate refill-meta (slot goal-id) (slot mps) (slot cap-color))
(defrule assign-refill-shelf-goal
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))

    (domain-facts-loaded)
    (wm-fact (key refbox phase) (value PRODUCTION))  
    (wm-fact (key game state) (value RUNNING))  

    (domain-fact (name mps-team) (param-values ?cs ?team-color))
    (domain-fact (name mps-type) (param-values ?cs CS))
    (domain-fact (name cs-color) (param-values ?cs ?col))
    (not domain-fact (name wp-on-shelf) (param-values ?wp ?cs ?spot))

    (not (and
        (goal (class GOAL-REFILL-SHELF) (id ?goal-id) (outcome ~COMPLETED))
        (refill-meta (goal-id ?goal-id) (mps ?cs) (cap-color ?col))
    ))

    =>

    (bind ?goal-id (gensym*))
    (assert (goal (class GOAL-REFILL-SHELF)
                (id ?goal-id)
                (type ACHIEVE)
                (sub-type SIMPLE)
                (verbosity NOISY) (is-executable TRUE)
    ))
    (assert (refill-meta (goal-id ?goal-id) (mps ?cs) (cap-color ?col)))
)

(defrule select-refill-shelf-goal
    ?g <- (goal (class GOAL-REFILL-SHELF) (mode FORMULATED))
    =>
    (modify ?g (mode SELECTED)) 
)

(defrule expand-refill-shelf-goal
    ?g <- (goal (id ?goal-id) (class GOAL-REFILL-SHELF) (mode SELECTED))
    (refill-meta (goal-id ?goal-id) (mps ?cs) (cap-color ?col))
    
    =>
    
	(plan-assert-sequential REFILL-PLAN ?goal-id central
		(plan-assert-action refill-shelf
		  ?cs LEFT (sym-cat CC- (random-id)) ?col)
		(plan-assert-action refill-shelf
		  ?cs MIDDLE (sym-cat CC- (random-id)) ?col)
		(plan-assert-action refill-shelf
		  ?cs RIGHT (sym-cat CC- (random-id)) ?col)
	)
    (printout t " Refilling " ?cs " . . ." crlf)
    (modify ?g (mode EXPANDED))
)

(defrule refill-shelf-goal-meta-cleanup
    (goal (id ?goal-id) (class GOAL-REFILL-SHELF) (outcome COMPLETED))
    ?gm <- (refill-meta (goal-id ?goal-id) (mps ?cs) (cap-color ?col))
    =>
    (retract ?gm)
)