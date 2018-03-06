
(deftemplate goal-meta
	(slot goal-id (type SYMBOL))
	(slot num-tries (type INTEGER))
  (multislot last-achieve (type INTEGER) (cardinality 2 2) (default 0 0))
)

(defglobal
  ?*GOAL-MAX-TRIES* = 3
)

; #  Goal Creation
(defrule goal-reasoner-create
  (not (goal (id TESTGOAL)))
  (not (goal-already-tried TESTGOAL))
  =>
  (assert (goal (id TESTGOAL)))
  ; This is just to make sure we formulate the goal only once.
  ; In an actual domain this would be more sophisticated.
  (assert (goal-already-tried TESTGOAL))
)

(defrule goal-reasoner-create-enter-field
  (not (goal (id ENTER-FIELD)))
  (not (goal-already-tried ENTER-FIELD))
  (wm-fact (key refbox state) (type UNKNOWN) (value RUNNING))
  (wm-fact (key refbox phase) (type UNKNOWN) (value PRODUCTION))
  (wm-fact (key domain fact robot-waiting args? r ?robot))
  (not (wm-fact (key domain fact entered-field args? r ?robot)))
  =>
  (assert (goal (id ENTER-FIELD)))
  ; This is just to make sure we formulate the goal only once.
  ; In an actual domain this would be more sophisticated.
  (assert (goal-already-tried ENTER-FIELD))
)

(defrule goal-reasoner-create-fill-cap-goal
	(not (goal (id FILL-CAP)))
	(not (goal-already-tried FILL-CAP))
	(wm-fact (key refbox state) (type UNKNOWN) (value RUNNING))
	(wm-fact (key refbox phase) (type UNKNOWN) (value PRODUCTION))
	(wm-fact (key domain fact mps-type args? m ?mps t CS)(value TRUE))
	(wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN&~DOWN) (value TRUE))
	(wm-fact (key domain fact cs-can-perform args? m ?mps op RETRIEVE_CAP) (value TRUE))
    (not (wm-fact (key domain fact cs-buffered args? m ?mps col ?cap-color) (value TRUE)))
	(not (wm-fact (key domain fact holding args? r ?robot wp ?wp)))
	(wm-fact (key domain fact entered-field args? r R-1))
	; (test (eq ?robot R-1))
	=>
	(assert (goal (id FILL-CAP) (params robot R-1 mps ?mps)))
	; This is just to make sure we formulate the goal only once.
	; In an actual domain this would be more sophisticated.
	(assert (goal-already-tried FILL-CAP))
)

(defrule goal-reasoner-create-clear-cs
	"Remove an unknown base from CS after retrieving a cap from it."
	(not (goal (id CLEAR-CS)))
	(not (goal-already-tried CLEAR-CS))
	(wm-fact (key refbox state) (type UNKNOWN) (value RUNNING))
	(wm-fact (key refbox phase) (type UNKNOWN) (value PRODUCTION))
	(wm-fact (key domain fact wp-at args? wp ?wp m ?mps side OUTPUT))
	(wm-fact (key domain fact wp-cap-color args? wp ?wp col CAP_NONE))
	;Maybe add a check for the base_color
	(wm-fact (key domain fact mps-type args? m ?mps t CS))
	(wm-fact (key domain fact mps-state args? m ?mps s READY-AT-OUTPUT))
	(not (wm-fact (key domain fact holding args? r ?robot wp ?wp)))
	; (test (eq ?robot R-1))
	=>
	(assert (goal (id CLEAR-CS) (params robot R-1 mps ?mps wp ?wp)))
	; This is just to make sure we formulate the goal only once.
	; In an actual domain this would be more sophisticated.
	(assert (goal-already-tried CLEAR-CS))
)

; ## Maintenance Goals
(defrule goal-reasoner-create-beacon-maintain
  (not (goal (id BEACONMAINTAIN)))
  =>
  (assert (goal (id BEACONMAINTAIN) (type MAINTAIN)))
)

(defrule goal-reasoner-create-wp-spawn-maintain
 (domain-facts-loaded)
 (not (goal (id WPSPAWN-MAINTAIN)))
 =>
 (assert (goal (id WPSPAWN-MAINTAIN) (type MAINTAIN)))
)

; ### sub-goals of the maintenance goal
(defrule goal-reasoner-create-beacon-achieve
  ?g <- (goal (id BEACONMAINTAIN) (mode SELECTED|DISPATCHED))
  (not (goal (id BEACONACHIEVE)))
  (time $?now)
  ; TODO: make interval a constant
  (goal-meta (goal-id BEACONMAINTAIN)
    (last-achieve $?last&:(timeout ?now ?last 1)))
  =>
  (assert (goal (id BEACONACHIEVE) (parent BEACONMAINTAIN)))
  (modify ?g (mode EXPANDED))
)

(defrule goal-reasoner-create-wp-spawn-achieve
  ?g <- (goal (id WPSPAWN-MAINTAIN) (mode SELECTED|DISPATCHED))
  (not (goal (id WPSPAWN-ACHIEVE)))
  (time $?now)
  ; TODO: make interval a constant
  (goal-meta (goal-id WPSPAWN-MAINTAIN)
  (last-achieve $?last&:(timeout ?now ?last 1)))
  (wm-fact (key domain fact self args? r ?robot))
  (not (and
    (domain-object (name ?wp) (type workpiece))
    (wm-fact (key domain fact wp-spawned-by args? wp ?wp r ?robot))))
  =>
  (assert (goal (id WPSPAWN-ACHIEVE) (parent WPSPAWN-MAINTAIN)))
  (modify ?g (mode EXPANDED))
)



; #  Goal Selection
; We can choose one or more goals for expansion, e.g., calling
; a planner to determine the required steps.
(defrule goal-reasoner-select
  ?g <- (goal (id ?goal-id) (mode FORMULATED))
  =>
  (modify ?g (mode SELECTED))
  (assert (goal-meta (goal-id ?goal-id)))
)

; #  Commit to goal (we "intend" it)
; A goal might actually be expanded into multiple plans, e.g., by
; different planners. This step would allow to commit one out of these
; plans.
(defrule goal-reasoner-commit
  ?g <- (goal (mode EXPANDED))
  =>
  (modify ?g (mode COMMITTED))
)

; #  Dispatch goal (action selection and execution now kick in)
; Trigger execution of a plan. We may commit to multiple plans
; (for different goals), e.g., one per robot, or for multiple
; orders. It is then up to action selection and execution to determine
; what to do when.
(defrule goal-reasoner-dispatch
  ?g <- (goal (mode COMMITTED))
  =>
  (modify ?g (mode DISPATCHED))
)

; #  Goal Monitoring

; ##Sub-Goals Evaluation
(deffunction random-id ()
  "Return a random task id"
  (return (random 0 1000000000))
)

(defrule goal-reasoner-evaluate-completed-subgoal-wp-spawn
  ?g <- (goal (id WPSPAWN-ACHIEVE) (parent WPSPAWN-MAINTAIN) (mode FINISHED) (outcome COMPLETED))
  ?p <- (goal (id WPSPAWN-MAINTAIN) (mode DISPATCHED))
  ?m <- (goal-meta (goal-id WPSPAWN-MAINTAIN))
  (time $?now)
  =>
  (printout debug "Goal '" WPSPAWN-ACHIEVE "' (part of '" WPSPAWN-MAINTAIN
    "') has been completed, Evaluating" crlf)
     (bind ?wp-id (sym-cat WP (random-id)))
  (assert
    (domain-object (name ?wp-id) (type workpiece))
    (wm-fact (key domain fact wp-unused args? wp ?wp-id) (value TRUE))
    (wm-fact (key domain fact wp-cap-color args? wp ?wp-id col CAP_NONE) (value TRUE))
    (wm-fact (key domain fact wp-ring1-color args? wp ?wp-id col RING_NONE) (value TRUE))
    (wm-fact (key domain fact wp-ring2-color args? wp ?wp-id col RING_NONE) (value TRUE))
    (wm-fact (key domain fact wp-ring3-color args? wp ?wp-id col RING_NONE) (value TRUE))
    (wm-fact (key domain fact wp-base-color args? wp ?wp-id col BASE_NONE) (value TRUE))
    (wm-fact (key domain fact wp-spawned-by args? wp ?wp-id r R-1) (value TRUE))
  )
  (modify ?g (mode EVALUATED))
  (modify ?m (last-achieve ?now))
)

; ## Goal Evaluation
(defrule goal-reasoner-evaluate-completed-subgoal-common
  ?g <- (goal (id ?goal-id) (parent ?parent-id&~nil) (mode FINISHED) (outcome COMPLETED))
  ?pg <- (goal (id ?parent-id) (mode DISPATCHED))
  ?m <- (goal-meta (goal-id ?parent-id))
  (time $?now)
  (test (neq ?goal-id WPSPAWN-ACHIEVE))
  =>
  (printout debug "Goal '" ?goal-id "' (part of '" ?parent-id
    "') has been completed, Evaluating" crlf)
  (modify ?g (mode EVALUATED))
  (modify ?m (last-achieve ?now))
)

(defrule goal-reasoner-evaluate-common
  ?g <- (goal (id ?goal-id) (parent nil) (mode FINISHED) (outcome ?outcome))
  ?gm <- (goal-meta (goal-id ?goal-id) (num-tries ?num-tries))
  =>
  (printout t "Goal '" ?goal-id "' has been " ?outcome ", evaluating" crlf)
  (if (eq ?outcome FAILED)
    then
    (bind ?num-tries (+ ?num-tries 1))
    (modify ?gm (num-tries ?num-tries))
  )
  (modify ?g (mode EVALUATED))
)

; # Goal Clean up
(defrule goal-reasoner-cleanup-completed-subgoal-common
  ?g <- (goal (id ?goal-id) (parent ?parent-id&~nil) (mode EVALUATED) (outcome COMPLETED))
  ?pg <- (goal (id ?parent-id) (mode DISPATCHED))
  ?m <- (goal-meta (goal-id ?parent-id))
  =>
  (printout debug "Goal '" ?goal-id "' (part of '" ?parent-id
     "') has been evaluated, cleaning up" crlf)
  (delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?goal-id)
   (delayed-do-for-all-facts ((?a plan-action)) (eq ?a:plan-id ?p:id)
    (retract ?a)
   )
   (retract ?p)
  )
  (retract ?g)
)

(defrule goal-reasoner-cleanup-common
  ?g <- (goal (id ?goal-id) (parent nil) (mode EVALUATED) (outcome ?outcome))
  ?gm <- (goal-meta (goal-id ?goal-id) (num-tries ?num-tries))
  =>
  (printout t "Goal '" ?goal-id "' has been Evaluated, cleaning up" crlf)
  (delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?goal-id)
    (delayed-do-for-all-facts ((?a plan-action)) (eq ?a:plan-id ?p:id)
      (retract ?a)
    )
    (retract ?p)
  )

  (if (and (eq ?outcome failed) (< ?num-tries ?*GOAL-MAX-TRIES*) )
    then
      (printout t "Triggering re-expansion" crlf)
      (modify ?g (mode SELECTED))
    else
      (retract ?g ?gm)
    )
)
