
(deftemplate goal-meta
  (slot goal-id (type SYMBOL))
  (slot num-tries (type INTEGER))
  (multislot last-achieve (type INTEGER) (cardinality 2 2) (default 0 0))
)

(defglobal
  ?*GOAL-MAX-TRIES* = 3
  ?*SALIENCE-GOAL-FORMULATE* = 500
  ?*SALIENCE-GOAL-REJECT* = 400
  ?*SALIENCE-GOAL-SELECT* = 300

  ; production order priorities
  ?*PRIORITY-FIND-MISSING-MPS* = 110
  ?*PRIORITY-DELIVER* = 100
  ?*PRIORITY-RESET-MPS* = 98
  ?*PRIORITY-CLEAR-BS* = 97
  ?*PRIORITY-PRODUCE-CX* = 95
  ?*PRIORITY-PRODUCE-C0* = 90
  ?*PRIORITY-ADD-ADDITIONAL-RING* = 85
  ?*PRIORITY-ADD-FIRST-RING* = 80
  ?*PRIORITY-CLEAR-CS* = 70
  ?*PRIORITY-CLEAR-RS* = 55
  ?*PRIORITY-PREFILL-CS* = 50
  ?*PRIORITY-PREFILL-RS-WITH-HOLDING-BASE* = 45
  ?*PRIORITY-PREFILL-RS* = 40
  ?*PRIORITY-ADD-ADDITIONAL-RING-WAITING* = 20
  ?*PRIORITY-DISCARD-UNKNOWN* = 10
  ?*PRIORITY-NOTHING-TO-DO* = -1
  ;ToDo:The proirites are copied from old agent
  ;     for the current moment. Filter out uneeded
  ;     later. For now needed for refrence.

)

; #  Goal Creation

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

(defrule goal-reasoer-create-goal-production-maintain
  "The parent production goal. Allowes formulation of
  production goals only if proper game state selected
  and domain loaded. Other production goals are
  formulated as sub-goals of this goal"
  (domain-facts-loaded)
  (not (goal (id PRODUCTION-MAINTAIN)))
  (wm-fact (key refbox state) (type UNKNOWN) (value RUNNING))
  (wm-fact (key refbox phase) (type UNKNOWN) (value PRODUCTION))
  =>
  (assert (goal (id PRODUCTION-MAINTAIN) (type MAINTAIN)))
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

; ## Achieve Goals
(defrule goal-reasoner-create-enter-field
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id PRODUCTION-MAINTAIN) (mode SELECTED))
  (not (goal (id ENTER-FIELD)))
  (not (goal-already-tried ENTER-FIELD))
  (not (goal (type ACHIEVE) ))
  (wm-fact (key domain fact robot-waiting args? r ?robot))
  (not (wm-fact (key domain fact entered-field args? r ?robot)))
  =>
  (printout t "Goal " ENTER-FIELD " formulated" crlf)
  (assert (goal (id ENTER-FIELD) (parent PRODUCTION-MAINTAIN)))
  ; This is just to make sure we formulate the goal only once.
  ; In an actual domain this would be more sophisticated.
  (assert (goal-already-tried ENTER-FIELD))
)

(defrule goal-reasoner-create-fill-cap-goal
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id PRODUCTION-MAINTAIN) (mode SELECTED))
  (not (goal (id FILL-CAP)))
  ; (not (goal-already-tried FILL-CAP))
  (not (goal (type ACHIEVE) ))
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN&~DOWN))
  (wm-fact (key domain fact cs-can-perform args? m ?mps op RETRIEVE_CAP))
  (not (wm-fact (key domain fact cs-buffered args? m ?mps col ?cap-color)))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?wp)))
  ;ToDo: remove condition and ensure by salience
  (wm-fact (key domain fact entered-field args? r R-1))
  ; (test (eq ?robot R-1))
  =>
  (printout t "Goal " FILL-CAP " formulated" crlf)
  (assert (goal (id FILL-CAP) (priority ?*PRIORITY-PREFILL-CS*)
                              (parent PRODUCTION-MAINTAIN)
                              (params robot R-1
                                      mps ?mps
                                      )))
  ; This is just to make sure we formulate the goal only once.
  ; In an actual domain this would be more sophisticated.
  (assert (goal-already-tried FILL-CAP))
)

(defrule goal-reasoner-create-clear-cs
  "Remove an unknown base from CS after retrieving a cap from it."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id PRODUCTION-MAINTAIN) (mode SELECTED))
  (not (goal (id CLEAR-CS)))
  ; (not (goal-already-tried CLEAR-CS))
  (not (goal (type ACHIEVE) ))
  (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side OUTPUT))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col CAP_NONE))
  ;Maybe add a check for the base_color
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (wm-fact (key domain fact mps-state args? m ?mps s READY-AT-OUTPUT))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?some-wp)))
  ; (test (eq ?robot R-1))
  =>
  (printout t "Goal " CLEAR-CS " formulated" crlf)
  (assert (goal (id CLEAR-CS) (priority ?*PRIORITY-CLEAR-CS*)
                              (parent PRODUCTION-MAINTAIN)
                              (params robot R-1
                                      mps ?mps
                                      wp ?wp
                                      )))
  ; This is just to make sure we formulate the goal only once.
  ; In an actual domain this would be more sophisticated.
  (assert (goal-already-tried CLEAR-CS))
)

(defrule goal-reasoner-insert-unknown-base-to-rs
  "Insert a base with unknown color in a RS for preparation"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id PRODUCTION-MAINTAIN) (mode SELECTED))
  (not (goal (id FILL-RS)))
  ; (not (goal-already-tried FILL-RS))
  (not (goal (type ACHIEVE) ))
  (wm-fact (key domain fact wp-usable args? wp ?wp))
  (wm-fact (key domain fact holding args? r ?robot wp ?wp))
  (wm-fact (key domain fact mps-type args? m ?mps t RS))
  (wm-fact (key domain fact mps-state args? m ?mps s ~DOWN&~BROKEN))
  (wm-fact (key domain fact rs-inc args? summand ?rs-before sum ?rs-after))
  (wm-fact (key domain fact rs-filled-with args? m ?mps n ?rs-before&ZERO|ONE|TOW))
  ;CCs don't have a base color. Hence, models base with UNKOWN color
  (not (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color)))
  =>
  (printout t "Goal " FILL-RS " formulated" crlf)
  (assert (goal (id FILL-RS) (priority ?*PRIORITY-PREFILL-RS*)
                             (parent PRODUCTION-MAINTAIN)
                             (params robot ?robot
                                     mps ?mps
                                     wp ?wp
                                     rs-before ?rs-before
                                     rs-after ?rs-after
                                     )))
  (assert (goal-already-tried FILL-RS))
  ;Todo: dont pass the RN in the params, reason about it and check
  ;it again for rejection Or selection
)

(defrule goal-reasoner-create-discard-unknown
  "Discard a base which is not needed if no RS can be pre-filled"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id PRODUCTION-MAINTAIN) (mode SELECTED))
  (not (goal (id DISCARD-UNKNOWN)))
  ; (not (goal-already-tried DISCARD-UNKNOWN))
  (not (goal (type ACHIEVE) ))
  ;To-Do: Model state IDLE
  (wm-fact (key domain fact holding args? r ?robot wp ?wp))
  (wm-fact (key domain fact mps-type args? m ?mps t RS))
  ;only discard if ring stations have at least two bases loaded
  (wm-fact (key domain fact rs-filled-with args? m ?mps n TOW|THREE))

  ;question: or would be more correct to create it and later
  ;  reject it because its not useful
  =>
  (printout t "Goal " DISCARD-UNKNOWN " formulated" crlf)
  (assert (goal (id DISCARD-UNKNOWN) (priority ?*PRIORITY-DISCARD-UNKNOWN*)
                                     (parent PRODUCTION-MAINTAIN)
                                     (params robot ?robot
                                             wp ?wp
                                             )))
  (assert (goal-already-tried DISCARD-UNKNOWN))
)

(defrule goal-reasoner-create-produce-c0
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id PRODUCTION-MAINTAIN) (mode SELECTED))
  (not (goal (id PRODUCE-C0)))
  ; (not (goal-already-tried PRODUCE-C0))
  (not (goal (type ACHIEVE) ))
  ;To-Do: Model state IDLE|wait-and-look-for-alternatives
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN))
  (wm-fact (key domain fact cs-buffered args? m ?mps col ?cap-color))
  (wm-fact (key domain fact cs-can-perform args? m ?mps op MOUNT_CAP))
  (wm-fact (key domain fact mps-type args? m ?bs t BS))
  (not (wm-fact (key domain fact wp-at args? wp ?some-wp m ?mps side ?any-side)))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
  (wm-fact (key domain fact mps-state args? m ?bs s ~BROKEN&~DOWN))
  ;To-Do: Model the bs active-side
  (wm-fact (key domain fact order-complexity args? ord ?order com C0))
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  ;note: could be moved to rejected checks
  (wm-fact (key refbox order ?order quantity-delivered CYAN) (value ?qd&:(> ?qr ?qd)))
  ;ToDo: All the time considerations need to be added
  =>
  (printout t "Goal " PRODUCE-C0 " formulated" crlf)
  (assert (goal (id PRODUCE-C0) (priority ?*PRIORITY-PRODUCE-C0*)
                                (parent PRODUCTION-MAINTAIN)
                                (params robot R-1
                                        bs ?bs
                                        bs-side INPUT
                                        bs-color ?base-color
                                        mps ?mps
                                        cs-color ?cap-color
                                        order ?order
                                        )))
  (assert (goal-already-tried PRODUCE-C0))
)

(defrule goal-reasoner-create-deliver
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id PRODUCTION-MAINTAIN) (mode SELECTED))
  (not (goal (id DELIVER)))
  ; (not (goal-already-tried DELIVER))
  (not (goal (type ACHIEVE) ))
  ;To-Do: Model state IDLE|wait-and-look-for-alternatives
  (wm-fact (key domain fact mps-type args? m ?ds t DS))
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN))

  (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side OUTPUT))
  (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?ring2-color))
  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?ring3-color))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col ?cap-color))

  (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity))
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?ring2-color))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?ring3-color))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
  (wm-fact (key domain fact order-gate args? ord ?order gate ?gate))
  ;note: could be moved to rejected checks
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  (wm-fact (key refbox order ?order quantity-delivered CYAN) (value ?qd&:(> ?qr ?qd)))

  (wm-fact (key evaluated fact wp-for-order args? wp ?wp ord ?order))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
  ;ToDo: All the time considerations need to be added
  =>
  (printout t "Goal " DELIVER " formulated" crlf)
  (assert (goal (id DELIVER) (priority ?*PRIORITY-DELIVER*)
                             (parent PRODUCTION-MAINTAIN)
                             (params robot R-1
                                     mps ?mps
                                     order ?order
                                     wp ?wp
                                     ds ?ds
                                     ds-gate ?gate
                                     base-color ?base-color
                                     cap-color ?cap-color
                                     )))
  (assert (goal-already-tried DELIVER))
)

; #  Goal Selection
; We can choose one or more goals for expansion, e.g., calling
; a planner to determine the required steps.
(defrule goal-reasoner-select
  (declare (salience ?*SALIENCE-GOAL-SELECT*))
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

; ## Goal Evaluation
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

(defrule goal-reasoner-evaluate-completed-subgoal-produce-c0
  ?g <- (goal (id PRODUCE-C0) (parent ?parent-id)
              (mode FINISHED) (outcome COMPLETED)
                              (params robot ?robot
                                      bs ?bs
                                      bs-side ?bs-side
                                      bs-color ?base-color
                                      mps ?mps
                                      cs-color ?cap-color
                                      order ?order
                                      ))
 ?gm <- (goal-meta (goal-id ?parent-id))
 (plan (goal-id PRODUCE-C0)
   (id ?plan-id))
 ?p <-(plan-action
   (plan-id ?plan-id)
   (action-name bs-dispense)
   (param-names r m side wp basecol)
         (param-values ?robot ?bs ?bs-side ?wp ?base-color))
 (time $?now)
 ;ToDO:  Remove param from the matching. This is dangerouns if params changed
 ;ToDo: function support for processing goal params by arg
 ;ToDo: function support for processing action params by name
 =>
 (printout t "Goal '" PRODUCE-C0 "' has been completed, Evaluating" crlf)
 (assert (wm-fact (key evaluated fact wp-for-order args? wp ?wp ord ?order) (value TRUE)))
 (modify ?g (mode EVALUATED))
 (modify ?gm (last-achieve ?now))
)

(defrule goal-reasoner-evaluate-completed-subgoal-common
  ?g <- (goal (id ?goal-id) (parent ?parent-id&~nil) (mode FINISHED) (outcome COMPLETED))
  ?pg <- (goal (id ?parent-id) (mode DISPATCHED))
  ?m <- (goal-meta (goal-id ?parent-id))
  (time $?now)
  (test (neq ?goal-id WPSPAWN-ACHIEVE))
  (test (neq ?goal-id PRODUCE-C0))
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
