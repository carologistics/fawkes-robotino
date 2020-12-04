; Rules copied from goal-production to set up the goal tree
(defrule goal-production-navgraph-compute-wait-positions-finished
  "Add the waiting points to the domain once their generation is finished."
  (NavGraphWithMPSGeneratorInterface (final TRUE))
=>
  (printout t "Navgraph generation of waiting-points finished. Getting waitpoints." crlf)
  (do-for-all-facts ((?waitzone navgraph-node)) (str-index "WAIT-" ?waitzone:name)
    (assert
      (domain-object (name (sym-cat ?waitzone:name)) (type waitpoint))
      (wm-fact (key navgraph waitzone args? name (sym-cat ?waitzone:name)) (is-list TRUE) (type INT) (values (nth$ 1 ?waitzone:pos) (nth$ 2 ?waitzone:pos)))
    )
  )
  (assert (wm-fact (key navgraph waitzone generated) (type BOOL) (value TRUE)))
)

(defrule goal-production-create-production-maintain
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (domain-facts-loaded)
  (not (goal (class PRODUCTION-MAINTAIN)))
  (wm-fact (key refbox phase) (value PRODUCTION))
  (wm-fact (key game state) (value RUNNING))
  =>
  (goal-tree-assert-run-endless PRODUCTION-MAINTAIN 1)
)

; Formulate a goal to visit a single station (here e.g: "C-BS-O")
(defrule goal-production-create-visit-station
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ; This will be the parent goal
  (goal (id ?production-id) (class NO-PROGRESS))
  ; Only formulate once (not sure if this is needed)
  (not (already-visited))
  ; Get robot name of self
  (wm-fact (key domain fact self args? r ?robot))
=>
  ; Create goal - params chosen to be compatible with goal-expander-go-wait in fixed-sequence.clp
  (assert (goal (id VISITSTATION) (parent ?production-id) (class GO-WAIT) (type ACHIEVE) (sub-type SIMPLE) (params r ?robot point WAIT-C-BS-O)))
  (assert (already-visited))
  (printout t "Goal " VISITSTATION " formulated" crlf)
)