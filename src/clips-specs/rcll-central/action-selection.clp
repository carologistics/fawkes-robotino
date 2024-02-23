(defrule action-selection-select
	?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
                      (id ?id) (state FORMULATED)
                      (action-name ?action-name&:(neq ?action-name move))
                      (param-values $?param-values))
	(plan (id ?plan-id) (goal-id ?goal-id) (suspended FALSE))
	(goal (id ?goal-id) (class ?class) (mode DISPATCHED) (verbosity ?verbosity))
	(or (wm-fact (key game state) (value RUNNING))
	    (test (eq ?action-name send-beacon))
	)

  (not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state PENDING|WAITING|RUNNING|FAILED)))
	(not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state FORMULATED) (id ?oid&:(< ?oid ?id))))
	=>
  (if (neq ?verbosity QUIET) then
    (printout t "Selected next action " ?action-name ?param-values crlf)
  )
	(modify ?pa (state PENDING))
)

(deffunction is-navgraph-node (?pos)
	(if (any-factp ((?nn navgraph-node)) (eq (str-cat ?pos) ?nn:name))
		then
		(return TRUE)
	)
	(return FALSE)
)

(deffunction skill-call-overwrite (?name ?param-names ?param-values $?opt-skiller)
	(bind ?skiller "Skiller")
	(if (> (length$ ?opt-skiller) 0) then (bind ?skiller (nth$ 1 ?opt-skiller)))
	(if (> (length$ ?opt-skiller) 1)
	 then
	)

	;re-map go-wait and move skill navgraph positions to coordinates from
	(if (and (member$ ?name (create$ go-wait move))
			 (is-navgraph-node (nth$ 4 ?param-values))
			 (not (eq (sub-string 1 3 (str-cat (nth$ 4 ?param-values))) "M-Z"))
			 (not (eq (sub-string 1 3 (str-cat (nth$ 4 ?param-values))) "C-Z"))
	 	)
	then
		(bind ?location-name (str-cat (nth$ 4 ?param-values)))
		(if (eq (nth$ 5 ?param-values) INPUT) then (bind ?location-name (sym-cat ?location-name -I)))
		(if (eq (nth$ 5 ?param-values) OUTPUT) then (bind ?location-name (sym-cat ?location-name -O)))
		(do-for-fact ((?nn navgraph-node)) (eq (str-cat ?location-name) ?nn:name)
			(bind ?name (sym-cat ?name -xyo))
			; get x, y and ori from navgraph. add a constant PI to the orientation because of 180 deg flip
			(bind ?param-names (create$ ?param-names x y ori))
			(bind ?ori (+ 0 (string-to-field (nth$ (+ 1 (member$ "orientation" ?nn:properties)) ?nn:properties))))
			(bind ?param-values (create$ ?param-values (nth$ 1 ?nn:pos) (nth$ 2 ?nn:pos) ?ori))
		)
	)

	; And here we rely on a function provided from the outside providing
	; a more sophisticated mapping.
	(bind ?sks (map-action-skill ?name ?param-names ?param-values))
	(printout logwarn "sks='" ?sks "'" crlf)

	(bind ?id UNKNOWN)
	(if (eq ?sks "")
			then
		(bind ?id (sym-cat ?name (gensym*)))
		(assert (skill (id ?id) (name (sym-cat ?name)) (status S_FAILED) (start-time (now))
		        (error-msg (str-cat "Failed to convert action '" ?name "' to skill string"))))
	else
		(bind ?m (blackboard-create-msg (str-cat "SkillerInterface::" ?skiller) "ExecSkillMessage"))
		(blackboard-set-msg-field ?m "skill_string" ?sks)

		(printout logwarn "Calling skill '" ?sks "'" crlf)
		(bind ?msgid (blackboard-send-msg ?m))
		(bind ?status (if (eq ?msgid 0) then S_FAILED else S_IDLE))
		(bind ?id (sym-cat ?name "-" ?msgid))
		(assert (skill (id ?id) (name (sym-cat ?name)) (skill-string ?sks)
		               (skiller ?skiller)
		               (status ?status) (msgid ?msgid) (start-time (now))))
	)
	(return ?id)
)

(defrule action-selection-done
	?g <- (goal (id ?goal-id) (class ~EXPLORATION) (mode DISPATCHED) (type ACHIEVE))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(not (plan-action (goal-id ?goal-id) (plan-id ?) (state ~FINAL)))
	=>
	(modify ?g (mode FINISHED) (outcome COMPLETED))
)

(defrule action-selection-failed
	(plan (id ?plan-id) (goal-id ?goal-id))
	?g <- (goal (id ?goal-id) (class ?class& : (neq ?class EXPLORATION)) (mode DISPATCHED))
	(plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state FAILED))
	=>
	(modify ?g (mode FINISHED) (outcome FAILED))
)

(defrule action-selection-select-move
	?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
	                    (id ?id) (state FORMULATED)
	                    (action-name ?action-name&move)
	                    (param-values ?r ?from ?from-side ?to ?to-side))
	(plan (id ?plan-id) (goal-id ?goal-id) (suspended FALSE))
	(goal (id ?goal-id) (class ?class) (mode DISPATCHED) (verbosity ?verbosity))

	(not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state PENDING|WAITING|RUNNING|FAILED)))
	(not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state FORMULATED) (id ?oid&:(< ?oid ?id))))
	(not (plan-action (state ~FORMULATED&~FAILED&~FINAL)
	                  (param-values ? ? ? ?to ?to-side)))
	=>
	(modify ?pa (state PENDING))
)

(defrule skill-action-start-overwrite
	(declare (salience ?*SALIENCE-OVERWRITE*))
	?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id) (state PENDING)
	                    (action-name ?action-name) (executable TRUE)
	                    (skiller ?skiller)
	                    (param-names $?params)
	                    (param-values $?param-values))
	(skill-action-mapping (name ?action-name))
	(not (skill-action-execinfo (skiller ?skiller)))
	(skiller-control (skiller ?skiller) (acquired TRUE))
	=>
	(bind ?skill-id (skill-call-overwrite ?action-name ?params ?param-values ?skiller))
	(modify ?pa (state WAITING))
	(bind ?args (create$))
	(loop-for-count (?i (length$ ?params))
		(bind ?args (append$ ?args (nth$ ?i ?params) (nth$ ?i ?param-values)))
	)
	(assert (skill-action-execinfo (goal-id ?goal-id) (plan-id ?plan-id)
	                               (action-id ?id) (skill-id ?skill-id)
	                               (skill-name ?action-name)
	                               (skill-args ?args) (skiller ?skiller)))
)
