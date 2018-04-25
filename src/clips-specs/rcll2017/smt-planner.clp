
;---------------------------------------------------------------------------
;  smt-planenr.clp - Planning for RCLL production with the plugin clips-smt
;
;  Created: Tue Feb 20 16:406 2018 (Cologne)
;  Copyright  2018  Igor Nicolai Bongartz
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------


;---------------------------------------------------------------------------
;  Create data for plugin clips-smt from fact base
;---------------------------------------------------------------------------

; High level pb strucutre collecting data the plugin needs
(deffunction smt-create-data (?robots ?machines ?orders ?rings)
	(bind ?p (pb-create "llsf_msgs.ClipsSmtData"))

	(foreach ?r ?robots
		(pb-add-list ?p "robots" ?r)
	)
	(foreach ?m ?machines
		(pb-add-list ?p "machines" ?m)
	)
	(foreach ?o ?orders
		(pb-add-list ?p "orders" ?o)
	)
	(foreach ?ring ?rings
		(pb-add-list ?p "rings" ?ring)
	)

	; (printout t "Proto:" (pb-tostring ?p) crlf)
	(return ?p)
)

; Add robot information from worldmodel to protobuf
(deffunction smt-create-robot (?name ?team-color ?number ?pose-x ?pose-y)
	(bind ?r (pb-create "llsf_msgs.Robot"))

	(pb-set-field ?r "name" ?name)
	(pb-set-field ?r "team_color" ?team-color)
	(pb-set-field ?r "number" 1) ; TODO adapt to constant ?*ROBOT-NUMBER*)

	; TODO is pose relevant for the plugin?
	(bind ?pose (pb-create "llsf_msgs.Pose2D"))
	(pb-set-field ?pose "x" ?pose-x)
	(pb-set-field ?pose "y" ?pose-y)
	(pb-set-field ?pose "ori" 0.0)
	(pb-set-field ?r "pose" ?pose)

	; Detect if some robot is currently holding wp
	(do-for-fact ((?holding wm-fact))
		(and
			(wm-key-prefix ?holding:key (create$ domain fact holding))
			(eq ?name (wm-key-arg ?holding:key r))
		)

		(bind ?o (pb-create "llsf_msgs.Order"))

		; TODO read information from ?holding
		(pb-set-field ?o "id" 1)
		(pb-set-field ?o "delivery_gate" 1)
		(pb-set-field ?o "complexity" 3)
		(pb-set-field ?o "quantity_requested" 1)
		(if (eq ?team-color CYAN)
		 then
		  (pb-set-field ?o "quantity_delivered_cyan" 0)
		 else
		  (pb-set-field ?o "quantity_delivered_magenta" 0)
		)
		(pb-set-field ?o "delivery_period_begin" 0)
		(pb-set-field ?o "delivery_period_end" 900)

		; Extract wp-color information of (possible intermediate) WP1

		; wp-base-color
		(do-for-fact ((?wm-fact wm-fact))
			(and
				(wm-key-prefix ?wm-fact:key (create$ domain fact wp-base-color))
				(eq WP1 (wm-key-arg ?wm-fact:key wp))
			)

			(pb-set-field ?o "base_color" (wm-key-arg ?wm-fact:key col))
		)

		; wp-cap-color
		(do-for-fact ((?wm-fact wm-fact))
			(and
				(wm-key-prefix ?wm-fact:key (create$ domain fact wp-cap-color))
				(eq WP1 (wm-key-arg ?wm-fact:key wp))
			)

			(pb-set-field ?o "cap_color" (wm-key-arg ?wm-fact:key col))
		)

		; wp-ring-color
		(bind ?rlist (create$))
		(do-for-fact ((?wm-fact wm-fact))
			(and
				(wm-key-prefix ?wm-fact:key (create$ domain fact wp-ring1-color))
				(eq WP1 (wm-key-arg ?wm-fact:key wp))
			)

			(bind ?rlist (append$ ?rlist (wm-key-arg ?wm-fact:key col)))
		)
		(do-for-fact ((?wm-fact wm-fact))
			(and
				(wm-key-prefix ?wm-fact:key (create$ domain fact wp-ring2-color))
				(eq WP1 (wm-key-arg ?wm-fact:key wp))
			)

			(bind ?rlist (append$ ?rlist (wm-key-arg ?wm-fact:key col)))
		)
		(do-for-fact ((?wm-fact wm-fact))
			(and
				(wm-key-prefix ?wm-fact:key (create$ domain fact wp-ring3-color))
				(eq WP1 (wm-key-arg ?wm-fact:key wp))
			)

			(bind ?rlist (append$ ?rlist (wm-key-arg ?wm-fact:key col)))
		)
		(foreach ?rings ?rlist
		 (pb-add-list ?o "ring_colors" ?rings)
		)

		(pb-set-field ?r "wp" ?o)
	)

	(return ?r)
)

(deffunction smt-create-robots (?team-color)
	(bind ?rv (create$))

	(do-for-all-facts ((?wm-fact wm-fact))
		(wm-key-prefix ?wm-fact:key (create$ domain fact robot-waiting))

		(bind ?rv (append$ ?rv (smt-create-robot (wm-key-arg ?wm-fact:key r) ?team-color 0 0 0))) ; TODO add correct pose of robot
	)

	(return ?rv)
)

; Add machine information from worldmodel to protobuf
(deffunction smt-create-machine (?name ?mtype ?state ?team-color ?pose-x ?pose-y)
	(bind ?m (pb-create "llsf_msgs.Machine"))

	(pb-set-field ?m "name" (str-cat ?name))
	(pb-set-field ?m "type" (str-cat ?mtype))
	(pb-set-field ?m "state" (str-cat ?state))
	(pb-set-field ?m "team_color" (str-cat ?team-color))

	; TODO is pose relevant as the location is inside the navgraph
	(bind ?pose (pb-create "llsf_msgs.Pose2D"))
	(pb-set-field ?pose "x" ?pose-x)
	(pb-set-field ?pose "y" ?pose-y)
	(pb-set-field ?pose "ori" 0.0)
	(pb-set-field ?m "pose" ?pose)

	; if machine is of type ringstation
	(if (eq ?mtype RS) then
		(bind ?rlist (create$))

		; set spec of rs
		(do-for-all-facts ((?wm-fact wm-fact))
			(and
				(wm-key-prefix ?wm-fact:key (create$ domain fact rs-ring-spec))
				(eq ?name (wm-key-arg ?wm-fact:key m))
			)

			(bind ?rlist (append$ ?rlist (wm-key-arg ?wm-fact:key r)))
		)

		(foreach ?rings ?rlist
			(pb-add-list ?m "ring_colors" ?rings)
		)

		; set amount of additional bases machine is loaded with
		(do-for-fact ((?wm-fact wm-fact))
			(and
				(wm-key-prefix ?wm-fact:key (create$ domain fact rs-filled-with))
				(eq ?name (wm-key-arg ?wm-fact:key m))
			)

		   (pb-set-field ?m "loaded_with" (wm-key-arg ?wm-fact:key n))
		)
    )

	; if machine is of type capstation
	(if (eq ?mtype CS) then

		; set amount of additional bases machine is loaded with
		(do-for-fact ((?wm-fact wm-fact))
			(and
				(wm-key-prefix ?wm-fact:key (create$ domain fact cs-prepared-for))
				(eq ?name (wm-key-arg ?wm-fact:key m))
			)

		   (pb-set-field ?m "cs_prepared_for" (wm-key-arg ?wm-fact:key c))
		)
	)

	; Detect which machine has wps at the output
	(do-for-fact ((?wp-at wm-fact))
		(and
			(wm-key-prefix ?wp-at:key (create$ domain fact wp-at))
			(eq ?name (wm-key-arg ?wp-at:key m))
		)

		(bind ?o (pb-create "llsf_msgs.Order"))

		; TODO read information from ?wp-at
		(pb-set-field ?o "id" 1)
		(pb-set-field ?o "delivery_gate" 1)
		(pb-set-field ?o "complexity" 3)
		(pb-set-field ?o "quantity_requested" 1)
		(if (eq ?team-color CYAN)
		 then
		  (pb-set-field ?o "quantity_delivered_cyan" 0)
		 else
		  (pb-set-field ?o "quantity_delivered_magenta" 0)
		)
		(pb-set-field ?o "delivery_period_begin" 0)
		(pb-set-field ?o "delivery_period_end" 900)

		; Extract wp-color information of (possible intermediate) WP1

		; wp-base-color
		(do-for-fact ((?wm-fact wm-fact))
			(and
				(wm-key-prefix ?wm-fact:key (create$ domain fact wp-base-color))
				(eq WP1 (wm-key-arg ?wm-fact:key wp))
			)

			(pb-set-field ?o "base_color" (wm-key-arg ?wm-fact:key col))
		)

		; wp-cap-color
		(do-for-fact ((?wm-fact wm-fact))
			(and
				(wm-key-prefix ?wm-fact:key (create$ domain fact wp-cap-color))
				(eq WP1 (wm-key-arg ?wm-fact:key wp))
			)

			(pb-set-field ?o "cap_color" (wm-key-arg ?wm-fact:key col))
		)

		; wp-ring-color
		(bind ?rlist (create$))
		(do-for-fact ((?wm-fact wm-fact))
			(and
				(wm-key-prefix ?wm-fact:key (create$ domain fact wp-ring1-color))
				(eq WP1 (wm-key-arg ?wm-fact:key wp))
			)

			(bind ?rlist (append$ ?rlist (wm-key-arg ?wm-fact:key col)))
		)
		(do-for-fact ((?wm-fact wm-fact))
			(and
				(wm-key-prefix ?wm-fact:key (create$ domain fact wp-ring2-color))
				(eq WP1 (wm-key-arg ?wm-fact:key wp))
			)

			(bind ?rlist (append$ ?rlist (wm-key-arg ?wm-fact:key col)))
		)
		(do-for-fact ((?wm-fact wm-fact))
			(and
				(wm-key-prefix ?wm-fact:key (create$ domain fact wp-ring3-color))
				(eq WP1 (wm-key-arg ?wm-fact:key wp))
			)

			(bind ?rlist (append$ ?rlist (wm-key-arg ?wm-fact:key col)))
		)
		(foreach ?rings ?rlist
		 (pb-add-list ?o "ring_colors" ?rings)
		)

		(pb-set-field ?m "wp" ?o)
	)

	(return ?m)
)

(deffunction smt-create-machines (?team-color)
	(bind ?rv (create$))

	(do-for-all-facts ((?wm-fact wm-fact) (?wm-fact2 wm-fact))
		(and
			(wm-key-prefix ?wm-fact:key (create$ domain fact mps-state))
			(wm-key-prefix ?wm-fact2:key (create$ domain fact mps-type))
			(eq (wm-key-arg ?wm-fact:key m) (wm-key-arg ?wm-fact2:key m))
		)

		(bind ?rv (append$ ?rv (smt-create-machine
									(wm-key-arg ?wm-fact:key m) ; name
									(wm-key-arg ?wm-fact2:key t) ; type
									(wm-key-arg ?wm-fact:key s) ; state
									?team-color
									0 ; pose-x
									0))) ; pose-y
	)

	(return ?rv)
)

; Add order information from worldmodel to protobuf
(deffunction smt-create-order (?id ?gate ?complexity ?q-req ?q-del ?begin ?end ?team-color)
	; (printout t "Creating Data msgs:: Order with id " ?id  crlf)
	(bind ?o (pb-create "llsf_msgs.Order"))

	(pb-set-field ?o "id" 1) ; TODO Use or ommit real ?id
	(pb-set-field ?o "delivery_gate" 1) ; TODO Use or ommit real ?gate
	(pb-set-field ?o "complexity" ?complexity)
	(pb-set-field ?o "quantity_requested" ?q-req)
	(if (eq ?team-color CYAN)
		then
			(pb-set-field ?o "quantity_delivered_cyan" ?q-del)
		else
			(pb-set-field ?o "quantity_delivered_magenta" ?q-del)
	)
	(pb-set-field ?o "delivery_period_begin" ?begin)
	(pb-set-field ?o "delivery_period_end" ?end)

	; Extract order-base-color
	(do-for-fact ((?wm-fact wm-fact))
		(and
			(wm-key-prefix ?wm-fact:key (create$ domain fact order-base-color))
			(eq ?id (wm-key-arg ?wm-fact:key ord))
		)
		(pb-set-field ?o "base_color" (wm-key-arg ?wm-fact:key col))
	)
	; Extract order-cap-color
	(do-for-fact ((?wm-fact wm-fact))
		(and
			(wm-key-prefix ?wm-fact:key (create$ domain fact order-cap-color))
			(eq ?id (wm-key-arg ?wm-fact:key ord))
		)
		(pb-set-field ?o "cap_color" (wm-key-arg ?wm-fact:key col))
	)

	; Extract order-ring-color
	(bind ?rlist (create$))
	; order-ring1-color
	(do-for-fact ((?wm-fact wm-fact))
		(and
			(wm-key-prefix ?wm-fact:key (create$ domain fact order-ring1-color))
			(eq ?id (wm-key-arg ?wm-fact:key ord))
		)
		(bind ?rlist (append$ ?rlist (wm-key-arg ?wm-fact:key col)))
	)
	; order-ring2-color
	(do-for-fact ((?wm-fact wm-fact))
		(and
			(wm-key-prefix ?wm-fact:key (create$ domain fact order-ring2-color))
			(eq ?id (wm-key-arg ?wm-fact:key ord))
		)
		(bind ?rlist (append$ ?rlist (wm-key-arg ?wm-fact:key col)))
	)
	; order-ring3-color
	(do-for-fact ((?wm-fact wm-fact))
		(and
			(wm-key-prefix ?wm-fact:key (create$ domain fact order-ring3-color))
			(eq ?id (wm-key-arg ?wm-fact:key ord))
		)
		(bind ?rlist (append$ ?rlist (wm-key-arg ?wm-fact:key col)))
	)

	(foreach ?rings ?rlist
	 (pb-add-list ?o "ring_colors" ?rings)
	)

  (return ?o)
)

(deffunction smt-create-orders (?team-color)
	(bind ?rv (create$))

	(do-for-fact ((?wm-fact wm-fact))
		(and
			(wm-key-prefix ?wm-fact:key (create$ domain fact order-complexity))
			(eq C3 (wm-key-arg ?wm-fact:key com)) ; Desiered complexity is set here
		)

		(do-for-fact ((?wm-fact2 wm-fact))
			(and
				(wm-key-prefix ?wm-fact2:key (create$ domain fact order-gate))
				(eq (wm-key-arg ?wm-fact:key ord) (wm-key-arg ?wm-fact2:key ord))
			)

			(do-for-fact ((?wm-fact3 wm-fact)) (eq ?wm-fact3:key (create$ refbox order (wm-key-arg ?wm-fact:key ord) quantity-requested))

				(do-for-fact ((?wm-fact4 wm-fact)) (eq ?wm-fact4:key (create$ refbox order (wm-key-arg ?wm-fact:key ord) quantity-delivered ?team-color))

					(do-for-fact ((?wm-fact5 wm-fact)) (eq ?wm-fact5:key (create$ refbox order (wm-key-arg ?wm-fact:key ord) delivery-begin))

						(do-for-fact ((?wm-fact6 wm-fact)) (eq ?wm-fact6:key (create$ refbox order (wm-key-arg ?wm-fact:key ord) delivery-end))

							(bind ?rv (append$ ?rv (smt-create-order
														(wm-key-arg ?wm-fact:key ord) ; order-id
														(wm-key-arg ?wm-fact2:key gate) ; TODO Add the correct gate information
														(wm-key-arg ?wm-fact:key com)
														?wm-fact3:value ; quantity-requested
														?wm-fact4:value ; quantity-delivered
														?wm-fact5:value ; begin
														?wm-fact6:value ; end
														?team-color)))
						)
					)
				)
			)
		)
	)
	(return ?rv)
)

; Add ring information from worldmodel to protobuf
(deffunction smt-create-ring (?ring-color ?req-bases)
  ; (printout t "Creating Data msgs:: Ring color " ?ring-color " with req-bases " ?req-bases  crlf)
  (bind ?r (pb-create "llsf_msgs.Ring"))

  (pb-set-field ?r "ring_color" ?ring-color)
  (bind ?req-bases-number 0)
      (switch ?req-bases
        (case ONE then (bind ?req-bases-number 1))
        (case TWO then (bind ?req-bases-number 2))
        (case THREE then (bind ?req-bases-number 3))
        (default (printout warn "Ring color not found" crlf))
      )
  (pb-set-field ?r "raw_material" ?req-bases-number)

  (return ?r)
)

(deffunction smt-create-rings (?team-color)
  (bind ?rv (create$))

      (do-for-all-facts ((?wm-fact wm-fact)) (wm-key-prefix ?wm-fact:key (create$ domain fact rs-ring-spec))

		(bind ?rv (append$ ?rv (smt-create-ring (wm-key-arg ?wm-fact:key r) (wm-key-arg ?wm-fact:key rn))))
      )

  (return ?rv)
)

;---------------------------------------------------------------------------
;  Call plugin clips-smt
;---------------------------------------------------------------------------

(defrule production-call-clips-smt
	; Call plugin clips-smt
	(goal (id ?goal-id&COMPLEXITY) (mode SELECTED))

	(wm-fact (key refbox phase) (value PRODUCTION))
	(wm-fact (key refbox team-color) (value ?team-color&CYAN|MAGENTA))
	(wm-fact (key domain fact rs-ring-spec args? m ?mps r ?ring-color rn ZERO) (value TRUE))

	(not (plan-requested ?goal-id))
=>
	(bind ?p
	  (smt-create-data
			(smt-create-robots ?team-color)
			(smt-create-machines ?team-color)
			(smt-create-orders ?team-color)
			(smt-create-rings ?team-color)
	  )
	)

	(smt-request "smt-plan" ?p)
	(assert (plan-requested ?goal-id))
)

;---------------------------------------------------------------------------
;  Extract plan generated by clips-smt from protobuf
;---------------------------------------------------------------------------

(defrule production-smt-plan-completed
	; Call plugin clips-smt
	(smt-plan-complete ?handle)

	?g <- (goal (id ?goal-id) (mode SELECTED))
	?plan-req <- (plan-requested ?goal-id)

	(wm-fact (key refbox team-color) (value ?team-color&CYAN|MAGENTA))
	(wm-fact (key domain fact order-complexity args? ord ?order-id com C3) (value TRUE))
	=>
	(printout t "SMT plan handle completed " ?handle  crlf)

	; Create instance of plan
	(bind ?plan-id SMT-PLAN) ; TODO combine with random id number
	(assert
		(plan (id ?plan-id) (goal-id ?goal-id))
	)

	; Import plan generate by plugin clips-smt into ?plans and extract the plan-actions
	(bind ?plans (smt-get-plan ?handle))
	; (printout t "Plan: " (pb-tostring ?plans) crlf)

	; Keep track of position TODO think about multiple robots, should this information be done here or on the plugin-side
	(bind ?from START)
	(bind ?from-side INPUT)

	; Assert plan-action enter-field
	(assert
		 (plan-action (id 99) (plan-id ?plan-id) (duration 4.0)
									(action-name enter-field)
									(param-names r team-color) (param-values R-1 ?team-color))
	)

	; Extract actions inside ActorSpecificPlan -> SequentialPlan -> Actions
	(progn$ (?ap (pb-field-list ?plans "plans"))

		(if (pb-has-field ?ap "sequential_plan") then

			(bind ?p (pb-field-value ?ap "sequential_plan"))
			(bind ?actions (pb-field-list ?p "actions"))

			(loop-for-count (?ai (length$ ?actions))

				(bind ?a (nth$ ?ai ?actions))
				(bind ?actname (pb-field-value ?a "name"))

				(switch ?actname

					;ACTION:::::ENTER-FIELD:::::
					(case "enter-field" then
						(printout warn "Ignoring enter-field, done implicitly" crlf)
					)

					;ACTION::::MOVE::::::
					(case "move" then
						(bind ?action-id (pb-field-value ?a "id"))
						(bind ?next-step-id (* ?action-id 100))
						(bind ?parents-ids (create$))
						(progn$ (?arg (pb-field-list ?a "parent_id"))
							(bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
						)
						(if (pb-has-field ?a "actor")
							then
							(bind ?action-specific-actor (string-to-field (pb-field-value ?a "actor")))
						)
						(progn$ (?arg (pb-field-list ?a "params"))
							(if (eq (pb-field-value ?arg "key") "to") then
								(bind ?to (pb-field-value ?arg "value"))
								(bind ?to-splitted (str-split ?to "-"))
								(bind ?to (str-join "-" (subseq$ ?to-splitted 1 2)))
								(bind ?to-side (if (eq (nth$ 3 ?to-splitted) "I") then INPUT else OUTPUT))
								(bind ?to (string-to-field ?to))
							else
								(printout warn "Unknown parameter " (pb-field-value ?arg "key") " for " ?actname crlf)
							)
						)

						(assert
							(plan-action
								(id ?next-step-id)
								(plan-id ?plan-id)
								(duration 4.0)
								(action-name move)
								(param-names r from from-side to to-side)
								(param-values ?action-specific-actor ?from ?from-side ?to ?to-side)
							)
						)
						(assert
							(wm-fact
								(key plan ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values move ?action-specific-actor ?from ?from-side ?to ?to-side)
							)
						)
						(assert
							(wm-fact
								(key plandep ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values ?parents-ids)
							)
						)

						(printout t "plan-action move added: " ?action-specific-actor " [" ?action-id  "] from: " ?from " at: " ?from-side " to: " ?to " at: " ?to-side crlf)

						; Keep track from where a robot comes from
						(bind ?from ?to)
						(bind ?from-side ?to-side)
					)

					;ACTION:::::WP-GET-SHELF:::::
					(case "wp-get-shelf" then
						(bind ?shelf FALSE)
						(bind ?action-id (pb-field-value ?a "id"))
						(bind ?next-step-id (* ?action-id 100))
						(bind ?parents-ids (create$))
						(progn$ (?arg (pb-field-list ?a "parent_id"))
							(bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
						)
						(if (pb-has-field ?a "actor")
							then
							(bind ?action-specific-actor (string-to-field (pb-field-value ?a "actor")))
						)
						(progn$ (?arg (pb-field-list ?a "params"))
							(if (eq (pb-field-value ?arg "key") "mps") then
								(bind ?mps (pb-field-value ?arg "value"))
								(bind ?mps-splitted (str-split ?mps "-"))
								(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
								(bind ?side (if (eq (nth$ 3 ?mps-splitted) "I") then INPUT else OUTPUT))
								(bind ?mps (string-to-field ?mps))
							else
								(if (eq (pb-field-value ?arg "key") "shelf") then
									(bind ?shelf (string-to-field (pb-field-value ?arg "value")))

								else
									(if (eq (pb-field-value ?arg "key") "wp") then
										(bind ?wp (string-to-field (pb-field-value ?arg "value")))

									else
										(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
									)
								)
							)
						)

						(assert
							(plan-action
								(id ?next-step-id)
								(plan-id ?plan-id)
								(duration 4.0)
								(action-name wp-get-shelf)
								(param-names r cc m spot)
								(param-values ?action-specific-actor ?wp ?mps ?shelf)
							)
						)
						(assert
							(wm-fact
								(key plan ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values wp-get-shelf ?action-specific-actor ?wp ?mps ?shelf)
							)
						)
						(assert
							(wm-fact
								(key plandep ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values ?parents-ids)
							)
						)

						(printout t "plan-action wp-get-shelf added: " ?action-specific-actor " [" ?action-id  "] from: " ?mps " at: " ?side " shelf: " ?shelf crlf)
					)

					;ACTION:::::WP-GET:::::
					(case "wp-get" then
						(bind ?action-id (pb-field-value ?a "id"))
						(bind ?next-step-id (* ?action-id 100))
						(bind ?parents-ids (create$))
						(progn$ (?arg (pb-field-list ?a "parent_id"))
							(bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
						)
						(if (pb-has-field ?a "actor")
							then
							(bind ?action-specific-actor (string-to-field (pb-field-value ?a "actor")))
						)
						(progn$ (?arg (pb-field-list ?a "params"))
							(if (eq (pb-field-value ?arg "key") "mps") then
								(bind ?mps (pb-field-value ?arg "value"))
								(bind ?mps-splitted (str-split ?mps "-"))
								(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
								(bind ?side (if (eq (nth$ 3 ?mps-splitted) "I") then INPUT else OUTPUT))
								(bind ?mps (string-to-field ?mps))
							else
								(if (eq (pb-field-value ?arg "key") "wp") then
									(bind ?wp (string-to-field (pb-field-value ?arg "value")))

								else
									(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
								)
							)
						)

						(assert
							(plan-action
								(id ?next-step-id)
								(plan-id ?plan-id)
								(duration 4.0)
								(action-name wp-get)
								(param-names r wp m side)
								(param-values ?action-specific-actor ?wp ?mps ?side)
							)
						)
						(assert
							(wm-fact
								(key plan ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values wp-get ?action-specific-actor ?wp ?mps ?side)
							)
						)
						(assert
							(wm-fact
								(key plandep ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values ?parents-ids)
							)
						)

						(printout t "plan-action wp-get added: " ?action-specific-actor " [" ?action-id  "] from: " ?mps " side: " ?side crlf)
					)

					;ACTION:::::WP-PUT:::::
					(case "wp-put" then
						(bind ?action-id (pb-field-value ?a "id"))
						(bind ?next-step-id (* ?action-id 100))
						(bind ?parents-ids (create$))
						(progn$ (?arg (pb-field-list ?a "parent_id"))
							(bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
						)
						(bind ?machine-feature CONVEYOR)
						(if (pb-has-field ?a "actor")
							then
							(bind ?action-specific-actor (string-to-field (pb-field-value ?a "actor")))
						)
						(progn$ (?arg (pb-field-list ?a "params"))
							(if (eq (pb-field-value ?arg "key") "mps") then
								(bind ?mps (pb-field-value ?arg "value"))
								(bind ?mps-splitted (str-split ?mps "-"))
								(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
								(bind ?side (if (eq (nth$ 3 ?mps-splitted) "I") then INPUT else OUTPUT))
								(bind ?mps (string-to-field ?mps))
							else
								(if (eq (pb-field-value ?arg "key") "slide") then
									(bind ?slide (pb-field-value ?arg "value"))
									(bind ?machine-feature (if (eq ?slide "true") then SLIDE else CONVEYOR))
								else
									(if (eq (pb-field-value ?arg "key") "wp") then
										(bind ?wp (string-to-field (pb-field-value ?arg "value")))

									else
										(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
									)
								)
							)
						)

						(assert
							(plan-action
								(id ?next-step-id)
								(plan-id ?plan-id)
								(duration 4.0)
								(action-name wp-put)
								(param-names r wp m)
								(param-values ?action-specific-actor ?wp ?mps)
							)
						)
						(assert
							(wm-fact
								(key plan ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values wp-put ?action-specific-actor ?wp ?mps)
							)
						)
						(assert
							(wm-fact
								(key plandep ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values ?parents-ids)
							)
						)

						(printout t "plan-action wp-put added: " ?action-specific-actor " [" ?action-id  "] to: " ?mps " at: " ?side crlf)
					)

					;ACTION:::::WP-PUT-SLIDE-CC:::::
					(case "wp-put-slide-cc" then
						(bind ?action-id (pb-field-value ?a "id"))
						(bind ?next-step-id (* ?action-id 100))
						(bind ?parents-ids (create$))
						(progn$ (?arg (pb-field-list ?a "parent_id"))
							(bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
						)
						(bind ?machine-feature CONVEYOR)
						(if (pb-has-field ?a "actor")
							then
							(bind ?action-specific-actor (string-to-field (pb-field-value ?a "actor")))
						)
						(progn$ (?arg (pb-field-list ?a "params"))
							(if (eq (pb-field-value ?arg "key") "mps") then
								(bind ?mps (pb-field-value ?arg "value"))
								(bind ?mps-splitted (str-split ?mps "-"))
								(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
								(bind ?side (if (eq (nth$ 3 ?mps-splitted) "I") then INPUT else OUTPUT))
								(bind ?mps (string-to-field ?mps))
							else
								(if (eq (pb-field-value ?arg "key") "slide") then
								  (bind ?slide (pb-field-value ?arg "value"))
								  (bind ?machine-feature (if (eq ?slide "true") then SLIDE else CONVEYOR))
								else
									(if (eq (pb-field-value ?arg "key") "rs-before") then
										(bind ?rs-before (string-to-field (pb-field-value ?arg "value")))

									else
										(if (eq (pb-field-value ?arg "key") "rs-after") then
											(bind ?rs-after (string-to-field (pb-field-value ?arg "value")))

										else
											(if (eq (pb-field-value ?arg "key") "wp") then
												 (bind ?wp (string-to-field (pb-field-value ?arg "value")))

											else
												(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
											)
										)
									)
								)
							)
						)
						(assert
							(plan-action
								(id ?next-step-id)
								(plan-id ?plan-id)
								(duration 4.0)
								(action-name wp-put-slide-cc)
								(param-names r wp m rs-before rs-after)
								(param-values ?action-specific-actor ?wp ?mps ?rs-before ?rs-after )
							)
						)
						(assert
							(wm-fact
								(key plan ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values wp-put-slide-cc ?action-specific-actor ?wp ?mps ?rs-before ?rs-after)
							)
						)
						(assert
							(wm-fact
								(key plandep ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values ?parents-ids)
							)
						)

						(printout t "plan-action wp-put-slide-cc added: " ?action-specific-actor " [" ?action-id  "] to: " ?mps " at: " ?side " with before: " ?rs-before " and after: " ?rs-after crlf)
					)

					;ACTION:::::WP-DISCARD:::::
					(case "wp-discard" then
						(bind ?action-id (pb-field-value ?a "id"))
						(bind ?next-step-id (* ?action-id 100))
						(bind ?parents-ids (create$))
						(progn$ (?arg (pb-field-list ?a "parent_id"))
							(bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
						)
						(if (pb-has-field ?a "actor")
							then
							(bind ?action-specific-actor (string-to-field (pb-field-value ?a "actor")))
						)
						(progn$ (?arg (pb-field-list ?a "params"))
							(if (eq (pb-field-value ?arg "key") "wp") then
								(bind ?wp (string-to-field (pb-field-value ?arg "value")))

							else
								(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
							)
						)

						(assert
							(plan-action
								(id ?next-step-id)
								(plan-id ?plan-id)
								(duration 4.0)
								(action-name wp-discard)
								(param-names r cc)
								(param-values ?action-specific-actor ?wp)
							)
						)
						(assert
							(wm-fact
								(key plan ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values wp-discard ?action-specific-actor ?wp)
							)
						)
						(assert
							(wm-fact
								(key plandep ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values ?parents-ids)
							)
						)

						(printout t "plan-action wp-discard added: " ?action-specific-actor " [" ?action-id  "]" crlf)
					)

					;ACTION:::::PREPARE-BS:::::
					(case "prepare-bs" then
						(bind ?action-id (pb-field-value ?a "id"))
						(bind ?next-step-id (* ?action-id 100))
						(bind ?parents-ids (create$))
						(progn$ (?arg (pb-field-list ?a "parent_id"))
							(bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
						)
						(if (pb-has-field ?a "actor")
							then
							(bind ?action-specific-actor (string-to-field (pb-field-value ?a "actor")))
						)
						(progn$ (?arg (pb-field-list ?a "params"))
							(if (eq (pb-field-value ?arg "key") "mps") then
								(bind ?mps (pb-field-value ?arg "value"))
								(bind ?mps-splitted (str-split ?mps "-"))
								(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
								(bind ?side (if (eq (nth$ 3 ?mps-splitted) "I") then INPUT else OUTPUT))
								(bind ?mps (string-to-field ?mps))
							else
								(if (eq (pb-field-value ?arg "key") "color") then
									(bind ?goal-base-color (string-to-field (pb-field-value ?arg "value")))

								else
									(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
								)
							)
						)

						(assert
							(plan-action
								(id ?next-step-id)
								(plan-id ?plan-id)
								(duration 4.0)
								(action-name prepare-bs)
								(param-names m side bc)
								(param-values ?mps ?side ?goal-base-color)
							)
						)
						(assert
							(wm-fact
								(key plan ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values prepare-bs ?mps ?side ?goal-base-color)
							)
						)
						(assert
							(wm-fact
								(key plandep ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values ?parents-ids)
							)
						)

						(printout t "plan-action prepare-bs added: " ?action-specific-actor " [" ?action-id  "] with base-color: " ?goal-base-color " at: " ?side crlf)
					)

					;ACTION:::::BS-DISPENSE:::::
					(case "bs-dispense" then
						(bind ?action-id (pb-field-value ?a "id"))
						(bind ?next-step-id (* ?action-id 100))
						(bind ?parents-ids (create$))
						(progn$ (?arg (pb-field-list ?a "parent_id"))
							(bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
						)
						(if (pb-has-field ?a "actor")
							then
							(bind ?action-specific-actor (string-to-field (pb-field-value ?a "actor")))
						)
						(progn$ (?arg (pb-field-list ?a "params"))
							(if (eq (pb-field-value ?arg "key") "mps") then
								(bind ?mps (pb-field-value ?arg "value"))
								(bind ?mps-splitted (str-split ?mps "-"))
								(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
								(bind ?side (if (eq (nth$ 3 ?mps-splitted) "I") then INPUT else OUTPUT))
								(bind ?mps (string-to-field ?mps))
							else
								(if (eq (pb-field-value ?arg "key") "color") then
									(bind ?goal-base-color (string-to-field (pb-field-value ?arg "value")))

								else
									(if (eq (pb-field-value ?arg "key") "wp") then
										(bind ?wp (string-to-field (pb-field-value ?arg "value")))

									else
										(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
									)
								)
							)
						)

						(assert
							(plan-action
								(id ?next-step-id)
								(plan-id ?plan-id)
								(duration 4.0)
								(action-name bs-dispense)
								(param-names m side wp basecol)
								(param-values ?mps ?side ?wp ?goal-base-color)
							)
						)
						(assert
							(wm-fact
								(key plan ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values bs-dispense ?mps ?side ?wp ?goal-base-color)
							)
						)
						(assert
							(wm-fact
								(key plandep ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values ?parents-ids)
							)
						)
						(printout t "plan-action bs-dispense added: " ?action-specific-actor " [" ?action-id  "] with basecolor: " ?goal-base-color crlf)
					)

					;ACTION:::::PREPARE-DS:::::
					(case "prepare-ds" then
						(bind ?action-id (pb-field-value ?a "id"))
						(bind ?next-step-id (* ?action-id 100))
						(bind ?parents-ids (create$))
						(progn$ (?arg (pb-field-list ?a "parent_id"))
							(bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
						)
						(if (pb-has-field ?a "actor")
							then
							(bind ?action-specific-actor (string-to-field (pb-field-value ?a "actor")))
						)
						(progn$ (?arg (pb-field-list ?a "params"))
							(if (eq (pb-field-value ?arg "key") "mps") then
								(bind ?mps (pb-field-value ?arg "value"))
								(bind ?mps-splitted (str-split ?mps "-"))
								(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
								(bind ?mps (string-to-field ?mps))

							else
								(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
							)
						)

						(do-for-fact ((?wm-fact wm-fact))
							(and
								(wm-key-prefix ?wm-fact:key (create$ domain fact order-gate))
								(eq ?order-id (wm-key-arg ?wm-fact:key ord))
							)

							(assert
								(plan-action
									(id ?next-step-id)
									(plan-id ?plan-id)
									(duration 4.0)
									(action-name prepare-ds)
									(param-names m gate)
									(param-values ?mps (wm-key-arg ?wm-fact:key gate))
								)
							)
							(assert
								(wm-fact
									(key plan ?plan-id ?goal-id ?next-step-id)
									(is-list TRUE)
									(values prepare-ds ?mps (wm-key-arg ?wm-fact:key gate))
								)
							)
							(assert
								(wm-fact
									(key plandep ?plan-id ?goal-id ?next-step-id)
									(is-list TRUE)
									(values ?parents-ids)
								)
							)
						)
						(printout t "plan-action prepare-ds added: " ?action-specific-actor " [" ?action-id  "]" crlf)
					)

					;ACTION:::::FULFILL-ORDER-C0:::::
					(case "fulfill-order-c0" then
						(bind ?action-id (pb-field-value ?a "id"))
						(bind ?next-step-id (* ?action-id 100))
						(bind ?parents-ids (create$))
						(progn$ (?arg (pb-field-list ?a "parent_id"))
							(bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
						)
						(progn$ (?arg (pb-field-list ?a "params"))
							(if (eq (pb-field-value ?arg "key") "mps") then
								(bind ?mps (pb-field-value ?arg "value"))
								(bind ?mps-splitted (str-split ?mps "-"))
								(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
								(bind ?mps (string-to-field ?mps))

							else
								(if (eq (pb-field-value ?arg "key") "base-color") then
									(bind ?base-color (string-to-field (pb-field-value ?arg "value")))

								else
									(if (eq (pb-field-value ?arg "key") "cap-color") then
										(bind ?cap-color (string-to-field (pb-field-value ?arg "value")))

									else
										(if (eq (pb-field-value ?arg "key") "wp") then
											(bind ?wp (string-to-field (pb-field-value ?arg "value")))

										else
											(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
										)
									)
								)
							)
						)

						; Extract information order-gate of the corresponding order with id order-id from fact-base
						; TODO Extract information from plan of clips-smt plugin via protobuf
						(do-for-fact ((?wm-fact wm-fact))
							(and
								(wm-key-prefix ?wm-fact:key (create$ domain fact order-gate))
								(eq ?order-id (wm-key-arg ?wm-fact:key ord))
							)

							(assert
								(plan-action
									(id ?next-step-id)
									(plan-id ?plan-id)
									(duration 4.0)
									(action-name fulfill-order-c0)
									(param-names ord wp m g basecol capcol)
									(param-values ?order-id ?wp ?mps (wm-key-arg ?wm-fact:key gate) ?base-color ?cap-color)
								)
							)
							(assert
								(wm-fact
									(key plan ?plan-id ?goal-id ?next-step-id)
									(is-list TRUE)
									(values fulfill-order-c0 ?order-id ?wp ?mps (wm-key-arg ?wm-fact:key gate) ?base-color ?cap-color)
								)
							)
							(assert
								(wm-fact
									(key plandep ?plan-id ?goal-id ?next-step-id)
									(is-list TRUE)
									(values ?parents-ids)
								)
							)
						)

						(printout t "plan-action fulfill-order-c0 added: " ?action-specific-actor " [" ?action-id  "] " ?order-id crlf)
					)

					;ACTION:::::FULFILL-ORDER-C1:::::
					(case "fulfill-order-c1" then
						(bind ?action-id (pb-field-value ?a "id"))
						(bind ?next-step-id (* ?action-id 100))
						(bind ?parents-ids (create$))
						(progn$ (?arg (pb-field-list ?a "parent_id"))
							(bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
						)
						(progn$ (?arg (pb-field-list ?a "params"))
							(if (eq (pb-field-value ?arg "key") "mps") then
								(bind ?mps (pb-field-value ?arg "value"))
								(bind ?mps-splitted (str-split ?mps "-"))
								(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
								(bind ?mps (string-to-field ?mps))
							else
								(if (eq (pb-field-value ?arg "key") "base-color") then
									(bind ?base-color (string-to-field (pb-field-value ?arg "value")))

								else
									(if (eq (pb-field-value ?arg "key") "ring1-color") then
										(bind ?ring1-color (string-to-field (pb-field-value ?arg "value")))

									else
										(if (eq (pb-field-value ?arg "key") "cap-color") then
											(bind ?cap-color (string-to-field (pb-field-value ?arg "value")))

										else
											(if (eq (pb-field-value ?arg "key") "wp") then
												(bind ?wp (string-to-field (pb-field-value ?arg "value")))

											else
												(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
											)
										)
									)
								)
							)
						)

						(do-for-fact ((?wm-fact wm-fact))
							(and
								(wm-key-prefix ?wm-fact:key (create$ domain fact order-gate))
								(eq ?order-id (wm-key-arg ?wm-fact:key ord))
							)

							(assert
								(plan-action
									(id ?next-step-id)
									(plan-id ?plan-id)
									(duration 4.0)
									(action-name fulfill-order-c1)
									(param-names ord wp m g basecol capcol ring1col)
									(param-values ?order-id ?wp ?mps (wm-key-arg ?wm-fact:key gate) ?base-color ?cap-color ?ring1-color)
								)
							)
							(assert
								(wm-fact
									(key plan ?plan-id ?goal-id ?next-step-id)
									(is-list TRUE)
									(values fulfill-order-c1 ?order-id ?wp ?mps (wm-key-arg ?wm-fact:key gate) ?base-color ?cap-color ?ring1-color)
								)
							)
							(assert
								(wm-fact
									(key plandep ?plan-id ?goal-id ?next-step-id)
									(is-list TRUE)
									(values ?parents-ids)
								)
							)
						)

						(printout t "plan-action fulfill-order-c1 added: " ?action-specific-actor " [" ?action-id  "] " ?order-id crlf)
					)

					;ACTION:::::FULFILL-ORDER-C2:::::
					(case "fulfill-order-c2" then
						(bind ?action-id (pb-field-value ?a "id"))
						(bind ?next-step-id (* ?action-id 100))
						(bind ?parents-ids (create$))
						(progn$ (?arg (pb-field-list ?a "parent_id"))
							(bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
						)
						(progn$ (?arg (pb-field-list ?a "params"))
							(if (eq (pb-field-value ?arg "key") "mps") then
								(bind ?mps (pb-field-value ?arg "value"))
								(bind ?mps-splitted (str-split ?mps "-"))
								(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
								(bind ?mps (string-to-field ?mps))
							else
								(if (eq (pb-field-value ?arg "key") "base-color") then
									(bind ?base-color (string-to-field (pb-field-value ?arg "value")))

								else
									(if (eq (pb-field-value ?arg "key") "ring1-color") then
										(bind ?ring1-color (string-to-field (pb-field-value ?arg "value")))

									else
										(if (eq (pb-field-value ?arg "key") "ring2-color") then
											(bind ?ring2-color (string-to-field (pb-field-value ?arg "value")))

										else
											(if (eq (pb-field-value ?arg "key") "cap-color") then
												(bind ?cap-color (string-to-field (pb-field-value ?arg "value")))

											else
												(if (eq (pb-field-value ?arg "key") "wp") then
													(bind ?wp (string-to-field (pb-field-value ?arg "value")))

												else
													(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
												)
											)
										)
									)
								)
							)
						)

						(do-for-fact ((?wm-fact wm-fact))
							(and
								(wm-key-prefix ?wm-fact:key (create$ domain fact order-gate))
								(eq ?order-id (wm-key-arg ?wm-fact:key ord))
							)

							(assert
								(plan-action
									(id ?next-step-id)
									(plan-id ?plan-id)
									(duration 4.0)
									(action-name fulfill-order-c2)
									(param-names ord wp m g basecol capcol ring1col ring2col)
									(param-values ?order-id ?wp ?mps (wm-key-arg ?wm-fact:key gate) ?base-color ?cap-color ?ring1-color ?ring2-color)
								)
							)
							(assert
								(wm-fact
									(key plan ?plan-id ?goal-id ?next-step-id)
									(is-list TRUE)
									(values fulfill-order-c2 ?order-id ?wp ?mps (wm-key-arg ?wm-fact:key gate) ?base-color ?cap-color ?ring1-color ?ring2-color)
								)
							)
							(assert
								(wm-fact
									(key plandep ?plan-id ?goal-id ?next-step-id)
									(is-list TRUE)
									(values ?parents-ids)
								)
							)
						)

						(printout t "plan-action fulfill-order-c2 added: " ?action-specific-actor " [" ?action-id  "] " ?order-id crlf)
					)

					;ACTION:::::FULFILL-ORDER-C3:::::
					(case "fulfill-order-c3" then
						(bind ?action-id (pb-field-value ?a "id"))
						(bind ?next-step-id (* ?action-id 100))
						(bind ?parents-ids (create$))
						(progn$ (?arg (pb-field-list ?a "parent_id"))
							(bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
						)
						(progn$ (?arg (pb-field-list ?a "params"))
							(if (eq (pb-field-value ?arg "key") "mps") then
								(bind ?mps (pb-field-value ?arg "value"))
								(bind ?mps-splitted (str-split ?mps "-"))
								(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
								(bind ?mps (string-to-field ?mps))

							 else
								(if (eq (pb-field-value ?arg "key") "base-color") then
								  (bind ?base-color (string-to-field (pb-field-value ?arg "value")))

								else
									(if (eq (pb-field-value ?arg "key") "ring1-color") then
									  (bind ?ring1-color (string-to-field (pb-field-value ?arg "value")))

									else
										(if (eq (pb-field-value ?arg "key") "ring2-color") then
										  (bind ?ring2-color (string-to-field (pb-field-value ?arg "value")))

										else
											(if (eq (pb-field-value ?arg "key") "ring3-color") then
											  (bind ?ring3-color (string-to-field (pb-field-value ?arg "value")))

											else
												(if (eq (pb-field-value ?arg "key") "cap-color") then
												  (bind ?cap-color (string-to-field (pb-field-value ?arg "value")))

												else
													(if (eq (pb-field-value ?arg "key") "wp") then
													  (bind ?wp (string-to-field (pb-field-value ?arg "value")))

													else
														(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
													)
												)
											)
										)
									)
								)
							)
						)

						(do-for-fact ((?wm-fact wm-fact))
							(and
								(wm-key-prefix ?wm-fact:key (create$ domain fact order-gate))
								(eq ?order-id (wm-key-arg ?wm-fact:key ord))
							)

							(assert
								(plan-action
									(id ?next-step-id)
									(plan-id ?plan-id)
									(duration 4.0)
									(action-name fulfill-order-c3)
									(param-names ord wp m g basecol capcol ring1col ring2col ring3col)
									(param-values ?order-id ?wp ?mps (wm-key-arg ?wm-fact:key gate) ?base-color ?cap-color ?ring1-color ?ring2-color ?ring3-color)
								)
							)
							(assert
								(wm-fact
									(key plan ?plan-id ?goal-id ?next-step-id)
									(is-list TRUE)
									(values fulfill-order-c3 ?order-id ?wp ?mps (wm-key-arg ?wm-fact:key gate) ?base-color ?cap-color ?ring1-color ?ring2-color ?ring3-color)
								)
							)
							(assert
								(wm-fact
									(key plandep ?plan-id ?goal-id ?next-step-id)
									(is-list TRUE)
									(values ?parents-ids)
								)
							)
						)

						(printout t "plan-action fulfill-order-c3 added: " ?action-specific-actor " [" ?action-id  "] " ?order-id crlf)
				  )

					;ACTION:::::PREPARE-CS:::::
					(case "prepare-cs" then
						(bind ?action-id (pb-field-value ?a "id"))
						(bind ?next-step-id (* ?action-id 100))
						(bind ?parents-ids (create$))
						(progn$ (?arg (pb-field-list ?a "parent_id"))
							(bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
						)
						(if (pb-has-field ?a "actor")
							then
							(bind ?action-specific-actor (string-to-field (pb-field-value ?a "actor")))
						)
						(progn$ (?arg (pb-field-list ?a "params"))
							(if (eq (pb-field-value ?arg "key") "mps") then
								(bind ?mps (pb-field-value ?arg "value"))
								(bind ?mps-splitted (str-split ?mps "-"))
								(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
								(bind ?mps (string-to-field ?mps))
							else
								(if (eq (pb-field-value ?arg "key") "operation") then
									(bind ?operation (string-to-field (pb-field-value ?arg "value")))

								else
									(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
								)
							)
						)

						(assert
							(plan-action
								(id ?next-step-id)
								(plan-id ?plan-id)
								(duration 4.0)
								(action-name prepare-cs)
								(param-names m op)
								(param-values ?mps ?operation)
							)
						)
						(assert
							(wm-fact
								(key plan ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values prepare-cs ?mps ?operation)
							)
						)
						(assert
							(wm-fact
								(key plandep ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values ?parents-ids)
							)
						)

						(printout t "plan-action prepare-cs added: " ?action-specific-actor " [" ?action-id  "] at: " ?mps " with " ?operation crlf)
					)

					;ACTION:::::CS-RETRIEVE-CAP:::::
					(case "cs-retrieve-cap" then
						(bind ?action-id (pb-field-value ?a "id"))
						(bind ?next-step-id (* ?action-id 100))
						(bind ?parents-ids (create$))
						(progn$ (?arg (pb-field-list ?a "parent_id"))
							(bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
						)
						(progn$ (?arg (pb-field-list ?a "params"))
							(if (eq (pb-field-value ?arg "key") "mps") then
								(bind ?mps (pb-field-value ?arg "value"))
								(bind ?mps-splitted (str-split ?mps "-"))
								(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
								(bind ?mps (string-to-field ?mps))
							else
								(if (eq (pb-field-value ?arg "key") "cap-color") then
									(bind ?cap-color (string-to-field (pb-field-value ?arg "value")))

								else
									(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
								)
							)
						)

						(assert
							(plan-action
								(id ?next-step-id)
								(plan-id ?plan-id)
								(duration 4.0)
								(action-name cs-retrieve-cap)
								(param-names m cc capcol)
								(param-values ?mps ?wp ?cap-color)
							)
						)
						(assert
							(wm-fact
								(key plan ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values cs-retrieve-cap ?mps ?wp ?cap-color)
							)
						)
						(assert
							(wm-fact
								(key plandep ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values ?parents-ids)
							)
						)

						(printout t "plan-action cs-retrieve-cap added: " ?action-specific-actor " [" ?action-id  "] at: " ?mps crlf)
					)

					;ACTION:::::CS-MOUNT-CAP:::::
					(case "cs-mount-cap" then
						(bind ?action-id (pb-field-value ?a "id"))
						(bind ?next-step-id (* ?action-id 100))
						(bind ?parents-ids (create$))
						(progn$ (?arg (pb-field-list ?a "parent_id"))
							(bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
						)
						(progn$ (?arg (pb-field-list ?a "params"))
							(if (eq (pb-field-value ?arg "key") "mps") then
								(bind ?mps (pb-field-value ?arg "value"))
								(bind ?mps-splitted (str-split ?mps "-"))
								(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
								(bind ?mps (string-to-field ?mps))
							else
								(if (eq (pb-field-value ?arg "key") "cap-color") then
									(bind ?cap-color (string-to-field (pb-field-value ?arg "value")))

								else
									(if (eq (pb-field-value ?arg "key") "wp") then
									  (bind ?wp (string-to-field (pb-field-value ?arg "value")))

									else
										(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
									)
								)
							)
						)

						(assert
							(plan-action
								(id ?next-step-id)
								(plan-id ?plan-id)
								(duration 4.0)
								(action-name cs-mount-cap)
								(param-names m wp capcol)
								(param-values ?mps ?wp ?cap-color)
							)
						)
						(assert
							(wm-fact
								(key plan ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values cs-mount-cap ?mps ?wp ?cap-color)
							)
						)
						(assert
							(wm-fact
								(key plandep ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values ?parents-ids)
							)
						)

						(printout t "plan-action cs-mount-cap added: " ?action-specific-actor " [" ?action-id  "] at: " ?mps crlf)
					)

					;ACTION:::::PREPARE-RS::::::
					(case "prepare-rs" then
						(bind ?action-id (pb-field-value ?a "id"))
						(bind ?next-step-id (* ?action-id 100))
						(bind ?parents-ids (create$))
						(progn$ (?arg (pb-field-list ?a "parent_id"))
							(bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
						)
						(if (pb-has-field ?a "actor")
							then
							(bind ?action-specific-actor (string-to-field (pb-field-value ?a "actor")))
						)
						(progn$ (?arg (pb-field-list ?a "params"))
							(if (eq (pb-field-value ?arg "key") "mps") then
								(bind ?mps (pb-field-value ?arg "value"))
								(bind ?mps-splitted (str-split ?mps "-"))
								(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
								(bind ?mps (string-to-field ?mps))
							else
								(if (eq (pb-field-value ?arg "key") "ring_color") then
										(bind ?goal-ring-color (string-to-field (pb-field-value ?arg "value")) ) ;temp: the color of the base of the goal is recognized here

								else
									(if (eq (pb-field-value ?arg "key") "rs-before") then
									  (bind ?rs-before (string-to-field (pb-field-value ?arg "value")))

									else
										(if (eq (pb-field-value ?arg "key") "rs-after") then
										  (bind ?rs-after (string-to-field (pb-field-value ?arg "value")))

										else
											(if (eq (pb-field-value ?arg "key") "r-req") then
											  (bind ?r-req (string-to-field (pb-field-value ?arg "value")))

											else
												(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
											)
										)
									)
								)
							)
						)

						(assert
							(plan-action
								(id ?next-step-id)
								(plan-id ?plan-id)
								(duration 4.0)
								(action-name prepare-rs)
								(param-names m rc rs-before rs-after r-req)
								(param-values ?mps ?goal-ring-color ?rs-before ?rs-after ?r-req))
						)
						(assert
							(wm-fact
								(key plan ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values prepare-rs ?mps ?goal-ring-color ?rs-before ?rs-after ?r-req)
							)
						)
						(assert
							(wm-fact
								(key plandep ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values ?parents-ids)
							)
						)

						(printout t "plan-action prepare-rs added: " ?action-specific-actor " [" ?action-id  "] at: " ?mps " with ring-color: " ?goal-ring-color crlf)
					  )

					;ACTION:::::RS-MOUNT-RING1:::::
					(case "rs-mount-ring1" then
						(bind ?action-id (pb-field-value ?a "id"))
						(bind ?next-step-id (* ?action-id 100))
						(bind ?parents-ids (create$))
						(progn$ (?arg (pb-field-list ?a "parent_id"))
							(bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
						)
						(progn$ (?arg (pb-field-list ?a "params"))
							(if (eq (pb-field-value ?arg "key") "mps") then
								(bind ?mps (pb-field-value ?arg "value"))
								(bind ?mps-splitted (str-split ?mps "-"))
								(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
								(bind ?mps (string-to-field ?mps))
							else
								(if (eq (pb-field-value ?arg "key") "ring-color") then
									(bind ?ring-color (string-to-field (pb-field-value ?arg "value")))

								else
									(if (eq (pb-field-value ?arg "key") "rs-before") then
									  (bind ?rs-before (string-to-field (pb-field-value ?arg "value")))

									else
										(if (eq (pb-field-value ?arg "key") "rs-after") then
										  (bind ?rs-after (string-to-field (pb-field-value ?arg "value")))

										else
											(if (eq (pb-field-value ?arg "key") "r-req") then
											  (bind ?r-req (string-to-field (pb-field-value ?arg "value")))

											else
												(if (eq (pb-field-value ?arg "key") "wp") then
												  (bind ?wp (string-to-field (pb-field-value ?arg "value")))

												else
													(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
												)
											)
										)
									)
								)
							)
						)

						(assert
							(plan-action
								(id ?next-step-id)
								(plan-id ?plan-id)
								(duration 4.0)
								(action-name rs-mount-ring1)
								(param-names m wp col rs-before rs-after r-req)
								(param-values ?mps ?wp ?ring-color ?rs-before ?rs-after ?r-req)
							)
						)
						(assert
							(wm-fact
								(key plan ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values rs-mount-ring1 ?mps ?wp ?ring-color ?rs-before ?rs-after ?r-req)
							)
						)
						(assert
							(wm-fact
								(key plandep ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values ?parents-ids)
							)
						)

						(printout t "plan-action rs-mount-ring1 added: " ?action-specific-actor " [" ?action-id  "] at: " ?mps " with ring-color: " ?ring-color crlf)
					)

					;ACTION:::::RS-MOUNT-RING2:::::
					(case "rs-mount-ring2" then
						(bind ?action-id (pb-field-value ?a "id"))
						(bind ?next-step-id (* ?action-id 100))
						(bind ?parents-ids (create$))
						(progn$ (?arg (pb-field-list ?a "parent_id"))
							(bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
						)
						(progn$ (?arg (pb-field-list ?a "params"))
							(if (eq (pb-field-value ?arg "key") "mps") then
								(bind ?mps (pb-field-value ?arg "value"))
								(bind ?mps-splitted (str-split ?mps "-"))
								(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
								(bind ?mps (string-to-field ?mps))
							else
								(if (eq (pb-field-value ?arg "key") "ring-color") then
								  (bind ?ring-color (string-to-field (pb-field-value ?arg "value")))

								else
									(if (eq (pb-field-value ?arg "key") "rs-before") then
										(bind ?rs-before (string-to-field (pb-field-value ?arg "value")))

									else
										(if (eq (pb-field-value ?arg "key") "rs-after") then
											(bind ?rs-after (string-to-field (pb-field-value ?arg "value")))

										else
											(if (eq (pb-field-value ?arg "key") "r-req") then
												(bind ?r-req (string-to-field (pb-field-value ?arg "value")))

											else
												(if (eq (pb-field-value ?arg "key") "col1") then
													(bind ?col1 (string-to-field (pb-field-value ?arg "value")))

												else
													(if (eq (pb-field-value ?arg "key") "wp") then
														(bind ?wp (string-to-field (pb-field-value ?arg "value")))

													else
														(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
													)
												)
											)
										)
									)
								)
							)
						)

						(assert
							(plan-action
								(id ?next-step-id)
								(plan-id ?plan-id)
								(duration 4.0)
								(action-name rs-mount-ring2)
								(param-names m wp col col1 rs-before rs-after r-req)
								(param-values ?mps ?wp ?ring-color ?col1 ?rs-before ?rs-after ?r-req )
							)
						)
						(assert
							(wm-fact
								(key plan ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values rs-mount-ring2 ?mps ?wp ?ring-color ?col1 ?rs-before ?rs-after ?r-req)
							)
						)
						(assert
							(wm-fact
								(key plandep ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values ?parents-ids)
							)
						)

						(printout t "plan-action rs-mount-ring2 added: " ?action-specific-actor " [" ?action-id  "] at: " ?mps " with ring-color: " ?ring-color crlf)
					)

					;ACTION:::::RS-MOUNT-RING3:::::
					(case "rs-mount-ring3" then
						(bind ?action-id (pb-field-value ?a "id"))
						(bind ?next-step-id (* ?action-id 100))
						(bind ?parents-ids (create$))
						(progn$ (?arg (pb-field-list ?a "parent_id"))
							(bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
						)
						(progn$ (?arg (pb-field-list ?a "params"))
							(if (eq (pb-field-value ?arg "key") "mps") then
								(bind ?mps (pb-field-value ?arg "value"))
								(bind ?mps-splitted (str-split ?mps "-"))
								(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
								(bind ?mps (string-to-field ?mps))
							else
								(if (eq (pb-field-value ?arg "key") "ring-color") then
								  (bind ?ring-color (string-to-field (pb-field-value ?arg "value")))

								else
									(if (eq (pb-field-value ?arg "key") "rs-before") then
										(bind ?rs-before (string-to-field (pb-field-value ?arg "value")))

									else
										(if (eq (pb-field-value ?arg "key") "rs-after") then
											(bind ?rs-after (string-to-field (pb-field-value ?arg "value")))

										else
											(if (eq (pb-field-value ?arg "key") "r-req") then
												(bind ?r-req (string-to-field (pb-field-value ?arg "value")))

											else
												(if (eq (pb-field-value ?arg "key") "col1") then
													(bind ?col1 (string-to-field (pb-field-value ?arg "value")))

												else
													(if (eq (pb-field-value ?arg "key") "col2") then
														(bind ?col2 (string-to-field (pb-field-value ?arg "value")))

													else
														(if (eq (pb-field-value ?arg "key") "wp") then
															(bind ?wp (string-to-field (pb-field-value ?arg "value")))

														else
															(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
														)
													)
												)
											)
										)
									)
								)
							)
						)
						(assert
							(plan-action
								(id ?next-step-id)
								(plan-id ?plan-id)
								(duration 4.0)
								(action-name rs-mount-ring3)
								(param-names m wp col col1 col2 rs-before rs-after r-req)
								(param-values ?mps ?wp ?ring-color ?col1 ?col2 ?rs-before ?rs-after ?r-req)
							)
						)
						(assert
							(wm-fact
								(key plan ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values rs-mount-ring3 ?mps ?wp ?ring-color ?col1 ?col2 ?rs-before ?rs-after ?r-req)
							)
						)
						(assert
							(wm-fact
								(key plandep ?plan-id ?goal-id ?next-step-id)
								(is-list TRUE)
								(values ?parents-ids)
							)
						)

						(printout t "plan-action rs-mount-ring3 added: " ?action-specific-actor " [" ?action-id  "] at: " ?mps " with ring-color: " ?ring-color crlf)
					)

					;ACTION:::::DEFAULT:::::
					(default (printout warn "Unknown action " ?actname crlf))
				)
			)

		else
			(printout warn "Sequential plan not set on ActorSpecificPlan" crlf)
		)
	)
	(pb-destroy ?plans)
	(modify ?g (mode EXPANDED))
	(retract ?plan-req)
)
