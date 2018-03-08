
;---------------------------------------------------------------------------
;  smt-planenr.clp - Planning for RCLL production with the plugin clips-smt
;
;  Created: Tue Feb 20 16:406 2018 (Cologne)
;  Copyright  2018  Igor Nicolai Bongartz
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deffunction plan-create-actgroup ($?plans)
	"Create ActorGroupPlan plan"
	(bind ?ap (pb-create "llsf_msgs.ActorGroupPlan"))
	(foreach ?p ?plans
		(pb-add-list ?ap "plans" ?p)
	)
	(return ?ap)
)

(deffunction plan-create-actspec-sequential (?actor-name ?plan)
	"Create ActorSpecificPlan plan from a sequential plan"
	(bind ?ap (pb-create "llsf_msgs.ActorSpecificPlan"))
	(pb-set-field ?ap "actor_name" ?actor-name)
	(pb-set-field ?ap "sequential_plan" ?plan)
	(return ?ap)
)

(deffunction plan-create-sequential ($?actions)
	"Create a sequential plan given a number of actions"
	(bind ?p (pb-create "llsf_msgs.SequentialPlan"))
	(foreach ?a ?actions
		(pb-add-list ?p "actions" ?a)
	)
	(return ?p)
)

; Parameters is a multi-field which must have an even number of
; arguments which are ordered [key, value, ...]
(deffunction plan-create-action (?name $?params)
	"Create a PlanAction for a given ?name with specified parameters."
	(bind ?a (pb-create "llsf_msgs.PlanAction"))
	(pb-set-field ?a "name" ?name)
	(if (neq (mod (length$ ?params) 2) 0)
	 then
		(printout error "Arguments for action " ?name " are malformed" crlf)
		(return ?a)
	)
	(loop-for-count (?i 1 (/ (length$ ?params) 2))
		(bind ?par (pb-create "llsf_msgs.PlanActionParameter"))
		(pb-set-field ?par "key" (nth$ (- (* ?i 2) 1) ?params))
		(pb-set-field ?par "value" (nth$ (* ?i 2) ?params))
		(pb-add-list ?a "params" ?par)
	)
	(return ?a)
)

(deffunction smt-create-data (?robots ?addrobots ?machines ?orders ?rings)
	(bind ?p (pb-create "llsf_msgs.ClipsSmtData"))
	(foreach ?r ?robots
		(pb-add-list ?p "robots" ?r)
	)
	(foreach ?ar ?addrobots
		(pb-add-list ?p "robots" ?ar)
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
  
    
	(printout t "Proto:" (pb-tostring ?p) crlf)
	(return ?p)
)

; Add robot information from worldmodel to protobuf
(deffunction smt-create-robot (?name ?team-color ?number ?pose-x ?pose-y)
	(bind ?r (pb-create "llsf_msgs.Robot"))
	(pb-set-field ?r "name" ?name)
	(pb-set-field ?r "team_color" ?team-color)
	(pb-set-field ?r "number" 1) ; TODO adapt to constant ?*ROBOT-NUMBER*)
	(bind ?pose (pb-create "llsf_msgs.Pose2D"))
	(pb-set-field ?pose "x" ?pose-x)
	(pb-set-field ?pose "y" ?pose-y)
	(pb-set-field ?pose "ori" 0.0)
	(pb-set-field ?r "pose" ?pose)
	(return ?r)
)

(deffunction smt-create-additional-robots (?team-color)
	(bind ?rv (create$))
	; TODO Add additional-robots
	; (do-for-all-facts ((?p pose)) TRUE
	;     (bind ?rv (append$ ?rv (smt-create-robot ?p:name ?team-color 0 ?p:x ?p:y)))
	; )
	(return ?rv)
)

(deffunction smt-create-robots (?team-color)
	(bind ?rv (create$))
	(do-for-all-facts ((?wm-fact wm-fact)) (wm-key-prefix ?wm-fact:key (create$ domain fact robot-waiting))
		(bind ?rv (append$ ?rv (smt-create-robot (wm-key-arg ?wm-fact:key r) ?team-color 0 0 0))) ; TODO Add correct pose of robot
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
	(bind ?pose (pb-create "llsf_msgs.Pose2D"))
	(pb-set-field ?pose "x" ?pose-x)
	(pb-set-field ?pose "y" ?pose-y)
	(pb-set-field ?pose "ori" 0.0)
	(pb-set-field ?m "pose" ?pose)

   ; set available rings for ring-stations
    (if (eq ?mtype RS) then
	  (bind ?rlist (create$))
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
  (printout t "Creating Data msgs:: Order with id " ?id  crlf)
  (bind ?o (pb-create "llsf_msgs.Order"))
	(pb-set-field ?o "id" ?id)
	(pb-set-field ?o "delivery_gate" ?gate)
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
	(do-for-all-facts ((?do domain-object)) (eq ?do:type order)
		(do-for-fact ((?wm-fact wm-fact)) 
			(and
				(wm-key-prefix ?wm-fact:key (create$ domain fact order-gate))
				(eq ?do:name (wm-key-arg ?wm-fact:key ord))
			)
			(do-for-fact ((?wm-fact2 wm-fact)) 
				(and
					(wm-key-prefix ?wm-fact2:key (create$ domain fact order-complexity))
					(eq ?do:name (wm-key-arg ?wm-fact2:key ord))
				)
				(do-for-fact ((?wm-fact3 wm-fact)) (eq ?wm-fact3:key (create$ refbox order ?do:name quantity-requested))
					(do-for-fact ((?wm-fact4 wm-fact)) (eq ?wm-fact4:key (create$ refbox order ?do:name quantity-delivered ?team-color))
						(do-for-fact ((?wm-fact5 wm-fact)) (eq ?wm-fact5:key (create$ refbox order ?do:name delivery-begin))
							(do-for-fact ((?wm-fact6 wm-fact)) (eq ?wm-fact6:key (create$ refbox order ?do:name delivery-end))
								(bind ?rv (append$ ?rv (smt-create-order
															?do:name ; order-id
															(wm-key-arg ?wm-fact:key gate) ; TODO Add the correct gate information
															(wm-key-arg ?wm-fact2:key com)
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
	)
	(return ?rv)
)

; Add ring information from worldmodel to protobuf
(deffunction smt-create-ring (?ring-color ?req-bases)
  (printout t "Creating Data msgs:: Ring color " ?ring-color  crlf)
  (bind ?r (pb-create "llsf_msgs.Ring"))
  (pb-set-field ?r "raw_material" ?req-bases)
  (pb-set-field ?r "ring_color" ?ring-color)
  (return ?r)
)

(deffunction smt-create-rings (?team-color)
  (bind ?rv (create$))
  ; (do-for-all-facts ((?r ring)) TRUE
  ;   (bind ?rv (append$ ?rv (smt-create-ring ?r:color ?r:req-bases)))
  ; )
  ; TODO Replace fix call by real data
  (bind ?rv (append$ ?rv (smt-create-ring "RING_GREEN" 2)))
  (bind ?rv (append$ ?rv (smt-create-ring "RING_BLUE" 1)))
  (bind ?rv (append$ ?rv (smt-create-ring "RING_ORANGE" 0)))
  (bind ?rv (append$ ?rv (smt-create-ring "RING_YELLOW" 0)))
  (return ?rv)
)

; Call plugin clips-smt
(defrule production-call-clips-smt
  (goal (id COMPLEXITY) (mode SELECTED))
  (wm-fact (key refbox phase) (value PRODUCTION))
  (wm-fact (key refbox team-color) (value ?team-color&CYAN|MAGENTA))
  ; (state IDLE)
  ; (not (plan-requested))
  ; (test (eq ?*ROBOT-NAME* "R-1"))
=>
	(bind ?p
	  (smt-create-data
		  (smt-create-robots ?team-color)
		  (smt-create-additional-robots ?team-color)
	    (smt-create-machines ?team-color)
	    (smt-create-orders ?team-color)
      (smt-create-rings ?team-color)
	  )
	)

	(smt-request "test" ?p)
	; (assert (plan-requested))
)

; Extract plan from protobuf
(defrule production-smt-plan-completed
	(smt-plan-complete ?handle)
	  ?g <- (goal (id COMPLEXITY) (mode SELECTED))
  (wm-fact (key refbox team-color) (value ?team-color&CYAN|MAGENTA))
	=>
	(printout t "SMT plan handle completed " ?handle  crlf)
	(assert
	 (plan (id COMPLEXITY-PLAN) (goal-id COMPLEXITY))
	)
	(bind ?plans (smt-get-plan ?handle))

	(printout t "Plan: " (pb-tostring ?plans) crlf)

	(bind ?from START)
	(bind ?from-side INPUT)
	(assert
		 (plan-action (id 99) (plan-id COMPLEXITY-PLAN) (duration 4.0)
									(action-name enter-field)
									(param-names r team-color) (param-values R-1 ?team-color))
	)
	(progn$ (?ap (pb-field-list ?plans "plans"))
		; ?ap is of type ActorSpecificPlan
		(bind ?actor-name (pb-field-value ?ap "actor_name"))
		(printout t "Working on ActorSpecificPlan of " ?actor-name crlf)
		(if (pb-has-field ?ap "sequential_plan")
		 then
			(bind ?p (pb-field-value ?ap "sequential_plan"))

			; (bind ?task-id (random-id))
			(bind ?actions (pb-field-list ?p "actions"))
			(bind ?steps (create$)) // TODO replace by plan-action
      (bind ?goal-base-color ""); Tempratly get the goal-info from the plan (for now a static C0 production is assumed)

			(loop-for-count (?ai (length$ ?actions))
				(bind ?a (nth$ ?ai ?actions))
				(bind ?actname (pb-field-value ?a "name"))
        (switch ?actname

		  ;ACTION:::::ENTER-FIELD:::::
          (case "enter-field" then (printout warn "Ignoring enter-field, done implicitly" crlf))

		  ;ACTION::::MOVE::::::
          (case "move" then
            (bind ?to "")
            (bind ?side "")
            (bind ?action-specific-actor "")
            (bind ?action-id (pb-field-value ?a "id"))
            (if (pb-has-field ?a "actor") 
              then
              (bind ?action-specific-actor (pb-field-value ?a "actor"))
            )
            (bind ?parents-ids (create$)) 
            (progn$ (?arg (pb-field-list ?a "parent_id"))
              (bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
            )
            (progn$ (?arg (pb-field-list ?a "params"))
              (if (eq (pb-field-value ?arg "key") "to") then
                (bind ?to (pb-field-value ?arg "value"))
				(bind ?to-splitted (str-split ?to "-"))
				(bind ?to (str-join "-" (subseq$ ?to-splitted 1 2)))
				(bind ?to-side (if (eq (nth$ 3 ?to-splitted) "I") then INPUT else OUTPUT))
               else
                (printout warn "Unknown parameter " (pb-field-value ?arg "key") " for " ?actname crlf)
              )
            )

            ; (bind ?next-step-id (+ ?task-id (+ (length$ ?steps) 1)))
            (bind ?next-step-id (* ?action-id 100))
            (bind ?steps (append$ ?steps ?next-step-id))
            ; (assert (step (name drive-to) (id ?next-step-id) (parents-ids ?parents-ids) (machine ?to) (side ?side) (actor ?action-specific-actor)))
			(assert
				 (plan-action (id ?next-step-id) (plan-id COMPLEXITY-PLAN) (duration 4.0)
											(action-name move)
											(param-names r from from-side to to-side) (param-values (string-to-field ?action-specific-actor) ?from ?from-side (string-to-field ?to) ?to-side))
			)
            (printout t "Action added: " ?action-specific-actor " [" ?action-id  "] move from: " ?from " at: " ?from-side " to: " ?to " at: " ?to-side crlf)
			(bind ?from (string-to-field ?to))
			(bind ?from-side ?to-side)
          )

		  ;ACTION:::::WP-GET-SHELF:::::
		  (case "wp-get-shelf" then
			(bind ?mps "")
			(bind ?side "")
			(bind ?shelf FALSE)
			(bind ?action-specific-actor "")
			(bind ?action-id (pb-field-value ?a "id"))
			(if (pb-has-field ?a "actor") 
			  then
			  (bind ?action-specific-actor (pb-field-value ?a "actor"))
			)
			(bind ?parents-ids (create$)) 
			(progn$ (?arg (pb-field-list ?a "parent_id"))
			  (bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
			)
			(progn$ (?arg (pb-field-list ?a "params"))
			  (if (eq (pb-field-value ?arg "key") "mps") then
				(bind ?mps (pb-field-value ?arg "value"))
				(bind ?mps-splitted (str-split ?mps "-"))
				(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
				(bind ?side (if (eq (nth$ 3 ?mps-splitted) "I") then INPUT else OUTPUT))
			   else
				(if (eq (pb-field-value ?arg "key") "shelf") then
				  (bind ?shelf (pb-field-value ?arg "value"))
				 else
				  (printout warn "Unknown parameter " (pb-field-value ?arg "key") " for " ?actname crlf)
				)
			  )
			)
			; Make sure that mps is of type CS
			; Match the corresponding cap carrier wp
			(do-for-fact ((?wm-fact wm-fact))
				(and
					(wm-key-prefix ?wm-fact:key (create$ domain fact mps-type))
					(eq (string-to-field ?mps) (wm-key-arg ?wm-fact:key m))
					(eq CS (wm-key-arg ?wm-fact:key t))
				)
				(do-for-fact ((?wm-fact2 wm-fact))
					(and
						(wm-key-prefix ?wm-fact2:key (create$ domain fact wp-on-shelf))
						(eq (wm-key-arg ?wm-fact:key m) (wm-key-arg ?wm-fact2:key m))
						(eq (string-to-field ?shelf) (wm-key-arg ?wm-fact2:key spot))
					)
					(bind ?next-step-id (* ?action-id 100))
					(bind ?wp (wm-key-arg ?wm-fact2:key wp))
					(assert
						 (plan-action (id ?next-step-id) (plan-id COMPLEXITY-PLAN) (duration 4.0)
													(action-name wp-get-shelf)
													(param-names r cc m spot) (param-values (string-to-field ?action-specific-actor) (wm-key-arg ?wm-fact2:key wp) (string-to-field ?mps) (string-to-field ?shelf)))
					)
					(printout t "Action added: " ?action-specific-actor " [" ?action-id  "] wp-get-shelf: " ?mps " at: " ?side " shelf: " ?shelf crlf)
				)
			)
		)

			;ACTION:::::WP-GET:::::
		  (case "wp-get" then
			(bind ?mps "")
			(bind ?side "")
			(bind ?action-specific-actor "")
			(bind ?action-id (pb-field-value ?a "id"))
			(if (pb-has-field ?a "actor") 
			  then
			  (bind ?action-specific-actor (pb-field-value ?a "actor"))
			)
			(bind ?parents-ids (create$)) 
			(progn$ (?arg (pb-field-list ?a "parent_id"))
			  (bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
			)
			(progn$ (?arg (pb-field-list ?a "params"))
			  (if (eq (pb-field-value ?arg "key") "mps") then
				(bind ?mps (pb-field-value ?arg "value"))
				(bind ?mps-splitted (str-split ?mps "-"))
				(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
				(bind ?side (if (eq (nth$ 3 ?mps-splitted) "I") then INPUT else OUTPUT))
			  else
				(printout warn "Unknown parameter " (pb-field-value ?arg "key") " for " ?actname crlf)
			  )
			)
			  ; (bind ?next-step-id (+ ?task-id (+ (length$ ?steps) 1)))
			  (bind ?next-step-id (* ?action-id 100))
			  (bind ?steps (append$ ?steps ?next-step-id))
			  ; (assert (step (name get-output) (id ?next-step-id) (parents-ids ?parents-ids) (machine ?mps) (side ?side) (machine-feature CONVEYOR) (actor ?action-specific-actor) ))
			(assert
				 (plan-action (id ?next-step-id) (plan-id COMPLEXITY-PLAN) (duration 4.0)
											(action-name wp-get)
											(param-names r wp m side) (param-values (string-to-field ?action-specific-actor) ?wp (string-to-field ?mps) ?side))
			)
			  (printout t "Action added: " ?action-specific-actor " [" ?action-id  "] wp-get from: " ?mps " side: " ?side crlf)

		  )

		  ;ACTION:::::WP-PUT:::::
		  (case "wp-put" then
			(bind ?mps "")
			(bind ?side "")
			(bind ?action-specific-actor "")
			(bind ?action-id (pb-field-value ?a "id"))
			(bind ?machine-feature CONVEYOR)
			(if (pb-has-field ?a "actor") 
			  then
			  (bind ?action-specific-actor (pb-field-value ?a "actor"))
			)
			(bind ?parents-ids (create$)) 
			(progn$ (?arg (pb-field-list ?a "parent_id"))
			  (bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
			)
			(progn$ (?arg (pb-field-list ?a "params"))
			  (if (eq (pb-field-value ?arg "key") "mps") then
				(bind ?mps (pb-field-value ?arg "value"))
				(bind ?mps-splitted (str-split ?mps "-"))
				(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
				(bind ?side (if (eq (nth$ 3 ?mps-splitted) "I") then INPUT else OUTPUT))
			  else
				(if (eq (pb-field-value ?arg "key") "slide") then
				  (bind ?slide (pb-field-value ?arg "value"))
				  (bind ?machine-feature (if (eq ?slide "true") then SLIDE else CONVEYOR))
				else
				  (printout warn "Unknown parameter " (pb-field-value ?arg "key") " for " ?actname crlf)
				)
			  )
			)
			; (bind ?next-step-id (+ ?task-id (+ (length$ ?steps) 1)))
			(bind ?next-step-id (* ?action-id 100))
			(bind ?steps (append$ ?steps ?next-step-id))
			; (assert (step (name insert) (id ?next-step-id) (parents-ids ?parents-ids) (machine ?mps) (side ?side) (machine-feature ?machine-feature) (already-at-mps FALSE) (actor ?action-specific-actor) )) ;MAGNOTE_ atmps should be true only when we had just picked from the shelf. Find that case
			(assert
				 (plan-action (id ?next-step-id) (plan-id COMPLEXITY-PLAN) (duration 4.0)
											(action-name wp-put)
											(param-names r wp m) (param-values (string-to-field ?action-specific-actor) ?wp (string-to-field ?mps)))
			)
			(printout t "Action added: " ?action-specific-actor " [" ?action-id  "] wp-put to: " ?mps " at: " ?side crlf)
		  )

		  ;ACTION:::::WP-DISCARD:::::
		  (case "wp-discard" then
			(bind ?action-specific-actor "")
			(bind ?action-id (pb-field-value ?a "id"))
			(if (pb-has-field ?a "actor") 
			  then
			  (bind ?action-specific-actor (pb-field-value ?a "actor"))
			)
			; (bind ?next-step-id (+ ?task-id (+ (length$ ?steps) 1)))
			(bind ?next-step-id (* ?action-id 100))
			(bind ?steps (append$ ?steps ?next-step-id))
			(bind ?cc ?wp)
			; (assert (step (name discard) (id ?next-step-id) (parents-ids ?parents-ids) (actor ?action-specific-actor) ))
			(assert
				 (plan-action (id ?next-step-id) (plan-id COMPLEXITY-PLAN) (duration 4.0)
											(action-name wp-discard)
											(param-names r cc) (param-values (string-to-field ?action-specific-actor) ?cc))
			)
			(printout t "Action added: " ?action-specific-actor " [" ?action-id  "] wp-discard" crlf)
		  )

		  ;ACTION:::::PREPARE-BS:::::
		  (case "prepare-bs" then
			(bind ?action-specific-actor "")
			(bind ?m "")
			(bind ?side "")
			(bind ?action-id (pb-field-value ?a "id"))
			(if (pb-has-field ?a "actor") 
			  then
			  (bind ?action-specific-actor (pb-field-value ?a "actor"))
			)
			(bind ?parents-ids (create$)) 
			(progn$ (?arg (pb-field-list ?a "parent_id"))
			  (bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
			)
			(progn$ (?arg (pb-field-list ?a "params"))
			  (if (eq (pb-field-value ?arg "key") "mps") then
				(bind ?mps (pb-field-value ?arg "value"))
				(bind ?mps-splitted (str-split ?mps "-"))
				(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
				(bind ?side (if (eq (nth$ 3 ?mps-splitted) "I") then INPUT else OUTPUT))
			  else
				  (if (eq (pb-field-value ?arg "key") "color") then
					; (bind ?goal-base-color (utils-remove-prefix (pb-field-value ?arg "value") base_)) ;temp: the color of the base of the goal is recognized here
					(bind ?goal-base-color (pb-field-value ?arg "value")) 

					else
						(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
					)
			  )
			)
			; (bind ?next-step-id (+ ?task-id (+ (length$ ?steps) 1)))
			(bind ?next-step-id (* ?action-id 100))
			(bind ?steps (append$ ?steps ?next-step-id))
			; (assert (step (name discard) (id ?next-step-id) (parents-ids ?parents-ids) (actor ?action-specific-actor) ))
			(assert
				 (plan-action (id ?next-step-id) (plan-id COMPLEXITY-PLAN) (duration 4.0)
											(action-name prepare-bs)
											(param-names m side bc) (param-values (string-to-field ?mps) ?side (string-to-field ?goal-base-color)))
			)
			(printout t "Action added: " ?action-specific-actor " [" ?action-id  "] prepare-bs with base-color: " ?goal-base-color crlf)
		  )

		  ;ACTION:::::BS-DISPENSE:::::
		  (case "bs-dispense" then
			(bind ?action-specific-actor "")
			(bind ?m "")
			(bind ?side "")
			(bind ?action-id (pb-field-value ?a "id"))
			(if (pb-has-field ?a "actor") 
			  then
			  (bind ?action-specific-actor (pb-field-value ?a "actor"))
			)
			(bind ?parents-ids (create$)) 
			(progn$ (?arg (pb-field-list ?a "parent_id"))
			  (bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
			)
			(progn$ (?arg (pb-field-list ?a "params"))
			  (if (eq (pb-field-value ?arg "key") "mps") then
				(bind ?mps (pb-field-value ?arg "value"))
				(bind ?mps-splitted (str-split ?mps "-"))
				(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
				(bind ?side (if (eq (nth$ 3 ?mps-splitted) "I") then INPUT else OUTPUT))
			  else
				  (if (eq (pb-field-value ?arg "key") "color") then
					; (bind ?goal-base-color (utils-remove-prefix (pb-field-value ?arg "value") base_)) ;temp: the color of the base of the goal is recognized here
					(bind ?goal-base-color (pb-field-value ?arg "value")) 

					else
						(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
					)
			  )
			)
			; (bind ?next-step-id (+ ?task-id (+ (length$ ?steps) 1)))
			(bind ?next-step-id (* ?action-id 100))
			(bind ?steps (append$ ?steps ?next-step-id))
			(bind ?wp WP1)
			; (assert (step (name discard) (id ?next-step-id) (parents-ids ?parents-ids) (actor ?action-specific-actor) ))
			(assert
				 (plan-action (id ?next-step-id) (plan-id COMPLEXITY-PLAN) (duration 4.0)
											(action-name bs-dispense)
											(param-names m side wp basecol) (param-values (string-to-field ?mps) ?side ?wp (string-to-field ?goal-base-color)))
			)
			(printout t "Action added: " ?action-specific-actor " [" ?action-id  "] bs-dispense with basecolor: " ?goal-base-color crlf)
		  )

		  ;ACTION:::::PREPARE-DS:::::
		  (case "prepare-ds" then
			(bind ?action-specific-actor "")
			(bind ?m "")
			(bind ?gate "")
			(bind ?action-id (pb-field-value ?a "id"))
			(if (pb-has-field ?a "actor") 
			  then
			  (bind ?action-specific-actor (pb-field-value ?a "actor"))
			)
			(bind ?parents-ids (create$)) 
			(progn$ (?arg (pb-field-list ?a "parent_id"))
			  (bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
			)
			(progn$ (?arg (pb-field-list ?a "params"))
			  (if (eq (pb-field-value ?arg "key") "mps") then
				(bind ?mps (pb-field-value ?arg "value"))
				(bind ?mps-splitted (str-split ?mps "-"))
				(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
			  else
			  (if (eq (pb-field-value ?arg "key") "gate") then
				 (bind ?gate (string-to-field (pb-field-value ?arg "value")))

				else
					(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
				)
			  )
			)
			; (bind ?next-step-id (+ ?task-id (+ (length$ ?steps) 1)))
			(bind ?next-step-id (* ?action-id 100))
			(bind ?steps (append$ ?steps ?next-step-id))
			; (assert (step (name discard) (id ?next-step-id) (parents-ids ?parents-ids) (actor ?action-specific-actor) ))
			(assert
				 (plan-action (id ?next-step-id) (plan-id COMPLEXITY-PLAN) (duration 4.0)
											(action-name prepare-ds)
											(param-names m gate) (param-values (string-to-field ?mps) ?gate))
			)
			(printout t "Action added: " ?action-specific-actor " [" ?action-id  "] prepare-ds" crlf)
		  )

		  ;ACTION:::::PREPARE-CS:::::
		  (case "prepare-cs" then
			(bind ?action-specific-actor "")
			(bind ?m "")
			(bind ?operation "")
			(bind ?action-id (pb-field-value ?a "id"))
			(if (pb-has-field ?a "actor") 
			  then
			  (bind ?action-specific-actor (pb-field-value ?a "actor"))
			)
			(bind ?parents-ids (create$)) 
			(progn$ (?arg (pb-field-list ?a "parent_id"))
			  (bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
			)
			(progn$ (?arg (pb-field-list ?a "params"))
			  (if (eq (pb-field-value ?arg "key") "mps") then
				(bind ?mps (pb-field-value ?arg "value"))
				(bind ?mps-splitted (str-split ?mps "-"))
				(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
			  else
				(if (eq (pb-field-value ?arg "key") "operation") then
				  (bind ?operation (pb-field-value ?arg "value"))

				else
					(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
				)
			  )
			)
			; (bind ?next-step-id (+ ?task-id (+ (length$ ?steps) 1)))
			(bind ?next-step-id (* ?action-id 100))
			(bind ?steps (append$ ?steps ?next-step-id))
			; (assert (step (name discard) (id ?next-step-id) (parents-ids ?parents-ids) (actor ?action-specific-actor) ))
			(assert
				 (plan-action (id ?next-step-id) (plan-id COMPLEXITY-PLAN) (duration 4.0)
											(action-name prepare-cs)
											(param-names m op) (param-values (string-to-field ?mps) (string-to-field ?operation)))
			)
			(printout t "Action added: " ?action-specific-actor " [" ?action-id  "] prepare-cs at: " ?mps " with " ?operation crlf)
		  )

		  ;ACTION:::::CS-RETRIEVE-CAP:::::
		  (case "cs-retrieve-cap" then
			(bind ?m "")
			(bind ?cap-color "")
			(bind ?action-id (pb-field-value ?a "id"))
			(bind ?parents-ids (create$)) 
			(progn$ (?arg (pb-field-list ?a "parent_id"))
			  (bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
			)
			(progn$ (?arg (pb-field-list ?a "params"))
			  (if (eq (pb-field-value ?arg "key") "mps") then
				(bind ?mps (pb-field-value ?arg "value"))
				(bind ?mps-splitted (str-split ?mps "-"))
				(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
			  else
				(if (eq (pb-field-value ?arg "key") "cap-color") then
				  (bind ?cap-color (pb-field-value ?arg "value"))

				else
					(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
				)
			  )
			)
			; (bind ?next-step-id (+ ?task-id (+ (length$ ?steps) 1)))
			(bind ?next-step-id (* ?action-id 100))
			(bind ?steps (append$ ?steps ?next-step-id))
			; (assert (step (name discard) (id ?next-step-id) (parents-ids ?parents-ids) (actor ?action-specific-actor) ))
			(assert
				 (plan-action (id ?next-step-id) (plan-id COMPLEXITY-PLAN) (duration 4.0)
											(action-name cs-retrieve-cap)
											(param-names m cc capcol) (param-values (string-to-field ?mps) ?wp (string-to-field ?cap-color)))
			)
			(printout t "Action added: " ?action-specific-actor " [" ?action-id  "] cs-retrieve-cap at: " ?mps crlf)
		  )

		  ;ACTION:::::PREPARE-RS::::::
		  (case "prepare-rs" then
			(bind ?action-specific-actor "")
			(bind ?m "")
			(bind ?action-id (pb-field-value ?a "id"))
			(if (pb-has-field ?a "actor") 
			  then
			  (bind ?action-specific-actor (pb-field-value ?a "actor"))
			)
			(bind ?parents-ids (create$)) 
			(progn$ (?arg (pb-field-list ?a "parent_id"))
			  (bind ?parents-ids (append$ ?parents-ids (* ?arg 100)))
			)
			(progn$ (?arg (pb-field-list ?a "params"))
			  (if (eq (pb-field-value ?arg "key") "mps") then
				(bind ?mps (pb-field-value ?arg "value"))
				(bind ?mps-splitted (str-split ?mps "-"))
				(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
			  else
				  (if (eq (pb-field-value ?arg "key") "ring_color") then
						(bind ?goal-ring-color (utils-remove-prefix (pb-field-value ?arg "value") ring_)) ;temp: the color of the base of the goal is recognized here

					else
						(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
					)
			  )
			)
			; (bind ?next-step-id (+ ?task-id (+ (length$ ?steps) 1)))
			(bind ?next-step-id (* ?action-id 100))
			(bind ?steps (append$ ?steps ?next-step-id))
			; (assert (step (name discard) (id ?next-step-id) (parents-ids ?parents-ids) (actor ?action-specific-actor) ))
			(assert
				 (plan-action (id ?next-step-id) (plan-id COMPLEXITY-PLAN) (duration 4.0)
											(action-name prepare-rs)
											(param-names m rc) (param-values (string-to-field ?mps) (string-to-field ?goal-ring-color)))
			)
			(printout t "Action added: " ?action-specific-actor " [" ?action-id  "] prepare-rs at: " ?mps crlf)
		  )

			;ACTION:::::DEFAULT:::::
			(default (printout warn "Unknown action " ?actname crlf))
        )
			)

			; (assert (task (id ?task-id) (state proposed) (steps ?steps) (robot ?actor-name)))
		 else
	  	(printout warn "Sequential plan not set on ActorSpecificPlan" crlf)
		)
	)
	(pb-destroy ?plans)
  (modify ?g (mode EXPANDED))
)
