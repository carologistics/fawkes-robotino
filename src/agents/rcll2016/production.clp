;---------------------------------------------------------------------------
;  production.clp - Planning for LLSF production
;                   Here we plan which job to execute when the robot is idle
;                   The job-execution is located in tasks.clp
;
;  Created: Sat Jun 16 12:35:16 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;                   Frederik Zwilling
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

; (defrule production-plan-test
;   (phase PRODUCTION)
;   (team-color ?team-color&CYAN|MAGENTA)
; =>
; 	(bind ?ap
; 	  (plan-create-actspec-sequential "R-1"
; 	    (plan-create-sequential (create$
; 	      (plan-create-action "move" "from" "START" "to" "C-BS-I")
; 	      (plan-create-action "move" "from" "C-BS-I" "to" "C-DS-I")
; 	    ))
; 	  )
; 	)
; 	(printout t "Plan: " (pb-tostring ?ap) crlf)

; 	(if (pb-has-field ?ap "sequential_plan")
; 	 then
; 		(bind ?p (pb-field-value ?ap "sequential_plan"))
; 		(printout t "Sequential plan is set:" crlf (pb-tostring ?p) crlf)
; 		(foreach ?a (pb-field-list ?p "actions")
; 			(bind ?actname (pb-field-value ?a "name"))
; 			(printout t "  Action '" ?actname "':" crlf)
; 			; Must use progn$ here, foreach cannot be nested in same scope
; 			(progn$ (?p (pb-field-list ?a "params"))
; 				(printout t "  - " (pb-field-value ?p "key") ": " (pb-field-value ?p "value") crlf)
; 			)
; 		)
; 	 else
; 	  (printout warn "Sequential plan not set on ActorSpecificPlan" crlf)
; 	)
; 	(pb-destroy ?ap)
; )

(deffunction smt-create-data (?robots ?addrobots ?machines ?orders)
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
	(printout t "Proto:" (pb-tostring ?p) crlf)
	(return ?p)
)

(deffunction smt-create-robot (?name ?team-color ?number ?pose-x ?pose-y)
	(bind ?r (pb-create "llsf_msgs.Robot"))
	(pb-set-field ?r "name" ?name)
	(pb-set-field ?r "team_color" ?team-color)
	(pb-set-field ?r "number" ?*ROBOT-NUMBER*)
	(bind ?pose (pb-create "llsf_msgs.Pose2D"))
	(pb-set-field ?pose "x" ?pose-x)
	(pb-set-field ?pose "y" ?pose-y)
	(pb-set-field ?pose "ori" 0.0)
	(pb-set-field ?r "pose" ?pose)
	(return ?r)
)

(deffunction smt-create-additional-robots (?team-color)
	(bind ?rv (create$))
	(do-for-all-facts ((?p pose)) TRUE
		(bind ?rv (append$ ?rv (smt-create-robot ?p:name ?team-color 0 ?p:x ?p:y)))
	)
	(return ?rv)
)

(deffunction smt-create-robots (?team-color)
	(bind ?rv (create$))
	(do-for-all-facts ((?r active-robot)) TRUE
		(bind ?rv (append$ ?rv (smt-create-robot ?r:name ?team-color 0 ?r:x ?r:y)))
	)
	(return ?rv)
)

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
	(return ?m)
)

(deffunction smt-create-machines (?team-color)
	(bind ?rv (create$))
	(do-for-all-facts ((?m machine)) (eq ?m:team ?team-color)
		(bind ?rv (append$ ?rv (smt-create-machine ?m:name ?m:mtype ?m:state ?m:team ?m:x ?m:y)))
	)
	(return ?rv)
)

(deffunction smt-create-order (?id ?product-id ?gate ?complexity ?q-req ?q-del ?begin ?end ?team-color)
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

  (do-for-fact ((?product product)) (eq ?product:id ?product-id)
    (bind ?rlist (create$))
    (progn$ (?r ?product:rings)
      (switch ?r
        (case GREEN then (bind ?rlist (append$ ?rlist "RING_GREEN")))
        (case BLUE then (bind ?rlist (append$ ?rlist "RING_BLUE")))
        (case ORANGE then (bind ?rlist (append$ ?rlist "RING_ORANGE")))
        (case YELLOW then (bind ?rlist (append$ ?rlist "RING_YELLOW")))
        (default (printout warn "Ring color not found" crlf))
      )
    )
    (foreach ?rings ?rlist
     (pb-add-list ?o "ring_colors" ?rings)
    )
    (switch ?product:cap
      (case BLACK then (pb-set-field ?o "cap_color" "CAP_BLACK"))
      (case GREY then (pb-set-field ?o "cap_color" "CAP_GREY"))
      (default (printout warn "Cap color not found" crlf))
    )
    (switch ?product:base
      (case BLACK then (pb-set-field ?o "base_color" "BASE_BLACK"))
      (case RED then (pb-set-field ?o "base_color" "BASE_RED"))
      (case SILVER then (pb-set-field ?o "base_color" "BASE_SILVER"))
      (default (printout warn "Base color not found" crlf))
    ) 
  )

  (return ?o)
)

(deffunction smt-create-orders (?team-color)
	(bind ?rv (create$))
	(do-for-all-facts ((?o order)) TRUE
		(bind ?rv (append$ ?rv (smt-create-order ?o:id ?o:product-id ?o:delivery-gate ?o:complexity ?o:quantity-requested ?o:quantity-delivered ?o:begin ?o:end ?team-color)))
	)
	(return ?rv)
)

(defrule production-call-clips-smt
  (phase PRODUCTION)
  (team-color ?team-color&CYAN|MAGENTA)
	(state IDLE)
	(not (plan-requested))
	(test (eq ?*ROBOT-NAME* "R-1"))
  (exists (machine))
  (exists (order))
=>
	(bind ?p
	  (smt-create-data
		  (smt-create-robots ?team-color)
		  (smt-create-additional-robots ?team-color)
	    (smt-create-machines ?team-color)
	    (smt-create-orders ?team-color)
	  )
	)

	(smt-request "test" ?p)
	(assert (plan-requested))
					;(smt-plan-complete "test"))
)

(defrule production-smt-plan-completed
	(smt-plan-complete ?handle)
	=>
	(printout t "SMT plan handle completed " ?handle  crlf)
	(bind ?plans (smt-get-plan ?handle))

	; Assert a simple example plan with multiple goals for each robot
	; (bind ?plans
	;   (plan-create-actgroup
	;   	(plan-create-actspec-sequential "R-1"
	;     	(plan-create-sequential (create$
	;       	(plan-create-action "move" "to" "C-BS-I")
	;       	(plan-create-action "move" "to" "C-DS-I")
	;     )))
	;   	(plan-create-actspec-sequential "R-2"
	;     	(plan-create-sequential (create$
	;       	(plan-create-action "move" "to" "C-CS1-I")
	;       	(plan-create-action "move" "to" "C-CS2-I")
	;     )))
	;   	(plan-create-actspec-sequential "R-3"
	;     	(plan-create-sequential (create$
	;       	(plan-create-action "move" "to" "C-RS1-I")
	;       	(plan-create-action "move" "to" "C-RS2-I")
	; 		)))
	; 	)
	; )

	(printout t "Plan: " (pb-tostring ?plans) crlf)

	(progn$ (?ap (pb-field-list ?plans "plans"))
		; ?ap is of type ActorSpecificPlan
		(bind ?actor-name (pb-field-value ?ap "actor_name"))
		(printout t "Working on ActorSpecificPlan of " ?actor-name crlf)
		(if (pb-has-field ?ap "sequential_plan")
		 then
			(bind ?p (pb-field-value ?ap "sequential_plan"))

			; (bind ?task-id (random-id))
			(bind ?actions (pb-field-list ?p "actions"))
			(bind ?steps (create$))
      (bind ?goal-base-color ""); Tempratly get the goal-info from the plan (for now a static C0 production is assumed)

			(loop-for-count (?ai (length$ ?actions))
				(bind ?a (nth$ ?ai ?actions))
				(bind ?actname (pb-field-value ?a "name"))
        (switch ?actname
          (case "enter-field" then (printout warn "Ignoring enter-field, done implicitly" crlf))
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
              (bind ?parents-ids (append$ ?parents-ids ?arg))
            )
            (progn$ (?arg (pb-field-list ?a "params"))
              (if (eq (pb-field-value ?arg "key") "to") then
                (bind ?to (pb-field-value ?arg "value"))
                (bind ?to-splitted (str-split ?to "-"))
                (bind ?to (str-join "-" (subseq$ ?to-splitted 1 2)))
                (bind ?side (if (eq (nth$ 3 ?to-splitted) "I") then INPUT else OUTPUT))
               else
                (printout warn "Unknown parameter " (pb-field-value ?arg "key") " for " ?actname crlf)
              )
            )

            ; (bind ?next-step-id (+ ?task-id (+ (length$ ?steps) 1)))
            (bind ?next-step-id (* ?action-id 100))
            (bind ?steps (append$ ?steps ?next-step-id))
            (assert (step (name drive-to) (id ?next-step-id) (parents-ids ?parents-ids) (machine ?to) (side ?side) (actor ?action-specific-actor)))
            (printout t "Action Added: " ?action-specific-actor " [" ?action-id  "] Driving to: " ?to " at: " ?side crlf)
          )
          ;ACTION:::::GET FROM SHELF::::::
          (case "retrieve_shelf" then
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
              (bind ?parents-ids (append$ ?parents-ids ?arg))
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
            (if (and (neq ?shelf "FALSE")
                       (any-factp ((?machine machine)) (and (eq ?machine:name (string-to-field ?mps))  (eq ?machine:mtype CS)))  )
            then
              ; (bind ?next-step-id (+ ?task-id (+ (length$ ?steps) 1)))
              (bind ?next-step-id (* ?action-id 100))
              (bind ?steps (append$ ?steps ?next-step-id))
              (assert (step (name get-from-shelf) (id ?next-step-id) (parents-ids ?parents-ids) (machine ?mps) (side ?side) (machine-feature SHELF) (actor ?action-specific-actor)))
              (printout t "Action Added: " ?action-specific-actor " [" ?action-id  "] Retrieving from Shelf: " ?mps " at: " ?side " shelf: " ?shelf crlf)
            else
              (printout t "Wrong Parameters passed to retrive_shelf Action (mps:" ?mps "side:" ?side "shelf:" ?shelf ")" crlf)
            )
          )
          ;ACTION:::::RETRIVE::::::
          (case "retrieve" then
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
              (bind ?parents-ids (append$ ?parents-ids ?arg))
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
            (if (any-factp ((?machine machine)) (and (eq ?machine:name (string-to-field ?mps))  (eq ?machine:mtype BS)))
              then
              ; (bind ?next-step-id (+ ?task-id (+ (length$ ?steps) 1)))
              (bind ?next-step-id (* ?action-id 100))
              (bind ?steps (append$ ?steps ?next-step-id))
              (assert (step (name get-base) (id ?next-step-id) (parents-ids ?parents-ids) (machine ?mps) (side ?side) (base ?goal-base-color) (actor ?action-specific-actor) ))
              (printout t "Action Added: " ?action-specific-actor " [" ?action-id  "] Retrieving Base from: " ?mps " at: " ?side " Base-Color: " ?goal-base-color  crlf)
              else
              ; (bind ?next-step-id (+ ?task-id (+ (length$ ?steps) 1)))
              (bind ?next-step-id (* ?action-id 100))
              (bind ?steps (append$ ?steps ?next-step-id))
              (assert (step (name get-output) (id ?next-step-id) (parents-ids ?parents-ids) (machine ?mps) (side ?side) (machine-feature CONVEYOR) (actor ?action-specific-actor) ))
              (printout t "Action Added: " ?action-specific-actor " [" ?action-id  "] Retrieving Output from: " ?mps " side: " ?side crlf)

            )
          )
          ;ACTION:::::FEED::::::
          (case "feed" then
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
              (bind ?parents-ids (append$ ?parents-ids ?arg))
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
            (assert (step (name insert) (id ?next-step-id) (parents-ids ?parents-ids) (machine ?mps) (side ?side) (machine-feature CONVEYOR) (already-at-mps FALSE) (actor ?action-specific-actor) )) ;MAGNOTE_ atmps should be true only when we had just picked from the shelf. Find that case
            (printout t "Action Added: " ?action-specific-actor " [" ?action-id  "] Brining Product to: " ?mps " at: " ?side crlf)
          )
          ;ACTION:::::Discard::::::
          (case "discard" then
            (bind ?action-specific-actor "")
            (bind ?action-id (pb-field-value ?a "id"))
            (if (pb-has-field ?a "actor") 
              then
              (bind ?action-specific-actor (pb-field-value ?a "actor"))
            )
            ; (bind ?next-step-id (+ ?task-id (+ (length$ ?steps) 1)))
            (bind ?next-step-id (* ?action-id 100))
            (bind ?steps (append$ ?steps ?next-step-id))
            (assert (step (name discard) (id ?next-step-id) (parents-ids ?parents-ids) (actor ?action-specific-actor) ))
            (printout t "Action Added: " ?action-specific-actor " [" ?action-id  "] discarding Product to" crlf)
          )
          ;ACTION:::::PREPARE::::::
          (case "prepare" then
            (bind ?mps "")
            (bind ?side "")
            (bind ?mps-type "")
            (bind ?operation "")
            (bind ?gate "")`
            (bind ?action-specific-actor "")
            (bind ?action-id (pb-field-value ?a "id"))
            (if (pb-has-field ?a "actor") 
              then
              (bind ?action-specific-actor (pb-field-value ?a "actor"))
            )
            (bind ?parents-ids (create$)) 
            (progn$ (?arg (pb-field-list ?a "parent_id"))
              (bind ?parents-ids (append$ ?parents-ids ?arg))
            )
            (progn$ (?arg (pb-field-list ?a "params"))
              (if (eq (pb-field-value ?arg "key") "mps") then
                (bind ?mps (pb-field-value ?arg "value"))
                (bind ?mps-splitted (str-split ?mps "-"))
                (bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
                (bind ?side (if (eq (nth$ 3 ?mps-splitted) "I") then INPUT else OUTPUT))
                (do-for-fact ((?machine machine)) (eq ?machine:name ?mps)
                    (bind ?mps-type ?machine:mtype)
                )
              else
                (if (eq (pb-field-value ?arg "key") "operation") then
                  (bind ?operation (pb-field-value ?arg "value"))
        				  (if (eq ?operation "RETRIEVE_CAP") then
                    		; (bind ?next-step-id (+ ?task-id (+ (length$ ?steps) 1)))
						    (bind ?next-step-id (* ?action-id 100))
						    (bind ?steps (append$ ?steps ?next-step-id))
						    (assert (step (name instruct-mps) (id ?next-step-id) (parents-ids ?parents-ids) (machine ?mps) (side ?side) (cs-operation RETRIEVE_CAP) (actor ?action-specific-actor) ))
        				  else
                    		; (bind ?next-step-id (+ ?task-id (+ (length$ ?steps) 1)))
                        (bind ?next-step-id (* ?action-id 100))
						            (bind ?steps (append$ ?steps ?next-step-id))
                    		(assert (step (name instruct-mps) (id ?next-step-id) (parents-ids ?parents-ids) (machine ?mps) (side ?side) (cs-operation MOUNT_CAP) (actor ?action-specific-actor) ))
        				  )
                else
                  (if (eq (pb-field-value ?arg "key") "color") then
                    (bind ?goal-base-color (utils-remove-prefix (pb-field-value ?arg "value") BASE_)) ;TEMP: The color of the base of the goal is recognized here
                    ; (bind ?next-step-id (+ ?task-id (+ (length$ ?steps) 1)))
                    (bind ?next-step-id (- (* ?action-id 100) 1)) ;Injected step (Needs to be executed but does not come in the plan)
                    (bind ?steps (append$ ?steps ?next-step-id))
                    (assert (step (name acquire-lock) (id ?next-step-id) (parents-ids ?parents-ids) (task-priority ?*PRIORITY-PREFILL-RS*) (lock PREPARE-BS) (actor ?action-specific-actor) )) ;is released after get-base
                    ; (bind ?next-step-id (+ ?task-id (+ (length$ ?steps) 1)))
                    (bind ?next-step-id (* ?action-id 100))
                    (bind ?steps (append$ ?steps ?next-step-id))
                    (assert (step (name instruct-mps) (id ?next-step-id) (parents-ids ?parents-ids) (machine ?mps) (side ?side) (base ?goal-base-color) (actor ?action-specific-actor) ))

                  else
                    (if (eq (pb-field-value ?arg "key") "gate") then
                   	  (bind ?gate (string-to-field (pb-field-value ?arg "value")))
                      ; (bind ?next-step-id (+ ?task-id (+ (length$ ?steps) 1)))
                      (bind ?next-step-id (* ?action-id 100))
                   	  (bind ?steps (append$ ?steps ?next-step-id))
		                  (assert (step (name instruct-mps) (id ?next-step-id) (parents-ids ?parents-ids) (machine ?mps) (side ?side) (gate ?gate) (actor ?action-specific-actor) ))
                    else
                      (printout warn "Unknown parameter " (pb-field-value ?arg "key") " for " ?actname crlf)
                    )
                  )
                )
              )
            )
            (printout t "Action Added: " ?action-specific-actor " [" ?action-id  "] Instructing to: " ?mps " at: " ?side crlf)
          )

					(default (printout warn "Unknown action " ?actname crlf))
        )
			)

			; (assert (task (id ?task-id) (state proposed) (steps ?steps) (robot ?actor-name)))
		 else
	  	(printout warn "Sequential plan not set on ActorSpecificPlan" crlf)
		)
	)
	(pb-destroy ?plans)
)


(defrule switch-to-the-next-task
  "Make a task containing the first step in a sequence of steps with no task"
  (phase PRODUCTION)
  (step (id ?id) (state inactive) (actor ?actor-name))
  (forall (step (id ?id_others)) (test (>= ?id_others ?id)))
  (or 	(not (task))
  		(forall (task (state ?task-state)) (test (eq ?task-state finished)))
  	)
  (lock-role MASTER)
  =>
  (bind ?task-id (random-id))
  (assert (task (id ?task-id) (state proposed) (steps ?id) (robot ?actor-name)))
  )


; (defrule switch-to-the-next-task
;   "Make a task containing the first step in a sequence of steps with no task"
;   (phase PRODUCTION)
;   (step (id ?id) (state inactive) (actor ?actor-name))
;   (forall (step (id ?id_others)) (test (>= ?id_others ?id)))
;   (not (task))
;   (lock-role MASTER)
;   =>
;   (bind ?task-id (random-id))
;   (assert (task (id ?task-id) (state proposed) (steps ?id) (robot ?actor-name)))
;   )


(defrule prod-propose-task-idle
  "If we are idle change state to the proposed task."
  (declare (salience ?*PRIORITY-HIGH*))
  (phase PRODUCTION)
  ?sf <- (state IDLE)
  (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (state proposed))
  =>
  (retract ?sf)
  (assert (state TASK-PROPOSED))
  )



(defrule prod-change-to-more-important-task-when-waiting-for-lock
  "If we run a low-priority task and look for an alternative AND a task with a higher priority is proposed, drop the current work and change to the priorized task."
  (declare (salience ?*PRIORITY-LOW*))
  (phase PRODUCTION)
  ?t <- (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (state running) (priority ?old-p))
  ?pt <- (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (state proposed) (priority ?p&:(> ?p ?old-p)))
  ?sf1 <- (state WAIT_AND_LOOK_FOR_ALTERATIVE)
  ?sf2 <- (state WAIT-FOR-LOCK)
  ?lock-get <- (lock (type GET) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?res))
  ?lock-ref <- (lock (type REFUSE) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?res))
  ?wfl <- (wait-for-lock (res ?res) (state get))
  ?exec <- (execute-skill ? ?)
  =>
  (retract ?sf1 ?sf2 ?wfl ?lock-get ?exec ?lock-ref)
  (modify ?t (state finished))
  (assert
    (state TASK-PROPOSED)
	  (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?res))
  )
)

; (defrule prod-remove-proposed-tasks
;   "Remove all proposed tasks, when you are neither idle nor looking for an alternative."
;   (declare (salience ?*PRIORITY-LOW*))
;   (phase PRODUCTION)
;   (not (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE))
;   ?pt <- (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (state proposed))
;   =>
;   (retract ?pt)
; )

; (defrule prod-prefill-cap-station
;   "Feed a CS with a cap from its shelf so that afterwards it can directly put the cap on a product."
;   (declare (salience ?*PRIORITY-PREFILL-CS*))
;   (phase PRODUCTION)
;   (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;   (team-color ?team-color&~nil)
;   (holding NONE)
;   (machine (mtype CS) (loaded-id 0) (incoming $?i&~:(member$ FILL_CAP ?i))
;            (name ?machine) (produced-id 0) (team ?team-color)
;            (state ~DOWN&~BROKEN))
;   (cap-station (name ?machine) (cap-loaded NONE)
; 	  (assigned-cap-color ?cap-color&~NONE))
;   ;check that the task was not rejected before
;   (not (and (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (name fill-cap) (state rejected) (id ?rej-id))
;             (step (name insert) (id ?rej-st&:(eq ?rej-st (+ ?rej-id 2))) (machine ?machine))))
;   (not (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-PREFILL-CS*))))
;   (found-tag (name ?machine))
;   =>
;   (printout t "PROD: FILL " ?machine " with " ?cap-color " cap from shelf" crlf)
;   (bind ?task-id (random-id))
;   (assert (task (name fill-cap) (id ?task-id) (state proposed)
; 		        (steps (create$ (+ ?task-id 1) (+ ?task-id 2)))
; 		        (priority ?*PRIORITY-PREFILL-CS*))
; 	  (step (name get-from-shelf) (id (+ ?task-id 1))
; 		  (task-priority ?*PRIORITY-PREFILL-CS*)
; 		  (machine ?machine)
;       (machine-feature SHELF))
; 	  (step (name insert) (id (+ ?task-id 2))
; 		  (task-priority ?*PRIORITY-PREFILL-CS*)
; 		  (machine ?machine)
;       (machine-feature CONVEYOR)
;       (already-at-mps TRUE))
;     (needed-task-lock (task-id ?task-id) (action FILL_CAP) (place ?machine))
;   )
; )

; (defrule prod-remove-empty-base-from-cs
;   "Remove an unknown base from CS after retrieving a cap from it."
;   (declare (salience ?*PRIORITY-CLEAR-CS*))
;   (phase PRODUCTION)
;   (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;   (team-color ?team-color&~nil)
;   (holding NONE)
;   (machine (mtype CS) (loaded-id 0) (incoming $?i&~:(member$ GET-PROD ?i))
;            (name ?machine) (produced-id ?produced-id&~0) (team ?team-color)
;            (state READY-AT-OUTPUT))
;   (product
;     (id ?produced-id)
;     (base UNKNOWN) ;only remove empty bases -> unknown color
;   )
;   ;check that the task was not rejected before
;   (not (and (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (name clear-cs) (state rejected) (id ?rej-id))
;             (step (name get-output) (id ?rej-st&:(eq ?rej-st (+ ?rej-id 1))) (machine ?machine))))
;   (not (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-CLEAR-CS*))))
;   (found-tag (name ?machine))
;   =>
;   (printout t "PROD: CLEAR " ?machine crlf)
;   (bind ?task-id (random-id))
;   (assert (task (name clear-cs) (id ?task-id) (state proposed)
; 		        (steps (create$ (+ ?task-id 1)))
; 		        (priority ?*PRIORITY-CLEAR-CS*))
; 	  (step (name get-output) (id (+ ?task-id 1))
; 		  (task-priority ?*PRIORITY-CLEAR-CS*)
; 		  (machine ?machine))
;     (needed-task-lock (task-id ?task-id) (action GET-PROD) (place ?machine))
;   )
; )

; (defrule prod-prefill-ring-station
;   "Feed a RS with a base from the BS so that it is ready for the next production."
;   (declare (salience ?*PRIORITY-PREFILL-RS*))
;   (phase PRODUCTION)
;   (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;   (team-color ?team-color&~nil)
;   (holding NONE)
;   (machine (mtype RS) (incoming $?i&~:(member$ PREFILL-RS ?i))
;     (name ?rs) (team ?team-color)
;     (state ~DOWN&~BROKEN))
;   (ring-station (name ?rs) (bases-loaded ?bases&:(< ?bases 3)))
;   (found-tag (name ?rs))
;   (machine (mtype BS)
;     (name ?bs) (team ?team-color)
;     (state ~DOWN&~BROKEN))
;   (base-station (name ?bs) (active-side ?bs-side) (fail-side ?fs&:(neq ?bs-side ?fs)))
;   (found-tag (name ?bs))
;   ;check that the task was not rejected before
;   (not (and (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (name fill-rs) (state rejected) (id ?rej-id))
;             (step (name insert) (id ?rej-st&:(eq ?rej-st (+ ?rej-id 5))) (machine ?rs))))
;   (not (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-PREFILL-RS*))))
;   =>
;   (printout t "PROD: FILL " ?rs " with base from BS" crlf)
;   (bind ?task-id (random-id))
;   (assert (task (name fill-rs) (id ?task-id) (state proposed)
; 		        (steps (create$ (+ ?task-id 1) (+ ?task-id 2) (+ ?task-id 3) (+ ?task-id 4) (+ ?task-id 5)))
; 		        (priority ?*PRIORITY-PREFILL-RS*))
;     (step (name drive-to) (id (+ ?task-id 1))
;       (task-priority ?*PRIORITY-PREFILL-RS*)
;       (machine ?bs) (side ?bs-side))
;     (step (name acquire-lock) (id (+ ?task-id 2))
;       (task-priority ?*PRIORITY-PREFILL-RS*) (lock PREPARE-BS)) ;is released after get-base
;     (step (name instruct-mps) (id (+ ?task-id 3))
;       (task-priority ?*PRIORITY-PREFILL-RS*)
;       (machine ?bs) (base RED) (side ?bs-side))
;     (step (name get-base) (id (+ ?task-id 4))
;       (task-priority ?*PRIORITY-PREFILL-RS*)
;       (machine ?bs) (machine-feature CONVEYOR)
;       (base RED) (side ?bs-side))
;     (step (name insert) (id (+ ?task-id 5))
;       (task-priority ?*PRIORITY-PREFILL-RS*)
;       (machine ?rs) (machine-feature SLIDE))
;     (needed-task-lock (task-id ?task-id) (action PREFILL-RS) (place ?rs))
;   )
; )

; (defrule insert-unknown-base-to-rs
;   "Insert a base with unknown color in a RS for preparation"
;   (declare (salience ?*PRIORITY-PREFILL-RS*))
;   (phase PRODUCTION)
;   (state IDLE)
;   (team-color ?team-color&~nil)
;   (holding ?product-id&~NONE)
;   (product (id ?product-id) (base UNKNOWN))
;   (machine (mtype RS) (incoming $?i&~:(member$ PREFILL-RS ?i))
;            (name ?rs) (team ?team-color)
;            (state ~DOWN&~BROKEN))
;   (ring-station (name ?rs) (bases-loaded ?bases&:(< ?bases 3)))
;   ;check that the task was not rejected before
;   (not (and
;     (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (name fill-rs) (state rejected) (id ?rej-id))
;     (step (name insert) (id ?rej-st&:(eq ?rej-st (+ ?rej-id 1))) (machine ?rs) (machine-feature SLIDE))
;   ))
;   (not (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-PREFILL-RS*))))
;   (found-tag (name ?rs))
;   =>
;   (printout t "PROD: INSERT unknown base " ?product-id " into " ?rs crlf)
;   (bind ?task-id (random-id))
;   (assert
;     (task (name fill-rs) (id ?task-id) (state proposed)
;       (steps (create$ (+ ?task-id 1)))
;       (priority ?*PRIORITY-PREFILL-RS*))
;     (step (name insert) (id (+ ?task-id 1))
;       (task-priority ?*PRIORITY-PREFILL-RS*)
;       (machine ?rs)
;       (machine-feature SLIDE))
;     (needed-task-lock (task-id ?task-id) (action PREFILL-RS) (place ?rs))
;   )
; )

; (defrule insert-unintentionally-holding-base-to-rs
;   "Insert a base we hold unintentionally (e.g. skill-fail) in a RS for preparation"
;   (declare (salience ?*PRIORITY-PREFILL-RS-WITH-HOLDING-BASE*))
;   (phase PRODUCTION)
;   (state IDLE)
;   (team-color ?team-color&~nil)
;   (holding ?product-id&~NONE)
;   (product (id ?product-id))
;   (machine (mtype RS) (incoming $?i&~:(member$ PREFILL-RS ?i))
;            (name ?rs) (team ?team-color)
;            (state ~DOWN&~BROKEN))
;   (ring-station (name ?rs) (bases-loaded ?bases&:(< ?bases 3)))
;   ;check that the task was not rejected before
;   (not (and
;     (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (name fill-rs) (state rejected) (id ?rej-id))
;     (step (name insert) (id ?rej-st&:(eq ?rej-st (+ ?rej-id 1))) (machine ?rs) (machine-feature SLIDE))
;   ))
;   (not (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-PREFILL-RS-WITH-HOLDING-BASE*))))
;   (found-tag (name ?rs))
;   =>
;   (printout t "PROD: INSERT unintentionally holding base " ?product-id " into " ?rs crlf)
;   (bind ?task-id (random-id))
;   (assert
;     (task (name fill-rs) (id ?task-id) (state proposed)
;       (steps (create$ (+ ?task-id 1)))
;       (priority ?*PRIORITY-PREFILL-RS-WITH-HOLDING-BASE*))
;     (step (name insert) (id (+ ?task-id 1))
;       (task-priority ?*PRIORITY-PREFILL-RS-WITH-HOLDING-BASE*)
;       (machine ?rs)
;       (machine-feature SLIDE))
;     (needed-task-lock (task-id ?task-id) (action PREFILL-RS) (place ?rs))
;   )
; )

; (defrule discard-unneeded-base
;   "Discard a base which is not needed if no RS can be pre-filled"
;   (declare (salience ?*PRIORITY-DISCARD-UNKNOWN*))
;   (phase PRODUCTION)
;   (state IDLE)
;   (team-color ?team-color&~nil)
;   (holding ?product-id&~NONE)
;   (product (id ?product-id))
;   (machine (mtype RS) (name ?rs))
;   ;only discard if ring stations have at least two bases loaded
;   (ring-station (name ?rs) (bases-loaded ?bl&~:(< ?bl 3)))
;   (found-tag (name ?rs))
;   (not (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-DISCARD-UNKNOWN*))))
;   =>
;   (printout t "PROD: Discard unneeded or unknown base " ?product-id crlf)
;   (bind ?task-id (random-id))
;   (assert
;     (task (name discard-unknown) (id ?task-id) (state proposed)
;       (steps (create$ (+ ?task-id 1)))
;       (priority ?*PRIORITY-DISCARD-UNKNOWN*))
;     (step (name discard) (id (+ ?task-id 1))
;       (task-priority ?*PRIORITY-DISCARD-UNKNOWN*))
;   )
; )

; (defrule prod-clear-bs
;   "Clear BS if base retrieval failed"
;   (declare (salience ?*PRIORITY-CLEAR-BS*))
;   (phase PRODUCTION)
;   (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;   (team-color ?team-color&~nil)
;   (holding NONE)
;   (machine (mtype BS) (name ?bs) (team ?team-color) (state READY-AT-OUTPUT))
;   (base-station (name ?bs) (fail-side ?side))
;   (not (locked-resource (resource ?res&:(eq ?res (sym-cat ?bs "-" ?side)))))
;   (not (and (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (name clear-bs) (state rejected) (id ?rej-id))
;             (step (name get-output) (id ?rej-st&:(eq ?rej-st (+ ?rej-id 1))) (machine ?bs))))
;   (not (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-CLEAR-BS*))))
;   =>
;   (printout t "PROD: Clearing BS due to failed base retrieval" crlf)
;   (bind ?task-id (random-id))
;   (assert
;     (task (name clear-bs) (id ?task-id) (state proposed)
;       (steps (create$ (+ ?task-id 1)))
;       (priority ?*PRIORITY-CLEAR-BS*))
;     (step (name get-output) (id (+ ?task-id 1))
;       (task-priority ?*PRIORITY-CLEAR-BS*)
;       (machine ?bs) (machine-feature CONVEYOR) (side ?side))
;     (needed-task-lock (task-id ?task-id) (action CLEAR-BS) (place ?bs))
;   )
; )

; (defrule prod-produce-c0
;   "Produce a C0"
;   (declare (salience ?*PRIORITY-PRODUCE-C0*))
;   (phase PRODUCTION)
;   (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;   (team-color ?team-color&~nil)
;   (holding NONE)
;   (game-time $?game-time)
;   (machine (mtype CS) (incoming $?i&~:(member$ PROD_CAP ?i))
;            (name ?cs) (team ?team-color) (produced-id 0)
;            (state ~DOWN&~BROKEN))
;   (cap-station (name ?cs) (cap-loaded ?cap-color) (assigned-cap-color ?cap-color))
;   (found-tag (name ?cs))
;   (machine (mtype BS)
;     (name ?bs) (team ?team-color)
;     (state ~DOWN&~BROKEN))
;   (base-station (name ?bs) (active-side ?bs-side) (fail-side ?fs&:(neq ?bs-side ?fs)))
;   (found-tag (name ?bs))
;   ;check that the task was not rejected before
;   (not (and (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (name produce-c0) (state rejected) (id ?rej-id))
;             (step (name insert) (id ?rej-st&:(eq ?rej-st (+ ?rej-id 6))) (machine ?cs))))
;   (not (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-PRODUCE-C0*))))
;   ;check for open C0 order
;   (product (id ?product-id) (rings $?r&:(eq 0 (length$ ?r))) (cap ?cap-color) (base ?base-color))
;   ?of <- (order (product-id ?product-id)
;     (quantity-requested ?qr) (quantity-delivered ?qd&:(> ?qr ?qd))
;     (begin ?begin&:(< ?begin (+ (nth$ 1 ?game-time) ?*PRODUCE-C0-AHEAD-TIME*)))
;     (end ?end&:(> ?end (+ (nth$ 1 ?game-time) ?*PRODUCE-C0-LATEST-TIME*)))
;     (in-production 0) (in-delivery ?id&:(> ?qr (+ ?qd ?id)))
;   )
;   =>
;   (printout warn "TODO: production durations not yet implemented" crlf)
;   (printout t "PROD: PRODUCE C0 with " ?cap-color " cap at " ?cs crlf)
;   (bind ?task-id (random-id))
;   (assert (task (name produce-c0) (id ?task-id) (state proposed)
;     (steps (create$ (+ ?task-id 1) (+ ?task-id 2) (+ ?task-id 3) (+ ?task-id 4) (+ ?task-id 5) (+ ?task-id 6)))
;     (priority ?*PRIORITY-PRODUCE-C0*))
;     (step (name drive-to) (id (+ ?task-id 1))
;       (task-priority ?*PRIORITY-PRODUCE-C0*)
;       (machine ?bs)
;       (side ?bs-side))
;     (step (name acquire-lock) (id (+ ?task-id 2))
;       (task-priority ?*PRIORITY-PRODUCE-C0*) (lock PREPARE-BS)) ;is released after get-base
;     (step (name instruct-mps) (id (+ ?task-id 3))
;       (task-priority ?*PRIORITY-PRODUCE-C0*)
;       (machine ?bs) (base ?base-color) (side ?bs-side))
;     (step (name get-base) (id (+ ?task-id 4))
;       (task-priority ?*PRIORITY-PRODUCE-C0*)
;       (machine ?bs) (machine-feature CONVEYOR)
;       (base ?base-color) (product-id ?product-id) (side ?bs-side))
;     (step (name drive-to) (id (+ ?task-id 5))
;       (task-priority ?*PRIORITY-PRODUCE-C0*)
;       (machine ?cs)
;       (side INPUT))
;     (step (name insert) (id (+ ?task-id 6))
;       (task-priority ?*PRIORITY-PRODUCE-C0*)
;       (machine ?cs)
;       (machine-feature CONVEYOR))
;     (needed-task-lock (task-id ?task-id) (action PROD_CAP) (place ?cs))
;   )
;   (synced-modify ?of in-production 2)
; )

; (defrule prod-add-first-ring
;   "Add first ring to a product"
;   (declare (salience ?*PRIORITY-ADD-FIRST-RING*))
;   (phase PRODUCTION)
;   (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;   (team-color ?team-color&~nil)
;   (holding NONE)
;   (game-time $?game-time)
;   (product
;     (id ?product-id)
;     (rings $?r&:(> (length$ ?r) 0))
;     (base ?base-color)
;   )
;   (ring (color ?ring-color&:(eq (nth$ 1 ?r) ?ring-color)) (req-bases ?rb))
;   (machine (mtype RS) (incoming $?i&~:(member$ PROD_RING ?i))
;            (name ?rs) (team ?team-color) (produced-id 0)
;            (state ~DOWN&~BROKEN)
;   )
;   (ring-station
;     (name ?rs)
;     (available-colors $?ac&:(member$ ?ring-color ?ac))
;     (bases-loaded ?bl&:(>= ?bl ?rb))
;   )
;   (found-tag (name ?rs))
;   (machine (mtype BS)
;     (name ?bs) (team ?team-color)
;     (state ~DOWN&~BROKEN)
;   )
;   (base-station (name ?bs) (active-side ?bs-side) (fail-side ?fs&:(neq ?bs-side ?fs)))
;   (found-tag (name ?bs))
;   ;check that the task was not rejected before
;   (not (and (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (name add-first-ring) (state rejected) (id ?rej-id))
;             (step (name insert) (id ?rej-st&:(eq ?rej-st (+ ?rej-id 6))) (machine ?rs))))
;   (not (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-ADD-FIRST-RING*))))
;   ?of <- (order (product-id ?product-id)
;     (quantity-requested ?qr) (quantity-delivered ?qd&:(> ?qr ?qd))
;     (complexity ?complexity)
;     (begin ?begin&:(< ?begin (+ (nth$ 1 ?game-time) (tac-ring-mount-time ?complexity 0))))
;     (end ?end&:(> ?end (+ (nth$ 1 ?game-time) ?*PRODUCE-CX-LATEST-TIME*)))
;     (in-production 0) (in-delivery ?id&:(> ?qr (+ ?qd ?id)))
;   )
;   =>
;   (printout warn "TODO: production durations not yet implemented")
;   (printout t "PROD: Add first ring to Cx with " ?ring-color " ring at " ?rs crlf)
;   (bind ?task-id (random-id))
;   (assert (task (name add-first-ring) (id ?task-id) (state proposed)
;     (steps (create$ (+ ?task-id 1) (+ ?task-id 2) (+ ?task-id 3) (+ ?task-id 4) (+ ?task-id 5) (+ ?task-id 6)))
;     (priority ?*PRIORITY-ADD-FIRST-RING*))
;     (step (name drive-to) (id (+ ?task-id 1))
;       (task-priority ?*PRIORITY-ADD-FIRST-RING*)
;       (machine ?bs)
;       (side ?bs-side))
;     (step (name acquire-lock) (id (+ ?task-id 2))
;       (task-priority ?*PRIORITY-ADD-FIRST-RING*) (lock PREPARE-BS)) ;is released after get-base
;     (step (name instruct-mps) (id (+ ?task-id 3))
;       (task-priority ?*PRIORITY-ADD-FIRST-RING*)
;       (machine ?bs) (base ?base-color) (lock PREPARE-BS) (side ?bs-side))
;     (step (name get-base) (id (+ ?task-id 4))
;       (task-priority ?*PRIORITY-ADD-FIRST-RING*)
;       (machine ?bs) (machine-feature CONVEYOR)
;       (base ?base-color) (product-id ?product-id) (side ?bs-side))
;     (step (name drive-to) (id (+ ?task-id 5))
;       (task-priority ?*PRIORITY-ADD-FIRST-RING*)
;       (machine ?rs)
;       (side INPUT))
;     (step (name insert) (id (+ ?task-id 6))
;       (task-priority ?*PRIORITY-ADD-FIRST-RING*)
;       (machine ?rs)
;       (machine-feature CONVEYOR)
;       (ring ?ring-color))
;     (needed-task-lock (task-id ?task-id) (action PROD_RING) (place ?rs))
;   )
;   (synced-modify ?of in-production 1)
; )

; (defrule prod-add-additional-ring
;   "Add ring 2 or 3 to a product"
;   (declare (salience ?*PRIORITY-ADD-ADDITIONAL-RING*))
;   (phase PRODUCTION)
;   (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;   (team-color ?team-color&~nil)
;   (holding NONE)
;   (game-time $?game-time)
;   (product
;     (id ?product-id)
;     (rings $?r&:(> (length$ ?r) 1))
;   )
;   (product
;     (id ?produced-id)
;     (product-id ?product-id)
;     (rings $?assembled&:(> (length$ ?r) (length$ ?assembled)))
;   )
;   (ring (color ?ring-color&:(eq (nth$ (+ (length$ ?assembled) 1) ?r) ?ring-color)) (req-bases ?rb))
;   (machine (mtype RS) (incoming $?i&~:(member$ PROD_RING ?i))
;            (name ?rs) (team ?team-color) (produced-id 0|?produced-id)
;            (state ~DOWN&~BROKEN)
;   )
;   (ring-station
;     (name ?rs)
;     (available-colors $?ac&:(member$ ?ring-color ?ac))
;     (bases-loaded ?bl&:(>= ?bl ?rb))
;   )
;   (found-tag (name ?rs))
;   (machine (mtype RS)
;     (name ?oldrs) (team ?team-color) (produced-id ?produced-id)
;     (state READY-AT-OUTPUT)
;   )
;   (not (and (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (name add-additional-ring) (state rejected) (id ?rej-id))
;             (step (name insert) (id ?rej-st&:(eq ?rej-st (+ ?rej-id 4))) (machine ?rs))))
;   (not (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-ADD-ADDITIONAL-RING*))))
;   ?of <- (order (product-id ?product-id)
;     (quantity-requested ?qr) (quantity-delivered ?qd&:(> ?qr ?qd))
;     (complexity ?complexity)
;     (begin ?begin&:(< ?begin (+ (nth$ 1 ?game-time) (tac-ring-mount-time ?complexity (length ?assembled)))))
;     (end ?end&:(> ?end (+ (nth$ 1 ?game-time) ?*PRODUCE-CX-LATEST-TIME*)))
;     (in-production 1) (in-delivery ?id&:(> ?qr (+ ?qd ?id)))
;   )
;   =>
;   (printout t "PROD: Add additional ring to Cx with " ?ring-color " ring at " ?rs crlf)
;   (bind ?task-id (random-id))
;   (assert (task (name add-additional-ring) (id ?task-id) (state proposed)
;     (steps (create$ (+ ?task-id 1) (+ ?task-id 2) (+ ?task-id 3) (+ ?task-id 4)))
;     (priority ?*PRIORITY-ADD-ADDITIONAL-RING*))
;     (step (name drive-to) (id (+ ?task-id 1))
;       (task-priority ?*PRIORITY-ADD-ADDITIONAL-RING*)
;       (machine ?oldrs)
;       (side OUTPUT))
;     (step (name get-output) (id (+ ?task-id 2))
;       (task-priority ?*PRIORITY-ADD-ADDITIONAL-RING*)
;       (machine ?oldrs) (machine-feature CONVEYOR)
;       (product-id ?product-id))
;     (step (name drive-to) (id (+ ?task-id 3))
;       (task-priority ?*PRIORITY-ADD-ADDITIONAL-RING*)
;       (machine ?rs)
;       (side INPUT))
;     (step (name insert) (id (+ ?task-id 4))
;       (task-priority ?*PRIORITY-ADD-ADDITIONAL-RING*)
;       (machine ?rs)
;       (machine-feature CONVEYOR)
;       (ring ?ring-color))
;     (needed-task-lock (task-id ?task-id) (action PROD_RING) (place ?rs))
;   )
; )

; (defrule prod-add-additional-ring-with-waiting
;   "Add ring 2 or 3 to a product while waiting with the product to free
;    the machine for an other bot wanting to use it
;    (lower priority, than without waiting)"
;   (declare (salience ?*PRIORITY-ADD-ADDITIONAL-RING-WAITING*))
;   (phase PRODUCTION)
;   (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;   (team-color ?team-color&~nil)
;   (holding NONE)
;   (game-time $?game-time)
;   (product
;     (id ?product-id)
;     (rings $?r&:(> (length$ ?r) 1))
;   )
;   (product
;     (id ?produced-id)
;     (product-id ?product-id)
;     (rings $?assembled&:(> (length$ ?r) (length$ ?assembled)))
;   )
;   (ring (color ?ring-color&:(eq (nth$ (+ (length$ ?assembled) 1) ?r) ?ring-color)) (req-bases ?rb))
;   (machine (mtype RS) (name ?rs) (team ?team-color) (produced-id 0|?produced-id)
;            (incoming $?i&:(member$ PROD_RING ?i)) ;other bot wants to use that machine but has to wait for us
;            ;(state ~DOWN&~BROKEN) state is not important
;   )
;   (ring-station
;     (name ?rs)
;     (available-colors $?ac&:(member$ ?ring-color ?ac))
;   )
;   (found-tag (name ?rs))
;   (machine (mtype RS)
;     (name ?oldrs) (team ?team-color) (produced-id ?produced-id)
;     (state READY-AT-OUTPUT)
;   )
;   (not (and (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (name add-additional-ring) (state rejected) (id ?rej-id))
;             (step (name insert) (id ?rej-st&:(eq ?rej-st (+ ?rej-id 5))) (machine ?rs))))
;   (not (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-ADD-ADDITIONAL-RING-WAITING*))))
;   ?of <- (order (product-id ?product-id)
;     (quantity-requested ?qr) (quantity-delivered ?qd&:(> ?qr ?qd))
;     (begin ?begin&:(< ?begin (+ (nth$ 1 ?game-time) ?*PRODUCE-CX-AHEAD-TIME*)))
;     (end ?end&:(> ?end (+ (nth$ 1 ?game-time) ?*PRODUCE-CX-LATEST-TIME*)))
;     (in-production 1) (in-delivery ?id&:(> ?qr (+ ?qd ?id)))
;   )
;   =>
;   (printout t "PROD: Add additional ring to Cx with " ?ring-color " ring at " ?rs crlf)
;   (printout t "Waiting with retrieved product before mounting to free RS" crlf)
;   (bind ?task-id (random-id))
;   (assert (task (name add-additional-ring) (id ?task-id) (state proposed)
;     (steps (create$ (+ ?task-id 1) (+ ?task-id 2) (+ ?task-id 3) (+ ?task-id 4) (+ ?task-id 5)))
;     (priority ?*PRIORITY-ADD-ADDITIONAL-RING-WAITING*))
;     (step (name drive-to) (id (+ ?task-id 1))
;       (task-priority ?*PRIORITY-ADD-ADDITIONAL-RING-WAITING*)
;       (machine ?oldrs)
;       (side OUTPUT))
;     (step (name get-output) (id (+ ?task-id 2))
;       (task-priority ?*PRIORITY-ADD-ADDITIONAL-RING-WAITING*)
;       (machine ?oldrs) (machine-feature CONVEYOR)
;       (product-id ?product-id))
;     (step (name wait-for-rs) (id (+ ?task-id 3))
;           (task-priority ?*PRIORITY-ADD-ADDITIONAL-RING-WAITING*)
;           (machine ?rs) (ring ?ring-color))
;     (step (name drive-to) (id (+ ?task-id 4))
;       (task-priority ?*PRIORITY-ADD-ADDITIONAL-RING-WAITING*)
;       (machine ?rs)
;       (side INPUT))
;     (step (name insert) (id (+ ?task-id 5))
;       (task-priority ?*PRIORITY-ADD-ADDITIONAL-RING-WAITING*)
;       (machine ?rs)
;       (machine-feature CONVEYOR)
;       (ring ?ring-color))
;   )
; )

; (defrule prod-produce-cx
;   "Produce a Cx"
;   (declare (salience ?*PRIORITY-PRODUCE-CX*))
;   (phase PRODUCTION)
;   (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;   (team-color ?team-color&~nil)
;   (holding NONE)
;   (game-time $?game-time)
;   (product
;     (id ?produced-id)
;     (product-id ?product-id)
;     (rings $?rp)
;     (cap NONE)
;   )
;   (machine (mtype CS) (incoming $?i&~:(member$ PROD_CAP ?i))
;            (name ?cs) (team ?team-color) (produced-id 0)
;            (state ~DOWN&~BROKEN))
;   (cap-station (name ?cs) (cap-loaded ?cap-color) (assigned-cap-color ?cap-color))
;   (found-tag (name ?cs))
;   (machine (mtype RS)
;     (name ?rs) (team ?team-color)
;     (state ~DOWN&~BROKEN)
;     (produced-id ?produced-id)
;   )
;   (found-tag (name ?rs))
;   ;check that the task was not rejected before
;   (not (and (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (name produce-cx) (state rejected) (id ?rej-id))
;             (step (name insert) (id ?rej-st&:(eq ?rej-st (+ ?rej-id 3))) (machine ?cs))))
;   (not (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-PRODUCE-CX*))))
;   ;check for open C0 order
;   (product
;     (id ?product-id)
;     (rings $?r&:(and (> (length$ ?r) 0) (eq ?rp ?r)))
;     (cap ?cap-color)
;   )
;   ?of <- (order (product-id ?product-id)
;     (quantity-requested ?qr) (quantity-delivered ?qd&:(> ?qr ?qd))
;     (begin ?begin&:(< ?begin (+ (nth$ 1 ?game-time) ?*PRODUCE-CX-AHEAD-TIME*)))
;     (end ?end&:(> ?end (+ (nth$ 1 ?game-time) ?*PRODUCE-CX-LATEST-TIME*)))
;     (in-production 1) (in-delivery ?id&:(> ?qr (+ ?qd ?id)))
;   )
;   =>
;   (printout warn "TODO: production durations not yet implemented")
;   (printout t "PROD: PRODUCE Cx with " ?cap-color " cap at " ?cs crlf)
;   (bind ?task-id (random-id))
;   (assert (task (name produce-cx) (id ?task-id) (state proposed)
;     (steps (create$ (+ ?task-id 1) (+ ?task-id 2) (+ ?task-id 3)))
;     (priority ?*PRIORITY-PRODUCE-CX*))
;     (step (name get-output) (id (+ ?task-id 1))
;       (task-priority ?*PRIORITY-PRODUCE-CX*)
;       (machine ?rs) (machine-feature CONVEYOR))
;     (step (name drive-to) (id (+ ?task-id 2))
;       (task-priority ?*PRIORITY-PRODUCE-CX*)
;       (machine ?cs)
;       (side INPUT))
;     (step (name insert) (id (+ ?task-id 3))
;       (task-priority ?*PRIORITY-PRODUCE-CX*)
;       (machine ?cs)
;       (machine-feature CONVEYOR))
;     (needed-task-lock (task-id ?task-id) (action PROD_CAP) (place ?cs))
;   )
; )

; (defrule prod-deliver
;   "Deliver product"
;   (declare (salience ?*PRIORITY-DELIVER*))
;   (phase PRODUCTION)
;   (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;   (team-color ?team-color&~nil)
;   (holding NONE)
;   (game-time $?game-time)
;   (machine (mtype CS) (produced-id ?produced-id&~0)
;            (name ?cs) (team ?team-color) (incoming $?i&~:(member$ GET-PROD ?i))
;            (state ~DOWN&~BROKEN))
;   (product
;     (id ?produced-id)
;     (product-id ?product-id)
;     (cap ?cap-color)
;   )
;   ?ds-f <- (machine (mtype DS)
;                     (name ?ds) (team ?team-color)
;                     (state ~DOWN&~BROKEN))
;   ;check that the task was not rejected before
;   (not (and (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (name deliver) (state rejected) (id ?rej-id))
;             (step (name get-output) (id ?rej-st&:(eq ?rej-st (+ ?rej-id 2))) (machine ?cs))))
;   (not (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-DELIVER*))))
;   ?of <- (order (product-id ?product-id)
;     (quantity-requested ?qr) (quantity-delivered ?qd&:(> ?qr ?qd))
;     (begin ?begin&:(< ?begin (+ (nth$ 1 ?game-time) ?*DELIVER-AHEAD-TIME*)))
;     (delivery-gate ?gate) (in-production ?ip&:(> ?ip 0)) (in-delivery ?id)
;   )
;   =>
;   (printout t "PROD: DELIVER C0 with " ?cap-color crlf)
;   (bind ?task-id (random-id))
;   (assert (task (name deliver) (id ?task-id) (state proposed)
;     (steps (create$ (+ ?task-id 1) (+ ?task-id 2) (+ ?task-id 3) (+ ?task-id 4)))
;     (priority ?*PRIORITY-DELIVER*))
;     (step (name drive-to) (id (+ ?task-id 1))
;       (task-priority ?*PRIORITY-PRODUCE-CX*)
;       (machine ?cs)
;       (side OUTPUT))
;     (step (name get-output) (id (+ ?task-id 2))
;       (task-priority ?*PRIORITY-DELIVER*)
;       (machine ?cs))
;     (step (name drive-to) (id (+ ?task-id 3))
;       (task-priority ?*PRIORITY-PRODUCE-CX*)
;       (machine ?ds)
;       (side INPUT))
;     (step (name insert) (id (+ ?task-id 4))
;       (task-priority ?*PRIORITY-DELIVER*)
;       (machine ?ds)
;       (machine-feature CONVEYOR)
;       (gate ?gate))
;     (needed-task-lock (task-id ?task-id) (action GET-PROD) (place ?cs))
;     (needed-task-lock (task-id ?task-id) (action DELIVER) (place ?ds))
;   )
; )

; ;(defrule prod-remove-product-with-expired-order-from-cs
; ;  "If we couldn't deliver a product in time and it is still in the cs, we have to clear the cs. In the same task we can fill the cs with a cap again"
; ;  (declare (salience ?*PRIORITY-CLEAR-CS*))
; ;  (phase PRODUCTION)
; ;  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
; ;  (team-color ?team-color&~nil)
; ;  (holding NONE)
; ;  (game-time $?game-time)
; ;  (machine (mtype CS) (produced-id ?produced-id&~0)
; ;           (name ?cs) (team ?team-color)
; ;           (state ~DOWN&~BROKEN))
; ;  (product
; ;    (id ?produced-id)
; ;    (product-id ?product-id)
; ;    (cap ?cap-color)
; ;  )
; ;  ;check that the task was not rejected before
; ;  (not (and (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (name clear-cs) (state rejected) (id ?rej-id))
; ;            (step (name insert) (id ?rej-st&:(eq ?rej-st (+ ?rej-id 2))) (machine ?cs))))
; ;  (not (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-CLEAR-CS*))))
; ;  (order (product-id ?product-id)
; ;         (end ?end&:(< ?end (nth$ 1 ?game-time)))
; ;  )
; ;  =>
; ;  (printout t "PROD: Clear " ?cs " and fill it again because the order is over" crlf)
; ;  (bind ?task-id (random-id))
; ;  (assert (task (name clear-cs) (id ?task-id) (state proposed)
; ;                (steps (create$ (+ ?task-id 1) (+ ?task-id 2) (+ ?task-id 3)))
; ;                (priority ?*PRIORITY-CLEAR-CS*))
; ;    (step (name get-output) (id (+ ?task-id 1))
; ;          (task-priority ?*PRIORITY-CLEAR-CS*)
; ;          (machine ?cs))
; ;    (step (name insert) (id (+ ?task-id 2))
; ;          (task-priority ?*PRIORITY-CLEAR-CS*)
; ;          (machine ?cs)
; ;          (machine-feature CONVEYOR))
; ;    (step (name get-output) (id (+ ?task-id 3))
; ;          (task-priority ?*PRIORITY-CLEAR-CS*)
; ;          (machine ?cs))
; ;    (needed-task-lock (task-id ?task-id) (action GET-PROD) (place ?cs))
; ;    (needed-task-lock (task-id ?task-id) (action FILL_CAP) (place ?cs))
; ;  )
; ;)

; (defrule prod-remove-product-with-expired-order-from-rs
;   "If we couldn't deliver a product in time and it is still in the rs, we have to clear the rs. In the same task we can fill the rs with a cap again"
;   (declare (salience ?*PRIORITY-CLEAR-RS*))
;   (phase PRODUCTION)
;   (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;   (team-color ?team-color&~nil)
;   (holding NONE)
;   (game-time $?game-time)
;   (machine (mtype RS) (produced-id ?produced-id&~0)
;            (name ?rs) (team ?team-color)
;            (state ~DOWN&~BROKEN))
;   (product
;     (id ?produced-id)
;     (product-id ?product-id)
;   )
;   ;check that the task was not rejected before
;   (not (and (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (name clear-rs) (state rejected) (id ?rej-id))
;             (step (name get-output) (id ?rej-st&:(eq ?rej-st (+ ?rej-id 1))) (machine ?rs))))
;   (not (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-CLEAR-RS*))))
;   (order (product-id ?product-id)
;          (end ?end&:(< ?end (nth$ 1 ?game-time)))
;   )
;   =>
;   (printout t "PROD: Clear " ?rs " because the order is over" crlf)
;   (bind ?task-id (random-id))
;   (assert (task (name clear-rs) (id ?task-id) (state proposed)
;                 (steps (create$ (+ ?task-id 1)))
;                 (priority ?*PRIORITY-CLEAR-RS*))
;     (step (name get-output) (id (+ ?task-id 1))
;           (task-priority ?*PRIORITY-CLEAR-RS*)
;           (machine ?rs))
;     (needed-task-lock (task-id ?task-id) (action GET-PROD) (place ?rs))
;   )
; )

; (defrule prod-find-missing-mps-exploration-catch-up
;   "If we have not found all mps until the production phase, we have to find them now."
;   (declare (salience ?*PRIORITY-FIND-MISSING-MPS*))
;   (phase PRODUCTION)
;   (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;   (team-color ?team-color&~nil)
;   (holding NONE)
;   ; there is a mps not found jet
;   (machine (name ?missing-mps) (team ?team-color))
;   (not (found-tag (name ?missing-mps)))
;   ; zone-to-explore
;   ?z-f <- (zone-exploration (name ?zone) (machine ?missing-mps)
;                             (still-to-explore TRUE) (team ?team-color)
;                             (incoming $?i&~:(member$ FIND_TAG ?i))
;                             (times-searched ?times-searched))
;   ;check that the task was not rejected before
;   (not (and (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (name exploration-catch-up) (state rejected) (id ?rej-id))
; 	    (step (name find-tag) (id ?rej-st&:(eq ?rej-st (+ ?rej-id 1))) (zone ?zone))))
;   (not (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-FIND-MISSING-MPS*))))
;   =>
;   (printout t "PROD: " ?missing-mps " still not found!"
;             " Searching for it in zone " ?zone crlf)
;   (bind ?task-id (random-id))
;   (assert (task (name exploration-catch-up) (id ?task-id) (state proposed)
; 		(steps (create$ (+ ?task-id 1)))
; 		(priority ?*PRIORITY-FIND-MISSING-MPS*))
; 	  (step (name find-tag) (id (+ ?task-id 1))
; 		(task-priority ?*PRIORITY-FIND-MISSING-MPS*)
; 		(zone ?zone) (machine ?missing-mps))
; 	  (needed-task-lock (task-id ?task-id) (action FIND_TAG) (place ?zone))
;   )
; )

; (defrule prod-nothing-to-do-save-factbase
;   "If the agent can't find any task, save the factbase to find problems"
;   (declare (salience ?*PRIORITY-NOTHING-TO-DO*))
;   (phase PRODUCTION)
;   (state IDLE)
;   (time $?now)
;   (game-time $?game-time)
;   (not (no-task-found ?))
;   (not (task (robot ?robot&:(eq ?robot ?*ROBOT-NAME*)) (state proposed|asked|ordered|running)))
;   =>
;   (printout error "Can't find any task!." crlf)
;   (printout error " Waiting..." crlf)
;   (save-facts (str-cat "agent-snapshot-no-task" (nth$ 1 ?now) ".clp") visible)

;   ;find random waiting point
;   (bind ?wpts (create$))
;   (do-for-all-facts ((?wpt zone-waitpoint)) TRUE
;     (bind ?wpts (create$ ?wpt:name ?wpts))
;   )
;   (bind ?wait-point WAIT1)
;   (if (> (length$ ?wpts) 0) then
;     (bind ?index (random 1 (length$ ?wpts)))
;     (bind ?wait-point (nth$ ?index ?wpts))
;   )
;   (skill-call ppgoto place (str-cat ?wait-point))
;   (assert (no-task-found (nth$ 1 ?game-time)))
; )

; (defrule prod-remove-nothing-to-do-fact
;   ?no-task <- (no-task-found ?)
;   (state ~IDLE)
;   =>
;   (retract ?no-task)
; )

; (defrule prod-remove-rejected-tasks-after-timeout
;   ?no-task <- (no-task-found ?waiting-since)
;   (game-time $?game-time&:(< (+ ?waiting-since ?*TIMEOUT-REMOVE-REJECTED-WHILE-WAITING*)
;                              (nth$ 1 ?game-time)))
;   (state IDLE)
;   =>
;   (printout t "Removing rejected tasks after waiting for "
;             ?*TIMEOUT-REMOVE-REJECTED-WHILE-WAITING* "s" crlf)
;   (coordination-remove-old-rejected-tasks)
;   (retract ?no-task)
;   (assert (no-task-found (nth$ 1 ?game-time)))
; )
