
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
		(switch (wm-key-arg ?wm-fact:key r)
		  (case GREEN then (bind ?rlist (append$ ?rlist "RING_GREEN")))
		  (case BLUE then (bind ?rlist (append$ ?rlist "RING_BLUE")))
		  (case ORANGE then (bind ?rlist (append$ ?rlist "RING_ORANGE")))
		  (case YELLOW then (bind ?rlist (append$ ?rlist "RING_YELLOW")))
		  (default (printout warn "Ring color not found" crlf))
		)
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
		(bind ?rv (append$ ?rv (smt-create-machine (wm-key-arg ?wm-fact:key m) (wm-key-arg ?wm-fact2:key t) (wm-key-arg ?wm-fact:key s) ?team-color 0 0))) ; TODO Do we only have access to machines from our team?
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

	; Extract order-base-color
	(do-for-all-facts ((?wm-fact wm-fact))
		(and
			(wm-key-prefix ?wm-fact:key (create$ domain fact order-base-color))
			(eq ?id (wm-key-arg ?wm-fact:key ord))
		)
		(switch (wm-key-arg ?wm-fact:key col)
		  (case BLACK then (pb-set-field ?o "cap_color" "CAP_BLACK"))
		  (case GREY then (pb-set-field ?o "cap_color" "CAP_GREY"))
		  (default (printout warn "Cap color not found" crlf))
		)
	)
	; Extract order-cap-color
	(do-for-all-facts ((?wm-fact wm-fact))
		(and
			(wm-key-prefix ?wm-fact:key (create$ domain fact order-cap-color))
			(eq ?id (wm-key-arg ?wm-fact:key ord))
		)
		(switch (wm-key-arg ?wm-fact:key col)
		  (case BLACK then (pb-set-field ?o "base_color" "BASE_BLACK"))
		  (case RED then (pb-set-field ?o "base_color" "BASE_RED"))
		  (case SILVER then (pb-set-field ?o "base_color" "BASE_SILVER"))
		  (default (printout warn "Base color not found" crlf))
		) 
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
	(do-for-all-facts ((?wm-fact wm-fact) (?wm-fact2 wm-fact) (?wm-fact3 wm-fact) (?wm-fact4 wm-fact) (?wm-fact5 wm-fact) (?wm-fact6 wm-fact))
		(and
			(wm-key-prefix ?wm-fact:key (create$ domain fact order-complexity))
			(wm-key-prefix ?wm-fact2:key (create$ domain fact order-gate))
			(eq (wm-key-arg ?wm-fact:key ord) (wm-key-arg ?wm-fact2:key ord))
			(eq ?wm-fact3:key (create$ refbox order (wm-key-arg ?wm-fact:key ord) quantity-requested))
			(eq ?wm-fact4:key (create$ refbox order (wm-key-arg ?wm-fact:key ord) quantity-delivered))
			(eq ?wm-fact5:key (create$ refbox order (wm-key-arg ?wm-fact:key ord) delivery-begin))
			(eq ?wm-fact6:key (create$ refbox order (wm-key-arg ?wm-fact:key ord) delivery-end))
		)
		(bind ?rv (append$ ?rv (smt-create-order
									(wm-key-arg ?wm-fact:key ord) ; order-id
									(wm-key-arg ?wm-fact:key ord) ; order-product-id TODO What is the difference
									(wm-key-arg ?wm-fact:key gate)
									(wm-key-arg ?wm-fact:key com)
									?wm-fact3:value ; quantity-requested
									?wm-fact4:value ; quantity-delivered
									?wm-fact5:value ; begin
									?wm-fact6:value ; end
									?team-color)))
	)
	(return ?rv)
)

(deffunction smt-create-ring (?ring-color ?req-bases)
  (printout t "Creating Data msgs:: Ring color " ?ring-color  crlf)
  (bind ?r (pb-create "llsf_msgs.Ring"))
  (pb-set-field ?r "raw_material" ?req-bases)
  (switch ?ring-color
    (case GREEN then (pb-set-field ?r "ring_color" "RING_GREEN"))
    (case BLUE then (pb-set-field ?r "ring_color" "RING_BLUE"))
    (case ORANGE then (pb-set-field ?r "ring_color" "RING_ORANGE"))
    (case YELLOW then (pb-set-field ?r "ring_color" "RING_YELLOW"))
    (default (printout warn "Ring color not found" crlf))
    )
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
  (goal (id COMPLEXITY))
  ; (phase PRODUCTION)
  ; (team-color ?team-color&CYAN|MAGENTA)
  ; (state IDLE)
  ; (not (plan-requested))
  ; (test (eq ?*ROBOT-NAME* "R-1"))
  ; (exists (machine))
  ; (exists (order))
  ; (exists (ring))
  ; (exists (ring-station))
  ; (exists (order (complexity C3)))
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
