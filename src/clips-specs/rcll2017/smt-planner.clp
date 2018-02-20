
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

   ; set available rings for ring-stations
    (if (eq ?mtype RS) then
      (do-for-fact ((?rs ring-station)) (eq ?rs:name ?name)
        (if (neq 0 (length$ ?rs:available-colors)) then
          (bind ?rlist (create$))
          (progn$ (?r ?rs:available-colors)
            (switch ?r
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
      )
    )

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
  (do-for-all-facts ((?r ring)) TRUE
    (bind ?rv (append$ ?rv (smt-create-ring ?r:color ?r:req-bases)))
  )
  (return ?rv)
)
