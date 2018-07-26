
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
(deffunction smt-create-data (?robots ?machines ?orders ?rings ?strategy ?window)
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
	(pb-set-field ?p "strategy" ?strategy)
	(pb-set-field ?p "window" ?window)

	(printout t "Proto:" (pb-tostring ?p) crlf)
	(return ?p)
)

; Add robot information from worldmodel to protobuf
(deffunction smt-create-robot (?name ?team-color ?number ?pose-x ?pose-y)
	(bind ?rSMT (pb-create "llsf_msgs.RobotSmt"))

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

	(printout t "Look for positions of robots" crlf)
	(bind ?location "START-I")
	(if (eq ?name R-1) then
		(do-for-fact ((?wm-fact wm-fact)) (wm-key-prefix ?wm-fact:key (create$ r-1-at position))
			(printout t "Found position of R-1 " ?wm-fact:value crlf)
			(bind ?location ?wm-fact:value)
		)
	else
		(if (eq ?name R-2) then
			(do-for-fact ((?wm-fact wm-fact)) (wm-key-prefix ?wm-fact:key (create$ r-2-at position))
				(printout t "Found position of R-2 " ?wm-fact:value crlf)
				(bind ?location ?wm-fact:value)
			)
		else
			(if (eq ?name R-3) then
				(do-for-fact ((?wm-fact wm-fact)) (wm-key-prefix ?wm-fact:key (create$ r-3-at position))
					(printout t "Found position of R-3 " ?wm-fact:value crlf)
					(bind ?location ?wm-fact:value)
				)
			)
		)
	)
	(pb-set-field ?rSMT "location" ?location)

	; Add information for robot holding wp
	; Fill dummy information for required fields in llsf_msgs.Order as we are only interested in the colors of the wp
	(bind ?wp (pb-create "llsf_msgs.Workpiece"))
	; Prepare colors in case product is not completed or a cap carrier
	(bind ?holding-cap "CAP_NONE_WP")
	(bind ?holding-ring1 "RING_NONE_WP")
	(bind ?holding-ring2 "RING_NONE_WP")
	(bind ?holding-ring3 "RING_NONE_WP")
	(bind ?holding-base "BASE_NONE_WP")

	; Detect if some robot is currently holding wp
	(do-for-fact ((?holding wm-fact))
		(and
			(wm-key-prefix ?holding:key (create$ domain fact holding))
			(eq ?name (wm-key-arg ?holding:key r))
		)

		(bind ?holding-wp (wm-key-arg ?holding:key wp))
		(if ; cap_carrier_grey
			(or
				(eq ?holding-wp CCG1)
				(eq ?holding-wp CCG2)
				(eq ?holding-wp CCG3)
			)
		then
			(bind ?holding-cap "CAP_GREY_WP")
			(bind ?holding-base "BASE_RANDOM_WP")
		else
			(if ; cap_carrier_black
				(or
					(eq ?holding-wp CCB1)
					(eq ?holding-wp CCB2)
					(eq ?holding-wp CCB3)
				)
			then
				(bind ?holding-cap "CAP_BLACK_WP")
				(bind ?holding-base "BASE_RANDOM_WP")
			else
				(if ; WP1 (product) or WP2 (additional base)
					(or
						(eq ?holding-wp WP1)
						(eq ?holding-wp WP2)
					)
				then
					; wp-base-color
					(do-for-fact ((?wm-fact wm-fact))
						(and
							(wm-key-prefix ?wm-fact:key (create$ domain fact wp-base-color))
							(eq ?holding-wp (wm-key-arg ?wm-fact:key wp))
						)

						; TODO Needs to be tested, because this case only appears for replaining
						(bind ?holding-base (str-cat (wm-key-arg ?wm-fact:key col) _WP))
					)

					; wp-cap-color
					(do-for-fact ((?wm-fact wm-fact))
						(and
							(wm-key-prefix ?wm-fact:key (create$ domain fact wp-cap-color))
							(eq ?holding-wp (wm-key-arg ?wm-fact:key wp))
						)

						(bind ?holding-cap (str-cat (wm-key-arg ?wm-fact:key col) _WP))
					)

					; wp-ring-color
					(do-for-fact ((?wm-fact wm-fact))
						(and
							(wm-key-prefix ?wm-fact:key (create$ domain fact wp-ring1-color))
							(eq ?holding-wp (wm-key-arg ?wm-fact:key wp))
						)

						(bind ?holding-ring1 (str-cat (wm-key-arg ?wm-fact:key col) _WP))
					)
					(do-for-fact ((?wm-fact wm-fact))
						(and
							(wm-key-prefix ?wm-fact:key (create$ domain fact wp-ring2-color))
							(eq ?holding-wp (wm-key-arg ?wm-fact:key wp))
						)

						(bind ?holding-ring2 (str-cat (wm-key-arg ?wm-fact:key col) _WP))
					)
					(do-for-fact ((?wm-fact wm-fact))
						(and
							(wm-key-prefix ?wm-fact:key (create$ domain fact wp-ring3-color))
							(eq ?holding-wp (wm-key-arg ?wm-fact:key wp))
						)

						(bind ?holding-ring3 (str-cat (wm-key-arg ?wm-fact:key col) _WP))
					)
				else
					(printout t "No known wp in holding" crlf)
				)

			)
		)
	)
	(pb-set-field ?wp "cap_color" ?holding-cap)
	(pb-add-list ?wp "ring_colors" ?holding-ring1)
	(pb-add-list ?wp "ring_colors" ?holding-ring2)
	(pb-add-list ?wp "ring_colors" ?holding-ring3)
	(pb-set-field ?wp "base_color" ?holding-base)

	(pb-set-field ?rSMT "wp" ?wp)

	(pb-set-field ?rSMT "robot" ?r)

	(return ?rSMT)
)

(deffunction smt-create-robots (?team-color)
	(bind ?rv (create$))

	; (do-for-all-facts ((?wm-fact wm-fact))
	;     (wm-key-prefix ?wm-fact:key (create$ domain objects-by-type robot))

	;     (foreach ?robot ?wm-fact:values
	;         (bind ?rv (append$ ?rv (smt-create-robot (string-to-field ?robot) ?team-color 0 0 0)))
	;     )

	; )

	(bind ?rv (append$ ?rv (smt-create-robot (string-to-field "R-1") ?team-color 0 0 0)))
	(bind ?rv (append$ ?rv (smt-create-robot (string-to-field "R-2") ?team-color 0 0 0)))
	(bind ?rv (append$ ?rv (smt-create-robot (string-to-field "R-3") ?team-color 0 0 0)))

	(return ?rv)
)

; Add machine information from worldmodel to protobuf
(deffunction smt-create-machine (?name ?mtype ?state ?team-color ?pose-x ?pose-y)
	(bind ?mSMT (pb-create "llsf_msgs.MachineSmt"))

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
		(bind ?cs_buffered "CAP_NONE")

		; set amount of additional bases machine is loaded with
		(do-for-fact ((?wm-fact wm-fact))
			(and
				(wm-key-prefix ?wm-fact:key (create$ domain fact cs-buffered))
				(eq ?name (wm-key-arg ?wm-fact:key m))
			)

			(bind ?cs_buffered (wm-key-arg ?wm-fact:key col))
		)
		(pb-set-field ?mSMT "cs_buffered" ?cs_buffered)

                (do-for-fact ((?wm-fact wm-fact))
			(and
				(wm-key-prefix ?wm-fact:key (create$ domain fact cs-color))
				(eq ?name (wm-key-arg ?wm-fact:key m))
			)

			(bind ?cs_color (wm-key-arg ?wm-fact:key col))
		)
		(pb-set-field ?mSMT "cap_color" ?cs_color)

	)

	; Add information for machine with wp-at
	; Fill dummy information for required fields in llsf_msgs.Order as we are only interested in the colors of the wp
	(bind ?wp (pb-create "llsf_msgs.Workpiece"))
	(bind ?wp-at-cap "CAP_NONE_WP")
	(bind ?wp-at-ring1 "RING_NONE_WP")
	(bind ?wp-at-ring2 "RING_NONE_WP")
	(bind ?wp-at-ring3 "RING_NONE_WP")
	(bind ?wp-at-base "BASE_NONE_WP")

	; Detect if some robot is currently wp-at wp
	(do-for-fact ((?wp-at wm-fact))
		(and
			(wm-key-prefix ?wp-at:key (create$ domain fact wp-at))
			(eq ?name (wm-key-arg ?wp-at:key m))
		)

		(bind ?wp-at-wp (wm-key-arg ?wp-at:key wp))
		(if ; cap_carrier_grey
			(or
				(eq ?wp-at-wp CCG1)
				(eq ?wp-at-wp CCG2)
				(eq ?wp-at-wp CCG3)
			)
		then
			(bind ?wp-at-cap "CAP_GREY_WP")
			(bind ?wp-at-base "BASE_RANDOM_WP")
		else
			(if ; cap_carrier_black
				(or
					(eq ?wp-at-wp CCB1)
					(eq ?wp-at-wp CCB2)
					(eq ?wp-at-wp CCB3)
				)
			then
				(bind ?wp-at-cap "CAP_BLACK_WP")
				(bind ?wp-at-base "BASE_RANDOM_WP")
			else
				(if ; WP1 (product) or WP2 (additional base)
					(or
						(eq ?wp-at-wp WP1)
						(eq ?wp-at-wp WP2)
					)
				then
					; wp-base-color
					(do-for-fact ((?wm-fact wm-fact))
						(and
							(wm-key-prefix ?wm-fact:key (create$ domain fact wp-base-color))
							(eq ?wp-at-wp (wm-key-arg ?wm-fact:key wp))
						)

						(bind ?wp-at-base (str-cat (wm-key-arg ?wm-fact:key col) _WP))
					)

					; wp-cap-color
					(do-for-fact ((?wm-fact wm-fact))
						(and
							(wm-key-prefix ?wm-fact:key (create$ domain fact wp-cap-color))
							(eq ?wp-at-wp (wm-key-arg ?wm-fact:key wp))
						)

						(bind ?wp-at-cap (str-cat (wm-key-arg ?wm-fact:key col) _WP))
					)

					; wp-ring-color
					(do-for-fact ((?wm-fact wm-fact))
						(and
							(wm-key-prefix ?wm-fact:key (create$ domain fact wp-ring1-color))
							(eq ?wp-at-wp (wm-key-arg ?wm-fact:key wp))
						)

						(bind ?wp-at-ring1 (str-cat (wm-key-arg ?wm-fact:key col) _WP))
					)
					(do-for-fact ((?wm-fact wm-fact))
						(and
							(wm-key-prefix ?wm-fact:key (create$ domain fact wp-ring2-color))
							(eq ?wp-at-wp (wm-key-arg ?wm-fact:key wp))
						)

						(bind ?wp-at-ring2 (str-cat (wm-key-arg ?wm-fact:key col) _WP))
					)
					(do-for-fact ((?wm-fact wm-fact))
						(and
							(wm-key-prefix ?wm-fact:key (create$ domain fact wp-ring3-color))
							(eq ?wp-at-wp (wm-key-arg ?wm-fact:key wp))
						)

						(bind ?wp-at-ring3 (str-cat (wm-key-arg ?wm-fact:key col) _WP))
					)
				else
					(printout t "No known wp in wp-at" crlf)
				)

			)
		)
	)
	(pb-set-field ?wp "cap_color" ?wp-at-cap)
	(pb-add-list ?wp "ring_colors" ?wp-at-ring1)
	(pb-add-list ?wp "ring_colors" ?wp-at-ring2)
	(pb-add-list ?wp "ring_colors" ?wp-at-ring3)
	(pb-set-field ?wp "base_color" ?wp-at-base)

	(pb-set-field ?mSMT "wp" ?wp)
	(pb-set-field ?mSMT "machine" ?m)

	(return ?mSMT)
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
(deffunction smt-create-order (?id ?complexity ?team-color)
	; (printout t "Creating Data msgs:: Order with id " ?id  crlf)
	(bind ?o (pb-create "llsf_msgs.Order"))

	(pb-set-field ?o "id" 1) ; TODO Use or ommit real ?id
	(pb-set-field ?o "delivery_gate" 1) ; TODO Use or ommit real ?gate
   (pb-set-field ?o "complexity" ?complexity)
	(pb-set-field ?o "quantity_requested" 1)
	(if (eq ?team-color CYAN)
	then
		(pb-set-field ?o "quantity_delivered_cyan" 0)
	else
		(pb-set-field ?o "quantity_delivered_magenta" 0)
	)
	(pb-set-field ?o "delivery_period_begin" 1)
	(pb-set-field ?o "delivery_period_end" 900)

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
	(if
		(or
			(eq C1 ?complexity)
			(eq C2 ?complexity)
			(eq C3 ?complexity)
		)
		then

		; order-ring1-color
		(do-for-fact ((?wm-fact wm-fact))
			(and
				(wm-key-prefix ?wm-fact:key (create$ domain fact order-ring1-color))
				(eq ?id (wm-key-arg ?wm-fact:key ord))
			)
			(bind ?rlist (append$ ?rlist (wm-key-arg ?wm-fact:key col)))
		)
	)
	(if
		(or
			(eq C2 ?complexity)
			(eq C3 ?complexity)
		)
		then

		; order-ring2-color
		(do-for-fact ((?wm-fact wm-fact))
			(and
				(wm-key-prefix ?wm-fact:key (create$ domain fact order-ring2-color))
				(eq ?id (wm-key-arg ?wm-fact:key ord))
			)
			(bind ?rlist (append$ ?rlist (wm-key-arg ?wm-fact:key col)))
		)
	)
	(if (eq C3 ?complexity)
		then

		; order-ring3-color
		(do-for-fact ((?wm-fact wm-fact))
			(and
				(wm-key-prefix ?wm-fact:key (create$ domain fact order-ring3-color))
				(eq ?id (wm-key-arg ?wm-fact:key ord))
			)
			(bind ?rlist (append$ ?rlist (wm-key-arg ?wm-fact:key col)))
		)
	)

	(foreach ?rings ?rlist
	 (pb-add-list ?o "ring_colors" ?rings)
	)

  (return ?o)
)

(deffunction smt-create-orders (?team-color ?order-id ?complexity)
	(bind ?rv (create$))
	(bind ?rv (append$ ?rv (smt-create-order ?order-id ?complexity ?team-color)))
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

; (defrule production-call-clips-smt-c0
;     ; Call plugin clips-smt
;     (goal (id ?goal-id&COMPLEXITY) (mode SELECTED))
;     (wm-fact (key refbox phase) (value PRODUCTION))
;     (wm-fact (key refbox team-color) (value ?team-color&CYAN|MAGENTA))

;     (not (complexity-c0-planned))
;     (wm-fact (key domain fact rs-ring-spec args? m ?mps r ?ring-color rn ZERO) (value TRUE))

;     ; Does an order of wanted complexity exists?
;     (wm-fact (key domain fact order-complexity args? ord ?order-id com ?complexity&C0) (value TRUE)) ; desired complexity is set here

;     ; Are all relevant details of this order available?
;     ; (wm-fact (key domain fact order-base-color args? ord ?order-id col ?base-col) (value TRUE))
;     ; (wm-fact (key domain fact order-ring1-color args? ord ?order-id col ?base-col) (value TRUE))
;     ; (wm-fact (key domain fact order-ring2-color args? ord ?order-id col ?base-col) (value TRUE))
;     ; (wm-fact (key domain fact order-ring3-color args? ord ?order-id col ?base-col) (value TRUE))
;     ; (wm-fact (key domain fact order-cap-color args? ord ?order-id col ?cap-col) (value TRUE))

;     ; Was the order already fulfilled?
;     (not (wm-fact (key domain fact order-fulfilled args? ord ?order-id) (value TRUE)))
;     ; Is the delivery-window in the future?
;     (wm-fact (key refbox game-time) (values ?sec ?sec-2))
;     (wm-fact (key refbox order ?order-id delivery-end) (value ?delivery-end&:(> ?delivery-end ?sec)))

;     ; There was no plan for this goal requested yet?
;     (not (plan-requested ?goal-id))
;     (not (plan-requested-ord ?goal-id ?order-id))

;     ; Only R-1 should run the planner
;     (wm-fact (key config rcll robot-name) (value "R-1"))
; =>
;     (printout t "SMT plan call for " ?order-id " with " ?delivery-end " " ?sec crlf)
;     (bind ?p
;       (smt-create-data
;             (smt-create-robots ?team-color)
;             (smt-create-machines ?team-color)
;             (smt-create-orders ?team-color ?order-id ?complexity)
;             (smt-create-rings ?team-color)
;             0 ; Strategy set here, 0 means MACRO and 1 WINDOW
;             5 ; Window size for strategy WINDOW
;       )
;     )

;     (smt-request "smt-plan" ?p)
;     (assert (plan-requested ?goal-id))
;     (assert (plan-requested-ord ?goal-id ?order-id))
;     (assert (plan-requested-ord ?goal-id ?order-id))
;     (assert (complexity-c0-planned))
; )


(defrule production-call-clips-smt-c3
	; Call plugin clips-smt
	(goal (id ?goal-id&COMPLEXITY) (mode SELECTED))
	(wm-fact (key refbox phase) (value PRODUCTION))
	(wm-fact (key refbox team-color) (value ?team-color&CYAN|MAGENTA))

	; (complexity-c0-planned)
	(wm-fact (key domain fact rs-ring-spec args? m ?mps r ?ring-color rn ZERO) (value TRUE))

	; Does an order of wanted complexity exists?
	(wm-fact (key domain fact order-complexity args? ord ?order-id com ?complexity&C3) (value TRUE)) ; desired complexity is set here

	; Are all relevant details of this order available?
	; (wm-fact (key domain fact order-base-color args? ord ?order-id col ?base-col) (value TRUE))
	; (wm-fact (key domain fact order-ring1-color args? ord ?order-id col ?base-col) (value TRUE))
	; (wm-fact (key domain fact order-ring2-color args? ord ?order-id col ?base-col) (value TRUE))
	; (wm-fact (key domain fact order-ring3-color args? ord ?order-id col ?base-col) (value TRUE))
	; (wm-fact (key domain fact order-cap-color args? ord ?order-id col ?cap-col) (value TRUE))

	; Was the order already fulfilled?
	(not (wm-fact (key domain fact order-fulfilled args? ord ?order-id) (value TRUE)))
	; Is the delivery-window in the future?
	(wm-fact (key refbox game-time) (values ?sec ?sec-2))
	(wm-fact (key refbox order ?order-id delivery-end) (value ?delivery-end&:(> ?delivery-end ?sec)))

	; There was no plan for this goal requested yet?
	(not (plan-requested ?goal-id))
	(not (plan-requested-ord ?goal-id ?order-id))

	; Only R-1 should run the planner
	(wm-fact (key config rcll robot-name) (value "R-1"))
=>
	(printout t "SMT plan call for " ?order-id " with " ?delivery-end " " ?sec crlf)
	(bind ?p
	  (smt-create-data
			(smt-create-robots ?team-color)
			(smt-create-machines ?team-color)
			(smt-create-orders ?team-color ?order-id ?complexity)
			(smt-create-rings ?team-color)
			0 ; Strategy set here, 0 means MACRO and 1 WINDOW
			5 ; Window size for strategy WINDOW
	  )
	)

	(smt-request "smt-plan" ?p)
	(assert (plan-requested ?goal-id))
	(assert (plan-requested-ord ?goal-id ?order-id))
	(assert (plan-requested-ord ?goal-id ?order-id))
)

;---------------------------------------------------------------------------
;  Extract plan generated by clips-smt from protobuf
;---------------------------------------------------------------------------

(defrule production-smt-plan-completed
	; Call plugin clips-smt
	?spc <- (smt-plan-complete ?handle)

	?g <- (goal (id ?goal-id) (mode SELECTED))
	?plan-req <- (plan-requested ?goal-id)
	?plan-req-ord <- (plan-requested-ord ?goal-id ?order-id)

	(wm-fact (key refbox team-color) (value ?team-color&CYAN|MAGENTA))
	=>
	(printout t "SMT plan handle completed " ?handle  crlf)

	(retract ?spc)
	; Create instance of plan
	(bind ?plan-id (string-to-field (str-cat SMT-PLAN (gensym)) ) )
	(assert
		(plan (id ?plan-id) (goal-id ?goal-id))
	)

	; Import plan generate by plugin clips-smt into ?plans and extract the plan-actions
	(bind ?plans (smt-get-plan ?handle))
	; (printout t "Plan: " (pb-tostring ?plans) crlf)

	; parent_id of 0 refers to already dependencies from the old plans
	(assert (wm-fact (key plan-action ?goal-id ?plan-id (string-to-field "0") status) (value FINAL)) )

	; Extract actions inside ActorSpecificPlan -> SequentialPlan -> Actions
	(progn$ (?ap (pb-field-list ?plans "plans"))

		(if (pb-has-field ?ap "sequential_plan") then

			(bind ?p (pb-field-value ?ap "sequential_plan"))
			(bind ?actions (pb-field-list ?p "actions"))
			(bind ?amount-plan-actions 0)


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
							(bind ?action-specific-actor (pb-field-value ?a "actor"))
						)
						(progn$ (?arg (pb-field-list ?a "params"))
							(if (eq (pb-field-value ?arg "key") "to") then
								(bind ?to-complete (pb-field-value ?arg "value"))
								(bind ?to-splitted (str-split ?to-complete "-"))
								(bind ?to (str-join "-" (subseq$ ?to-splitted 1 2)))
								(bind ?to-side (if (eq (nth$ 3 ?to-splitted) "I") then "INPUT" else "OUTPUT"))
							else
								(if (eq (pb-field-value ?arg "key") "from") then
									(bind ?from-complete (pb-field-value ?arg "value"))
									(if (eq ?from-complete "START-I") then
										(bind ?from "START")
										(bind ?from-side "INPUT")
										else
											(bind ?from-splitted (str-split ?from-complete "-"))
											(bind ?from (str-join "-" (subseq$ ?from-splitted 1 2)))
											(bind ?from-side (if (eq (nth$ 3 ?from-splitted) "I") then "INPUT" else "OUTPUT"))
									)
								else
									(printout warn "Unknown parameter " (pb-field-value ?arg "key") " for " ?actname crlf)
								)
							)
						)

						(if (not (eq ?from-complete ?to-complete)) then
							(assert
								(wm-fact
									(key plan-action ?goal-id ?plan-id ?next-step-id action)
									(is-list TRUE)
									(values "move" ?action-specific-actor ?from ?from-side ?to ?to-side)
								)
							)
							(assert
								(wm-fact
									(key plan-action ?goal-id ?plan-id ?next-step-id dep)
									(is-list TRUE)
									(values ?parents-ids)
								)
							)
							(assert
								(wm-fact
									(key plan-action ?goal-id ?plan-id ?next-step-id status)
									(value FORMULATED)
								)
							)
							(assert
								(wm-fact
									(key plan-action ?goal-id ?plan-id ?next-step-id actor)
									(value ?action-specific-actor)
								)
							)
							(bind ?amount-plan-actions (+ ?amount-plan-actions 1))

							(printout t "plan-action move added: " ?action-specific-actor " [" ?action-id  "] from: " ?from " at: " ?from-side " to: " ?to " at: " ?to-side crlf)

						else
							(assert (wm-fact (key plan-action ?goal-id ?plan-id ?next-step-id status) (value "FINAL")) )
							(printout t "plan-action move added: " ?action-specific-actor " [" ?action-id  "] is not necessary and marked as final for parent dependencies"  crlf)
						)
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
							(bind ?action-specific-actor (pb-field-value ?a "actor"))
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
									(if (eq (pb-field-value ?arg "key") "wp") then
										(bind ?wp (pb-field-value ?arg "value"))

									else
										(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
									)
								)
							)
						)

						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id action)
								(is-list TRUE)
								(values "wp-get-shelf" ?action-specific-actor ?wp ?mps ?shelf)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id dep)
								(is-list TRUE)
								(values ?parents-ids)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id status)
								(value FORMULATED)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id actor)
								(value ?action-specific-actor)
							)
						)
						(bind ?amount-plan-actions (+ ?amount-plan-actions 1))

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
							(bind ?action-specific-actor (pb-field-value ?a "actor"))
						)
						(progn$ (?arg (pb-field-list ?a "params"))
							(if (eq (pb-field-value ?arg "key") "mps") then
								(bind ?mps (pb-field-value ?arg "value"))
								(bind ?mps-splitted (str-split ?mps "-"))
								(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
								(bind ?side (if (eq (nth$ 3 ?mps-splitted) "I") then INPUT else OUTPUT))
							else
								(if (eq (pb-field-value ?arg "key") "wp") then
									(bind ?wp (pb-field-value ?arg "value"))

								else
									(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
								)
							)
						)

						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id action)
								(is-list TRUE)
								(values "wp-get" ?action-specific-actor ?wp ?mps ?side)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id dep)
								(is-list TRUE)
								(values ?parents-ids)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id status)
								(value FORMULATED)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id actor)
								(value ?action-specific-actor)
							)
						)
						(bind ?amount-plan-actions (+ ?amount-plan-actions 1))

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
							(bind ?action-specific-actor (pb-field-value ?a "actor"))
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
									(if (eq (pb-field-value ?arg "key") "wp") then
										(bind ?wp (pb-field-value ?arg "value"))

									else
										(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
									)
								)
							)
						)

						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id action)
								(is-list TRUE)
								(values "wp-put" ?action-specific-actor ?wp ?mps)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id dep)
								(is-list TRUE)
								(values ?parents-ids)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id status)
								(value FORMULATED)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id actor)
								(value ?action-specific-actor)
							)
						)
						(bind ?amount-plan-actions (+ ?amount-plan-actions 1))

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
							(bind ?action-specific-actor (pb-field-value ?a "actor"))
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
									(if (eq (pb-field-value ?arg "key") "rs-before") then
										(bind ?rs-before (pb-field-value ?arg "value"))

									else
										(if (eq (pb-field-value ?arg "key") "rs-after") then
											(bind ?rs-after (pb-field-value ?arg "value"))

										else
											(if (eq (pb-field-value ?arg "key") "wp") then
												 (bind ?wp (pb-field-value ?arg "value"))

											else
												(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
											)
										)
									)
								)
							)
						)

						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id action)
								(is-list TRUE)
								(values "wp-put-slide-cc" ?action-specific-actor ?wp ?mps ?rs-before ?rs-after)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id dep)
								(is-list TRUE)
								(values ?parents-ids)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id status)
								(value FORMULATED)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id actor)
								(value ?action-specific-actor)
							)
						)
						(bind ?amount-plan-actions (+ ?amount-plan-actions 1))

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
							(bind ?action-specific-actor (pb-field-value ?a "actor"))
						)
						(progn$ (?arg (pb-field-list ?a "params"))
							(if (eq (pb-field-value ?arg "key") "wp") then
								(bind ?wp (pb-field-value ?arg "value"))

							else
								(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
							)
						)

						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id action)
								(is-list TRUE)
								(values "wp-discard" ?action-specific-actor ?wp)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id dep)
								(is-list TRUE)
								(values ?parents-ids)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id status)
								(value FORMULATED)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id actor)
								(value ?action-specific-actor)
							)
						)
						(bind ?amount-plan-actions (+ ?amount-plan-actions 1))

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
							(bind ?action-specific-actor (pb-field-value ?a "actor"))
						)
						(progn$ (?arg (pb-field-list ?a "params"))
							(if (eq (pb-field-value ?arg "key") "mps") then
								(bind ?mps (pb-field-value ?arg "value"))
								(bind ?mps-splitted (str-split ?mps "-"))
								(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
								(bind ?side (if (eq (nth$ 3 ?mps-splitted) "I") then INPUT else OUTPUT))
							else
								(if (eq (pb-field-value ?arg "key") "color") then
									(bind ?goal-base-color (pb-field-value ?arg "value"))

								else
									(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
								)
							)
						)

						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id action)
								(is-list TRUE)
								(values "prepare-bs" ?mps ?side ?goal-base-color)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id dep)
								(is-list TRUE)
								(values ?parents-ids)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id status)
								(value FORMULATED)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id actor)
								(value ?action-specific-actor)
							)
						)
						(bind ?amount-plan-actions (+ ?amount-plan-actions 1))

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
							(bind ?action-specific-actor (pb-field-value ?a "actor"))
						)
						(progn$ (?arg (pb-field-list ?a "params"))
							(if (eq (pb-field-value ?arg "key") "mps") then
								(bind ?mps (pb-field-value ?arg "value"))
								(bind ?mps-splitted (str-split ?mps "-"))
								(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
								(bind ?side (if (eq (nth$ 3 ?mps-splitted) "I") then INPUT else OUTPUT))
							else
								(if (eq (pb-field-value ?arg "key") "color") then
									(bind ?goal-base-color (pb-field-value ?arg "value"))

								else
									(if (eq (pb-field-value ?arg "key") "wp") then
										(bind ?wp (pb-field-value ?arg "value"))

									else
										(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
									)
								)
							)
						)

						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id action)
								(is-list TRUE)
								(values "bs-dispense" ?action-specific-actor ?mps ?side ?wp ?goal-base-color)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id dep)
								(is-list TRUE)
								(values ?parents-ids)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id status)
								(value FORMULATED)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id actor)
								(value ?action-specific-actor)
							)
						)
						(bind ?amount-plan-actions (+ ?amount-plan-actions 1))

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
							(bind ?action-specific-actor (pb-field-value ?a "actor"))
						)
						(progn$ (?arg (pb-field-list ?a "params"))
							(if (eq (pb-field-value ?arg "key") "mps") then
								(bind ?mps (pb-field-value ?arg "value"))
								(bind ?mps-splitted (str-split ?mps "-"))
								(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))

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
								(wm-fact
									(key plan-action ?goal-id ?plan-id ?next-step-id action)
									(is-list TRUE)
									(values "prepare-ds" ?mps (wm-key-arg ?wm-fact:key gate))
								)
							)
							(assert
								(wm-fact
									(key plan-action ?goal-id ?plan-id ?next-step-id dep)
									(is-list TRUE)
									(values ?parents-ids)
								)
							)
							(assert
								(wm-fact
									(key plan-action ?goal-id ?plan-id ?next-step-id status)
									(value FORMULATED)
								)
							)
							(assert
								(wm-fact
									(key plan-action ?goal-id ?plan-id ?next-step-id actor)
									(value ?action-specific-actor)
								)
							)
						)
						(bind ?amount-plan-actions (+ ?amount-plan-actions 1))

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

							else
								(if (eq (pb-field-value ?arg "key") "base-color") then
									(bind ?base-color (pb-field-value ?arg "value"))

								else
									(if (eq (pb-field-value ?arg "key") "cap-color") then
										(bind ?cap-color (pb-field-value ?arg "value"))

									else
										(if (eq (pb-field-value ?arg "key") "wp") then
											(bind ?wp (pb-field-value ?arg "value"))

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
								(wm-fact
									(key plan-action ?goal-id ?plan-id ?next-step-id action)
									(is-list TRUE)
									(values "fulfill-order-c0" ?order-id ?wp ?mps (wm-key-arg ?wm-fact:key gate) ?base-color ?cap-color)
								)
							)
							(assert
								(wm-fact
									(key plan-action ?goal-id ?plan-id ?next-step-id dep)
									(is-list TRUE)
									(values ?parents-ids)
								)
							)
							(assert
								(wm-fact
									(key plan-action ?goal-id ?plan-id ?next-step-id status)
									(value FORMULATED)
								)
							)
							(assert
								(wm-fact
									(key plan-action ?goal-id ?plan-id ?next-step-id actor)
									(value ?action-specific-actor)
								)
							)
						)
						(bind ?amount-plan-actions (+ ?amount-plan-actions 1))

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
							else
								(if (eq (pb-field-value ?arg "key") "base-color") then
									(bind ?base-color (pb-field-value ?arg "value"))

								else
									(if (eq (pb-field-value ?arg "key") "ring1-color") then
										(bind ?ring1-color (pb-field-value ?arg "value"))

									else
										(if (eq (pb-field-value ?arg "key") "cap-color") then
											(bind ?cap-color (pb-field-value ?arg "value"))

										else
											(if (eq (pb-field-value ?arg "key") "wp") then
												(bind ?wp (pb-field-value ?arg "value"))

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
								(wm-fact
									(key plan-action ?goal-id ?plan-id ?next-step-id action)
									(is-list TRUE)
									(values "fulfill-order-c1" ?order-id ?wp ?mps (wm-key-arg ?wm-fact:key gate) ?base-color ?cap-color ?ring1-color)
								)
							)
							(assert
								(wm-fact
									(key plan-action ?goal-id ?plan-id ?next-step-id dep)
									(is-list TRUE)
									(values ?parents-ids)
								)
							)
							(assert
								(wm-fact
									(key plan-action ?goal-id ?plan-id ?next-step-id status)
									(value FORMULATED)
								)
							)
							(assert
								(wm-fact
									(key plan-action ?goal-id ?plan-id ?next-step-id actor)
									(value ?action-specific-actor)
								)
							)
						)
						(bind ?amount-plan-actions (+ ?amount-plan-actions 1))

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
							else
								(if (eq (pb-field-value ?arg "key") "base-color") then
									(bind ?base-color (pb-field-value ?arg "value"))

								else
									(if (eq (pb-field-value ?arg "key") "ring1-color") then
										(bind ?ring1-color (pb-field-value ?arg "value"))

									else
										(if (eq (pb-field-value ?arg "key") "ring2-color") then
											(bind ?ring2-color (pb-field-value ?arg "value"))

										else
											(if (eq (pb-field-value ?arg "key") "cap-color") then
												(bind ?cap-color (pb-field-value ?arg "value"))

											else
												(if (eq (pb-field-value ?arg "key") "wp") then
													(bind ?wp (pb-field-value ?arg "value"))

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
								(wm-fact
									(key plan-action ?goal-id ?plan-id ?next-step-id action)
									(is-list TRUE)
									(values "fulfill-order-c2" ?order-id ?wp ?mps (wm-key-arg ?wm-fact:key gate) ?base-color ?cap-color ?ring1-color ?ring2-color)
								)
							)
							(assert
								(wm-fact
									(key plan-action ?goal-id ?plan-id ?next-step-id dep)
									(is-list TRUE)
									(values ?parents-ids)
								)
							)
							(assert
								(wm-fact
									(key plan-action ?goal-id ?plan-id ?next-step-id status)
									(value FORMULATED)
								)
							)
							(assert
								(wm-fact
									(key plan-action ?goal-id ?plan-id ?next-step-id actor)
									(value ?action-specific-actor)
								)
							)
						)
						(bind ?amount-plan-actions (+ ?amount-plan-actions 1))

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

							 else
								(if (eq (pb-field-value ?arg "key") "base-color") then
								  (bind ?base-color (pb-field-value ?arg "value"))

								else
									(if (eq (pb-field-value ?arg "key") "ring1-color") then
									  (bind ?ring1-color (pb-field-value ?arg "value"))

									else
										(if (eq (pb-field-value ?arg "key") "ring2-color") then
										  (bind ?ring2-color (pb-field-value ?arg "value"))

										else
											(if (eq (pb-field-value ?arg "key") "ring3-color") then
											  (bind ?ring3-color (pb-field-value ?arg "value"))

											else
												(if (eq (pb-field-value ?arg "key") "cap-color") then
												  (bind ?cap-color (pb-field-value ?arg "value"))

												else
													(if (eq (pb-field-value ?arg "key") "wp") then
													  (bind ?wp (pb-field-value ?arg "value"))

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
								(wm-fact
									(key plan-action ?goal-id ?plan-id ?next-step-id action)
									(is-list TRUE)
									(values "fulfill-order-c3" ?order-id ?wp ?mps (wm-key-arg ?wm-fact:key gate) ?base-color ?cap-color ?ring1-color ?ring2-color ?ring3-color)
								)
							)
							(assert
								(wm-fact
									(key plan-action ?goal-id ?plan-id ?next-step-id dep)
									(is-list TRUE)
									(values ?parents-ids)
								)
							)
							(assert
								(wm-fact
									(key plan-action ?goal-id ?plan-id ?next-step-id status)
									(value FORMULATED)
								)
							)
							(assert
								(wm-fact
									(key plan-action ?goal-id ?plan-id ?next-step-id actor)
									(value ?action-specific-actor)
								)
							)
						)
						(bind ?amount-plan-actions (+ ?amount-plan-actions 1))

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
							(bind ?action-specific-actor (pb-field-value ?a "actor"))
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

						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id action)
								(is-list TRUE)
								(values "prepare-cs" ?mps ?operation)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id dep)
								(is-list TRUE)
								(values ?parents-ids)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id status)
								(value FORMULATED)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id actor)
								(value ?action-specific-actor)
							)
						)
						(bind ?amount-plan-actions (+ ?amount-plan-actions 1))

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
							else
								(if (eq (pb-field-value ?arg "key") "cap-color") then
									(bind ?cap-color (pb-field-value ?arg "value"))

								else
									(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
								)
							)
						)

						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id action)
								(is-list TRUE)
								(values "cs-retrieve-cap" ?mps ?wp ?cap-color)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id dep)
								(is-list TRUE)
								(values ?parents-ids)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id status)
								(value FORMULATED)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id actor)
								(value ?action-specific-actor)
							)
						)
						(bind ?amount-plan-actions (+ ?amount-plan-actions 1))

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
							else
								(if (eq (pb-field-value ?arg "key") "cap-color") then
									(bind ?cap-color (pb-field-value ?arg "value"))

								else
									(if (eq (pb-field-value ?arg "key") "wp") then
									  (bind ?wp (pb-field-value ?arg "value"))

									else
										(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
									)
								)
							)
						)

						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id action)
								(is-list TRUE)
								(values "cs-mount-cap" ?mps ?wp ?cap-color)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id dep)
								(is-list TRUE)
								(values ?parents-ids)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id status)
								(value FORMULATED)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id actor)
								(value ?action-specific-actor)
							)
						)
						(bind ?amount-plan-actions (+ ?amount-plan-actions 1))

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
							(bind ?action-specific-actor (pb-field-value ?a "actor"))
						)
						(progn$ (?arg (pb-field-list ?a "params"))
							(if (eq (pb-field-value ?arg "key") "mps") then
								(bind ?mps (pb-field-value ?arg "value"))
								(bind ?mps-splitted (str-split ?mps "-"))
								(bind ?mps (str-join "-" (subseq$ ?mps-splitted 1 2)))
							else
								(if (eq (pb-field-value ?arg "key") "ring_color") then
										(bind ?goal-ring-color (pb-field-value ?arg "value"))  ;temp: the color of the base of the goal is recognized here

								else
									(if (eq (pb-field-value ?arg "key") "rs-before") then
									  (bind ?rs-before (pb-field-value ?arg "value"))

									else
										(if (eq (pb-field-value ?arg "key") "rs-after") then
										  (bind ?rs-after (pb-field-value ?arg "value"))

										else
											(if (eq (pb-field-value ?arg "key") "r-req") then
											  (bind ?r-req (pb-field-value ?arg "value"))

											else
												(printout warn "Unknown parameter " (pb-field-value ?arg "key") crlf)
											)
										)
									)
								)
							)
						)

						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id action)
								(is-list TRUE)
								(values "prepare-rs" ?mps ?goal-ring-color ?rs-before ?rs-after ?r-req)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id dep)
								(is-list TRUE)
								(values ?parents-ids)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id status)
								(value FORMULATED)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id actor)
								(value ?action-specific-actor)
							)
						)
						(bind ?amount-plan-actions (+ ?amount-plan-actions 1))

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
							else
								(if (eq (pb-field-value ?arg "key") "ring-color") then
									(bind ?ring-color (pb-field-value ?arg "value"))

								else
									(if (eq (pb-field-value ?arg "key") "rs-before") then
									  (bind ?rs-before (pb-field-value ?arg "value"))

									else
										(if (eq (pb-field-value ?arg "key") "rs-after") then
										  (bind ?rs-after (pb-field-value ?arg "value"))

										else
											(if (eq (pb-field-value ?arg "key") "r-req") then
											  (bind ?r-req (pb-field-value ?arg "value"))

											else
												(if (eq (pb-field-value ?arg "key") "wp") then
												  (bind ?wp (pb-field-value ?arg "value"))

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
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id action)
								(is-list TRUE)
								(values "rs-mount-ring1" ?mps ?wp ?ring-color ?rs-before ?rs-after ?r-req)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id dep)
								(is-list TRUE)
								(values ?parents-ids)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id status)
								(value FORMULATED)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id actor)
								(value ?action-specific-actor)
							)
						)
						(bind ?amount-plan-actions (+ ?amount-plan-actions 1))

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
							else
								(if (eq (pb-field-value ?arg "key") "ring-color") then
								  (bind ?ring-color (pb-field-value ?arg "value"))

								else
									(if (eq (pb-field-value ?arg "key") "rs-before") then
										(bind ?rs-before (pb-field-value ?arg "value"))

									else
										(if (eq (pb-field-value ?arg "key") "rs-after") then
											(bind ?rs-after (pb-field-value ?arg "value"))

										else
											(if (eq (pb-field-value ?arg "key") "r-req") then
												(bind ?r-req (pb-field-value ?arg "value"))

											else
												(if (eq (pb-field-value ?arg "key") "col1") then
													(bind ?col1 (pb-field-value ?arg "value"))

												else
													(if (eq (pb-field-value ?arg "key") "wp") then
														(bind ?wp (pb-field-value ?arg "value"))

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
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id action)
								(is-list TRUE)
								(values "rs-mount-ring2" ?mps ?wp ?ring-color ?col1 ?rs-before ?rs-after ?r-req)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id dep)
								(is-list TRUE)
								(values ?parents-ids)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id status)
								(value FORMULATED)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id actor)
								(value ?action-specific-actor)
							)
						)
						(bind ?amount-plan-actions (+ ?amount-plan-actions 1))

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
							else
								(if (eq (pb-field-value ?arg "key") "ring-color") then
								  (bind ?ring-color (pb-field-value ?arg "value"))

								else
									(if (eq (pb-field-value ?arg "key") "rs-before") then
										(bind ?rs-before (pb-field-value ?arg "value"))

									else
										(if (eq (pb-field-value ?arg "key") "rs-after") then
											(bind ?rs-after (pb-field-value ?arg "value"))

										else
											(if (eq (pb-field-value ?arg "key") "r-req") then
												(bind ?r-req (pb-field-value ?arg "value"))

											else
												(if (eq (pb-field-value ?arg "key") "col1") then
													(bind ?col1 (pb-field-value ?arg "value"))

												else
													(if (eq (pb-field-value ?arg "key") "col2") then
														(bind ?col2 (pb-field-value ?arg "value"))

													else
														(if (eq (pb-field-value ?arg "key") "wp") then
															(bind ?wp (pb-field-value ?arg "value"))

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
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id action)
								(is-list TRUE)
								(values "rs-mount-ring3" ?mps ?wp ?ring-color ?col1 ?col2 ?rs-before ?rs-after ?r-req)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id dep)
								(is-list TRUE)
								(values ?parents-ids)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id status)
								(value FORMULATED)
							)
						)
						(assert
							(wm-fact
								(key plan-action ?goal-id ?plan-id ?next-step-id actor)
								(value ?action-specific-actor)
							)
						)
						(bind ?amount-plan-actions (+ ?amount-plan-actions 1))

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
	(printout t "Amount of plan-actions added: " ?amount-plan-actions crlf)
	(assert (wm-fact (key plan-action ?goal-id ?plan-id amount-plan-actions) (value ?amount-plan-actions)))
	(assert (wm-fact (key plan-action ?goal-id ?plan-id amount-plan-actions-collected-r-2) (value 1)))
	(assert (wm-fact (key plan-action ?goal-id ?plan-id amount-plan-actions-collected-r-3) (value 1)))
	(assert (wm-fact (key plan-action ?goal-id ?plan-id order) (value ?order-id)))
	(pb-destroy ?plans)
	(modify ?g (mode EXPANDED))
	(retract ?plan-req)
	(retract ?plan-req-ord)
)

;---------------------------------------------------------------------------
;  Add plan action out of wm-facts plan after wm-fact sync
;  TODO
;    - Add filter for correct robot (compare ?action-specific-actor with own name) in order to add only own actions
;    - Add filter for R-1 for exogenous actions
;---------------------------------------------------------------------------

(defrule production-add-plan-action-enter-field
	; If wm-fact plan enter-field is found assert the corresponding plan-action enter-field
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "enter-field" ?robot-name&:(eq ?robot-name (cx-identity)) ?team-color)
	)
	=>
	(printout t "Enter-field action translated from wm-fact-base by " (cx-identity) " and " ?robot-name crlf)
	(assert
		 (plan-action
			(id ?next-step-id)
			(plan-id ?plan-id)
			(goal-id ?goal-id)
                        (duration 4.0)
			(action-name enter-field)
			(param-names r team-color)
			(param-values (string-to-field ?robot-name) (string-to-field ?team-color)))
	)
)

(defrule production-add-plan-action-move
	; If wm-fact plan move is found assert the corresponding plan-action move
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "move" ?robot-name&:(eq ?robot-name (cx-identity)) ?from ?from-side ?to ?to-side)
	)
	=>
	(printout t "Move action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name move)
			(param-names r from from-side to to-side)
			(param-values (string-to-field ?robot-name) (string-to-field ?from) (string-to-field ?from-side) (string-to-field ?to) (string-to-field ?to-side))
		)
	)
)

(defrule production-add-plan-action-wp-get-shelf
	; If wm-fact plan wp-get-shelf is found assert the corresponding plan-action wp-get-shelf
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "wp-get-shelf" ?robot-name&:(eq ?robot-name (cx-identity)) ?wp ?mps ?shelf)
	)
	=>
	(printout t "Wp-get-shelf action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name wp-get-shelf)
			(param-names r cc m spot)
			(param-values (string-to-field ?robot-name) (string-to-field ?wp) (string-to-field ?mps) (string-to-field ?shelf))
		)
	)
)

(defrule production-add-plan-action-wp-get
	; If wm-fact plan wp-get is found assert the corresponding plan-action wp-get
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "wp-get" ?robot-name&:(eq ?robot-name (cx-identity)) ?wp ?mps ?side)
	)
	=>
	(printout t "Wp-get action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name wp-get)
			(param-names r wp m side)
			(param-values (string-to-field ?robot-name) (string-to-field ?wp) (string-to-field ?mps) (string-to-field ?side))
		)
	)
)

(defrule production-add-plan-action-wp-put
	; If wm-fact plan wp-put is found assert the corresponding plan-action wp-put
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "wp-put" ?robot-name&:(eq ?robot-name (cx-identity)) ?wp ?mps)
	)
	=>
	(printout t "Wp-put action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name wp-put)
			(param-names r wp m)
			(param-values (string-to-field ?robot-name) (string-to-field ?wp) (string-to-field ?mps))
		)
	)
)

(defrule production-add-plan-action-wp-put-slide-cc
	; If wm-fact plan wp-put-slide-cc is found assert the corresponding plan-action wp-put-slide-cc
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "wp-put-slide-cc" ?robot-name&:(eq ?robot-name (cx-identity)) ?wp ?mps ?rs-before ?rs-after)
	)
	=>
	(printout t "Wp-put-slide-cc action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name wp-put-slide-cc)
			(param-names r wp m rs-before rs-after)
			(param-values (string-to-field ?robot-name) (string-to-field ?wp) (string-to-field ?mps) (string-to-field ?rs-before) (string-to-field ?rs-after))
		)
	)
)

(defrule production-add-plan-action-wp-discard
	; If wm-fact plan wp-discard is found assert the corresponding plan-action wp-discard
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "wp-discard" ?robot-name&:(eq ?robot-name (cx-identity)) ?wp)
	)
	=>
	(printout t "Wp-discard action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name wp-discard)
			(param-names r cc)
			(param-values (string-to-field ?robot-name) (string-to-field ?wp))
		)
	)
)

(defrule production-add-plan-action-prepare-bs
	; If wm-fact plan prepare-bs is found assert the corresponding plan-action prepare-bs
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id actor)
		(value ?robot-name&:(eq ?robot-name (cx-identity)))
	)
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "prepare-bs" ?mps ?side ?goal-base-color)
	)
	=>
	(assert
	(printout t "Prepare-bs action translated from wm-fact-base by " (cx-identity) crlf)
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name prepare-bs)
			(param-names m side bc)
			(param-values (string-to-field ?mps) (string-to-field ?side) (string-to-field ?goal-base-color))
		)
	)
)

(defrule production-add-plan-action-bs-dispense
	; If wm-fact plan bs-dispense is found assert the corresponding plan-action bs-dispense
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "bs-dispense" ?robot-name&:(eq ?robot-name (cx-identity)) ?mps ?side ?wp ?goal-base-color)
	)
	=>
	(printout t "Bs-dispense action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name bs-dispense)
			(param-names r m side wp basecol)
			(param-values (string-to-field ?robot-name) (string-to-field ?mps) (string-to-field ?side) (string-to-field ?wp) (string-to-field ?goal-base-color))
		)
	)
)

(defrule production-add-plan-action-prepare-ds
	; If wm-fact plan prepare-ds is found assert the corresponding plan-action prepare-ds
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id actor)
		(value ?robot-name&:(eq ?robot-name (cx-identity)))
	)
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "prepare-ds" ?mps ?gate)
	)
	=>
	(printout t "Prepare-ds action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name prepare-ds)
			(param-names m gate)
			(param-values (string-to-field ?mps) (string-to-field ?gate))
		)
	)
)

(defrule production-add-plan-action-fulfill-order-c0
	; If wm-fact plan fulfill-order-c0 is found assert the corresponding plan-action fulfill-order-c0
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id actor)
		(value ?robot-name&:(eq ?robot-name (cx-identity)))
	)
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "fulfill-order-c0" ?order-id ?wp ?mps ?gate ?base-color ?cap-color)
	)
	=>
	(printout t "Fulfill-c0 action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name fulfill-order-c0)
			(param-names ord wp m g basecol capcol)
			(param-values (string-to-field ?order-id) (string-to-field ?wp) (string-to-field ?mps) (string-to-field ?gate) (string-to-field ?base-color) (string-to-field ?cap-color))
		)
	)
)

(defrule production-add-plan-action-fulfill-order-c1
	; If wm-fact plan fulfill-order-c1 is found assert the corresponding plan-action fulfill-order-c1
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id actor)
		(value ?robot-name&:(eq ?robot-name (cx-identity)))
	)
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "fulfill-order-c1" ?order-id ?wp ?mps ?gate ?base-color ?cap-color ?ring1-color)
	)
	=>
	(printout t "Fulfill-c1 action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name fulfill-order-c1)
			(param-names ord wp m g basecol capcol ring1col)
			(param-values (string-to-field ?order-id) (string-to-field ?wp) (string-to-field ?mps) (string-to-field ?gate) (string-to-field ?base-color) (string-to-field ?cap-color) (string-to-field ?ring1-color))
		)
	)
)

(defrule production-add-plan-action-fulfill-order-c2
	; If wm-fact plan fulfill-order-c2 is found assert the corresponding plan-action fulfill-order-c2
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id actor)
		(value ?robot-name&:(eq ?robot-name (cx-identity)))
	)
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "fulfill-order-c2" ?order-id ?wp ?mps ?gate ?base-color ?cap-color ?ring1-color ?ring2-color)
	)
	=>
	(printout t "Fulfill-c2 action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name fulfill-order-c2)
			(param-names ord wp m g basecol capcol ring1col ring2col)
			(param-values (string-to-field ?order-id) (string-to-field ?wp) (string-to-field ?mps) (string-to-field ?gate) (string-to-field ?base-color) (string-to-field ?cap-color) (string-to-field ?ring1-color) (string-to-field ?ring2-color))
		)
	)
)

(defrule production-add-plan-action-fulfill-order-c3
	; If wm-fact plan fulfill-order-c3 is found assert the corresponding plan-action fulfill-order-c3
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id actor)
		(value ?robot-name&:(eq ?robot-name (cx-identity)))
	)
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "fulfill-order-c3" ?order-id ?wp ?mps ?gate ?base-color ?cap-color ?ring1-color ?ring2-color ?ring3-color)
	)
	=>
	(printout t "Fulfill-c3 action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name fulfill-order-c3)
			(param-names ord wp m g basecol capcol ring1col ring2col ring3col)
			(param-values (string-to-field ?order-id) (string-to-field ?wp) (string-to-field ?mps) (string-to-field ?gate) (string-to-field ?base-color) (string-to-field ?cap-color) (string-to-field ?ring1-color) (string-to-field ?ring2-color) (string-to-field ?ring3-color))
		)
	)
)

(defrule production-add-plan-action-prepare-cs
	; If wm-fact plan prepare-cs is found assert the corresponding plan-action prepare-cs
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id actor)
		(value ?robot-name&:(eq ?robot-name (cx-identity)))
	)
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "prepare-cs" ?mps ?operation)
	)
	=>
	(printout t "Prepare-cs action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name prepare-cs)
			(param-names m op)
			(param-values (string-to-field ?mps) (string-to-field ?operation))
		)
	)
)

(defrule production-add-plan-action-cs-retrieve-cap
	; If wm-fact plan cs-retrieve-cap is found assert the corresponding plan-action cs-retrieve-cap
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id actor)
		(value ?robot-name&:(eq ?robot-name (cx-identity)))
	)
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "cs-retrieve-cap" ?mps ?wp ?cap-color)
	)
	=>
	(printout t "Cs-retrieve-cap action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name cs-retrieve-cap)
			(param-names m cc capcol)
			(param-values (string-to-field ?mps) (string-to-field ?wp) (string-to-field ?cap-color))
		)
	)
)

(defrule production-add-plan-action-cs-mount-cap
	; If wm-fact plan cs-mount-cap is found assert the corresponding plan-action cs-mount-cap
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id actor)
		(value ?robot-name&:(eq ?robot-name (cx-identity)))
	)
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "cs-mount-cap" ?mps ?wp ?cap-color)
	)
	=>
	(printout t "Cs-mount-cap action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name cs-mount-cap)
			(param-names m wp capcol)
			(param-values (string-to-field ?mps) (string-to-field ?wp) (string-to-field ?cap-color))
		)
	)
)

(defrule production-add-plan-action-prepare-rs
	; If wm-fact plan prepare-rs is found assert the corresponding plan-action prepare-rs
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id actor)
		(value ?robot-name&:(eq ?robot-name (cx-identity)))
	)
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "prepare-rs" ?mps ?goal-ring-color ?rs-before ?rs-after ?r-req)
	)
	=>
	(printout t "Prepare-rs action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name prepare-rs)
			(param-names m rc rs-before rs-after r-req)
			(param-values (string-to-field ?mps) (string-to-field ?goal-ring-color) (string-to-field ?rs-before) (string-to-field ?rs-after) (string-to-field ?r-req))
		)
	)
)

(defrule production-add-plan-action-rs-mount-ring1
	; If wm-fact plan rs-mount-ring1 is found assert the corresponding plan-action rs-mount-ring1
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id actor)
		(value ?robot-name&:(eq ?robot-name (cx-identity)))
	)
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "rs-mount-ring1" ?mps ?wp ?ring-color ?rs-before ?rs-after ?r-req)
	)
	=>
	(printout t "Rs-mount-ring1 action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name rs-mount-ring1)
			(param-names m wp col rs-before rs-after r-req)
			(param-values (string-to-field ?mps) (string-to-field ?wp) (string-to-field ?ring-color) (string-to-field ?rs-before) (string-to-field ?rs-after) (string-to-field ?r-req))
		)
	)
)

(defrule production-add-plan-action-rs-mount-ring2
	; If wm-fact plan rs-mount-ring2 is found assert the corresponding plan-action rs-mount-ring2
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id actor)
		(value ?robot-name&:(eq ?robot-name (cx-identity)))
	)
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "rs-mount-ring2" ?mps ?wp ?ring-color ?col1 ?rs-before ?rs-after ?r-req)
	)
	=>
	(printout t "Rs-mount-ring2 action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name rs-mount-ring2)
			(param-names m wp col col1 rs-before rs-after r-req)
			(param-values (string-to-field ?mps) (string-to-field ?wp) (string-to-field ?ring-color) (string-to-field ?col1) (string-to-field ?rs-before) (string-to-field ?rs-after) (string-to-field ?r-req))
		)
	)
)

(defrule production-add-plan-action-rs-mount-ring3
	; If wm-fact plan rs-mount-ring3 is found assert the corresponding plan-action rs-mount-ring3
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id actor)
		(value ?robot-name&:(eq ?robot-name (cx-identity)))
	)
	(wm-fact
		(key plan-action ?goal-id ?plan-id ?next-step-id action)
		(values "rs-mount-ring3" ?mps ?wp ?ring-color ?col1 ?col2 ?rs-before ?rs-after ?r-req)
	)
	=>
	(printout t "Rs-mount-ring3 action translated from wm-fact-base by " (cx-identity) crlf)
	(assert
		(plan-action
			(id (string-to-field (str-cat ?next-step-id)))
			(plan-id ?plan-id)
			(goal-id ?goal-id)
			(duration 4.0)
			(action-name rs-mount-ring3)
			(param-names m wp col col1 col2 rs-before rs-after r-req)
			(param-values (string-to-field ?mps) (string-to-field ?wp) (string-to-field ?ring-color) (string-to-field ?col1) (string-to-field ?col2) (string-to-field ?rs-before) (string-to-field ?rs-after) (string-to-field ?r-req))
		)
	)
)

;---------------------------------------------------------------------------
; For every wm-fact plan-action ... action added append the id to a wm-fact plan-action ... amount-plan-actions-collected
; if the amount of actions collected corresponds to the amount of overall plan-actions any robot (R-2 R-3) is ready for expansion
;---------------------------------------------------------------------------

(defrule production-add-plan-actions-collected-r-2
	(wm-fact (key config rcll robot-name) (value "R-2"))
	(wm-fact (key plan-action ?goal-id ?plan-id  amount-plan-actions) (value ?amount-plan-actions))
	?apac <- (wm-fact (key plan-action ?goal-id ?plan-id amount-plan-actions-collected-r-2) (value ?amount-plan-actions-collected))
	(wm-fact (key plan-action ?goal-id ?plan-id ?id action))
	(not (wm-fact (key plan-action ?goal-id ?plan-id ?id action-added-r-2)) )
	(not (wm-fact (key plan-action ?goal-id ?plan-id expanded-r-2)) )
=>
	(assert (wm-fact (key plan-action ?goal-id ?plan-id ?id action-added-r-2)) )
	(modify ?apac (value (+ ?amount-plan-actions-collected 1)) )
	(printout t "Collect action " ?id " for R-2" crlf)
	(printout t "Amoutn of collected actions is " ?amount-plan-actions-collected " for R-2 /" ?amount-plan-actions crlf)
)

(defrule production-add-plan-actions-collected-r-3
	(wm-fact (key config rcll robot-name) (value "R-3"))
	(wm-fact (key plan-action ?goal-id ?plan-id  amount-plan-actions) (value ?amount-plan-actions))
	?apac <- (wm-fact (key plan-action ?goal-id ?plan-id amount-plan-actions-collected-r-3) (value ?amount-plan-actions-collected))
	(wm-fact (key plan-action ?goal-id ?plan-id ?id action))
	(not (wm-fact (key plan-action ?goal-id ?plan-id ?id action-added-r-3)) )
	(not (wm-fact (key plan-action ?goal-id ?plan-id expanded-r-3)) )
=>
	(printout t "Collect action " ?id " for R-3" crlf)
	(assert (wm-fact (key plan-action ?goal-id ?plan-id ?id action-added-r-3)) )
	(modify ?apac (value (+ ?amount-plan-actions-collected 1)) )
	(printout t "Amoutn of collected actions is " ?amount-plan-actions-collected " for R-3 /" ?amount-plan-actions crlf)
)

(defrule production-no-call-clips-smt-r-2
	?g <- (goal (id ?goal-id&COMPLEXITY) (mode SELECTED))
	(not (wm-fact (key plan-action ?goal-id ?plan-id expanded-r-2)) )

	; R-2 should only mark the goal as expanded
	(wm-fact (key config rcll robot-name) (value "R-2"))
	(wm-fact (key plan-action ?goal-id ?plan-id amount-plan-actions-collected-r-2) (value ?amount-plan-actions-collected))
	(wm-fact (key plan-action ?goal-id ?plan-id amount-plan-actions) (value ?amount-plan-actions))
	; (test (eq (string-to-field (str-cat (+ (length$ ?amount-plan-actions-collected) 1))) (string-to-field (str-cat ?amount-plan-actions)) ))
	(test (= ?amount-plan-actions ?amount-plan-actions-collected))
=>
	(printout t "Expand for R-2 with " ?amount-plan-actions " and collected " ?amount-plan-actions-collected crlf)
	(modify ?g (mode EXPANDED))
	(assert
		(plan (id ?plan-id) (goal-id ?goal-id))
		(wm-fact (key plan-action ?goal-id ?plan-id expanded-r-2))
	)
)

(defrule production-no-call-clips-smt-r-3
	?g <- (goal (id ?goal-id&COMPLEXITY) (mode SELECTED))
	(not (wm-fact (key plan-action ?goal-id ?plan-id expanded-r-3)) )

	; R-3 should only mark the goal as expanded
	(wm-fact (key config rcll robot-name) (value "R-3"))
	(wm-fact (key plan-action ?goal-id ?plan-id amount-plan-actions-collected-r-3) (value ?amount-plan-actions-collected))
	(wm-fact (key plan-action ?goal-id ?plan-id amount-plan-actions) (value ?amount-plan-actions))
	; (test (eq (string-to-field (str-cat (+ (length$ ?amount-plan-actions-collected) 1))) (string-to-field (str-cat ?amount-plan-actions)) ))
	(test (= ?amount-plan-actions ?amount-plan-actions-collected))
=>
	(printout t "Expand for R-3 with " ?amount-plan-actions " and collected " ?amount-plan-actions-collected crlf)
	(modify ?g (mode EXPANDED))
	(assert
		(plan (id ?plan-id) (goal-id ?goal-id))
		(wm-fact (key plan-action ?goal-id ?plan-id expanded-r-3))
	)
)
