;---------------------------------------------------------------------------
;  refbox.clp - Refbox communitcation and actions
;
;  Created: Mon 8 Jan 2017 13:21:21 CET
;  Copyright  2017  Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file in the doc directory.
;---------------------------------------------------------------------------

(defrule refbox-init
  "Initialization of refbox related facts."
  (executive-init)
  (domain-loaded)
  =>
  (assert 
    (wm-fact (id "/refbox/team-color") )
    (wm-fact (id "/refbox/points/magenta") (type UINT) (value 0) )
    (wm-fact (id "/refbox/points/cyan") (type UINT) (value 0) )
    (wm-fact (id "/refbox/phase")  (type UNKNOWN) (value PRE_GAME) )
    (wm-fact (id "/refbox/state")  (type UNKNOWN) (value WAIT_START) )
    (wm-fact (id "/refbox/game-time?sec=0.0,nsec=0.0")  (type BOOL) (value TRUE) )
  )  
)

(defrule refbox-comm-enable-local-public
  "Enable local peer connection to the unencrypted refbox channel"
  (wm-fact (id "/config/rcll/peer-address") (value ?peer-address))
  (wm-fact (id "/config/rcll/peer-send-port") (value ?peer-send-port))
  (wm-fact (id "/config/rcll/peer-recv-port") (value ?peer-recv-port))
  (not (wm-fact (id "/refbox/comm/peer-enabled") (value TRUE)))
  =>
  (printout t "Enabling local peer (public)" crlf)
  (bind ?peer-id (pb-peer-create-local ?peer-address ?peer-send-port ?peer-recv-port))
  (assert (wm-fact (id "/refbox/comm/peer-enabled") (value TRUE) (type BOOL))
          (wm-fact (id "/refbox/comm/peer-id/public") (value ?peer-id) (type INT))
   )
)

(defrule refbox-comm-enable-public
  "Enable peer connection to the unencrypted refbox channel"
  (confval (path "/config/rcll/peer-address") (value ?peer-address))
  (confval (path "/config/rcll/peer-port") (value ?peer-port))
  (not (wm-fact (id "/refbox/comm/peer-enabled") (value TRUE)))
  =>
  (printout t "Enabling remote peer (public)" crlf)
  (bind ?peer-id (pb-peer-create ?peer-address ?peer-port))
  (assert (wm-fact (id "/refbox/comm/peer-enabled") (value TRUE) (type BOOL))
          (wm-fact (id "/refbox/comm/peer-id/public") (value ?peer-id) (type INT))
   )
)

(defrule refbox-recv-BeaconSignal
  ?pf <- (protobuf-msg (type "llsf_msgs.BeaconSignal") (ptr ?p))
  (time $?now)
  =>
  (bind ?beacon-name (pb-field-value ?p "peer_name"))
  (printout t "Beacon Recieved from " ?beacon-name crlf)
  (retract ?pf) 
)


(defrule refbox-recv-GameState
  ?pf <- (protobuf-msg (type "llsf_msgs.GameState") (ptr ?p) (rcvd-from ?host ?port))
  ?gt <- (wm-fact (key refbox game-time $?args )  (value TRUE))
  ?rp <- (wm-fact (id "/refbox/phase")  (value ?phase) )
  ?rs <- (wm-fact (id "/refbox/state")  (value ?state) )
  ?pm <- (wm-fact (id "/refbox/points/magenta"))
  ?pc <- (wm-fact (id "/refbox/points/cyan")) 
  ?tc <- (wm-fact (id "/refbox/team-color")  (value ?team-color) )
  (wm-fact (id "/config/rcll/team-name")  (value ?team-name) )
  =>
  (retract ?pf ?gt ?pm ?pc)
  (bind ?new-state (pb-field-value ?p "state"))
  (bind ?new-phase (pb-field-value ?p "phase"))
  (bind ?new-team-color ?team-color)
  (if (and (pb-has-field ?p "team_cyan")
           (eq (pb-field-value ?p "team_cyan") ?team-name))
    then (bind ?new-team-color CYAN))
  (if (and (pb-has-field ?p "team_magenta")
           (eq (pb-field-value ?p "team_magenta") ?team-name))
    then (bind ?new-team-color MAGENTA))
  (if (neq ?new-team-color ?team-color) then
    (printout warn "Switching team color from " ?team-color " to " ?new-team-color crlf)
    (retract ?tc)
    (assert  (wm-fact (id "/refbox/team-color")  (value ?new-team-color) ))
  )
  (if (neq ?phase ?new-phase) then
    (retract ?rp)
    (assert  (wm-fact (id "/refbox/phase")  (value ?new-phase) ))
  )
  (if (neq ?state ?new-state) then
    (retract ?rs)
    (assert  (wm-fact (id "/refbox/state")  (value ?new-state) ))
  )
  (bind ?time (pb-field-value ?p "game_time"))
  (bind ?sec (pb-field-value ?time "sec"))
  (bind ?nsec (pb-field-value ?time "nsec"))
  (assert (wm-fact (key refbox game-time args? sec ?sec nsec ?nsec ) (type BOOL) (value TRUE)) )
  (assert (wm-fact (id "/refbox/points/magenta") (type UINT) (value (pb-field-value ?p "points_magenta")) ))
  (assert (wm-fact (id "/refbox/points/cyan") (type UINT) (value (pb-field-value ?p "points_cyan")) ))
)


(defrule net-recv-order
  "Assert orders sent by the refbox."
  ; An order is modled in the pddl domain as the predicats
  ; (order-complexity ?ord - order ?com - order-complexity)
  ; (order-base-color ?ord - order ?col - base-color)
  ; (order-ring1-color ?ord - order ?col - ring-color)
  ; (order-ring2-color ?ord - order ?col - ring-color)
  ; (order-ring3-color ?ord - order ?col - ring-color)
  ; (order-cap-color ?ord - order ?col - cap-color)
  ; (order-gate ?ord - order ?gate - ds-gate)
  ?pf <- (protobuf-msg (type "llsf_msgs.OrderInfo") (ptr ?ptr))
  (wm-fact (id "/refbox/team-color") (value ?team-color) )
  =>
  (foreach ?o (pb-field-list ?ptr "orders")
    (bind ?id (pb-field-value ?o "id"))
    (bind ?order-id (sym-cat o ?id))
    ;check if the order is new
    (if (not (any-factp ((?wm-fact wm-fact)) (and (wm-key-prefix ?wm-fact:key (create$ domain fact order-complexity) )
                                                  (eq ?order-id (nth$ (+ (member$ order ?wm-fact:key) 1) ?wm-fact:key)))))
        then
          (bind ?complexity (pb-field-value ?o "complexity"))
          (bind ?delivery-gate (pb-field-value ?o "delivery_gate"))
          (bind ?quantity-requested (pb-field-value ?o "quantity_requested"))
          (bind ?begin (pb-field-value ?o "delivery_period_begin"))
          (bind ?end (pb-field-value ?o "delivery_period_end"))
          (if (pb-has-field ?o "base_color") then
            (bind ?base (pb-field-value ?o "base_color"))
          else
            (bind ?base UNKNOWN)
          )
          (bind ?cap (pb-field-value ?o "cap_color"))
          (progn$ (?p (pb-field-list ?o "ring_colors"))
            (assert (wm-fact (key domain fact (sym-cat order- ring ?p-index -color) args? order ?order-id ring-color ?p) (type BOOL) (value TRUE) ))
          )
          (assert 
            (wm-fact (key domain fact order-complexity args? order ?order-id complexity ?complexity) (type BOOL) (value TRUE) )
            (wm-fact (key domain fact order-base-color args? order ?order-id base-color ?base) (type BOOL) (value TRUE) )
            (wm-fact (key domain fact order-cap-color  args? order ?order-id cap-color ?cap) (type BOOL) (value TRUE) )
            (wm-fact (key domain fact order-gate  args? order ?order-id ds-gate (sym-cat GATE- ?delivery-gate)) (type BOOL) (value TRUE) )
            (wm-fact (key refbox order ?order-id quantity-requested) (type UINT) (value ?quantity-requested) )
            (wm-fact (key refbox order ?order-id delivery-begin) (type UINT) (value ?begin) )
            (wm-fact (key refbox order ?order-id delivery-end) (type UINT) (value ?end) )
            )
          (printout t "Added order " ?id " with " (pb-field-value ?o "cap_color") crlf)
      else
          (if (eq ?team-color CYAN) then
            (bind ?quantity-delivered (pb-field-value ?o "quantity_delivered_cyan"))
          else
            (bind ?quantity-delivered (pb-field-value ?o "quantity_delivered_magenta"))
          )      
          (do-for-fact ((?old-deliverd-fact wm-fact)) (wm-key-prefix ?old-deliverd-fact:key (create$ refbox order ?order-id quantity-delivered ?team-color)) 
              (retract ?old-deliverd-fact)
          )
          (assert (wm-fact (key refbox order ?order-id quantity-delivered ?team-color) (type UINT) (value ?quantity-delivered) ))
    )
  )
  (retract ?pf)
)
