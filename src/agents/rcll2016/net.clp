
;---------------------------------------------------------------------------
;  net.clp - Robotino agent -- network functions
;
;  Created: Wed Apr 24 21:50:12 2013 (Magdeburg)
;  Copyright  2013  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defrule net-enable-local-public
  "Enable local peer connection to the unencrypted refbox channel"
  (confval (path "/clips-agent/rcll2016/peer-address") (value ?peer-address))
  (confval (path "/clips-agent/rcll2016/peer-send-port") (value ?peer-send-port))
  (confval (path "/clips-agent/rcll2016/peer-recv-port") (value ?peer-recv-port))
  (not (peer-enabled))
  =>
  (printout t "Enabling local peer (public)" crlf)
  (bind ?peer-id (pb-peer-create-local ?peer-address ?peer-send-port ?peer-recv-port))
  (assert (peer-enabled)
	  (peer-id public ?peer-id))
)

(defrule net-enable-public
  "Enable peer connection to the unencrypted refbox channel"
  (confval (path "/clips-agent/rcll2016/peer-address") (value ?peer-address))
  (confval (path "/clips-agent/rcll2016/peer-port") (value ?peer-port))
  (not (peer-enabled))
  =>
  (printout t "Enabling remote peer (public)" crlf)
  (bind ?peer-id (pb-peer-create ?peer-address ?peer-port))
  (assert (peer-enabled)
	  (peer-id public ?peer-id))
)

(defrule net-enable-local-team-private
  "Enable local peer connection to the encrypted team channel"
  (peer-enabled)
  (not (private-peer-enabled))
  (team-color ?team-color&CYAN|MAGENTA)
  (private-peer-address ?address)
  (private-peer-ports ?team-color ?send-port ?recv-port)
  (private-peer-key ?key ?cipher)
  =>
  (printout t "Enabling local peer (cyan only)" crlf)
  (bind ?peer-id (pb-peer-create-local-crypto ?address ?send-port ?recv-port ?key ?cipher))
  (assert (private-peer-enabled)
	  (peer-id private ?peer-id))
)

(defrule net-enable-team-private
  "Enable peer connection to the encrypted team channel"
  (peer-enabled)
  (not (private-peer-enabled))
  (team-color ?team-color&CYAN|MAGENTA)
  (private-peer-address ?address)
  (private-peer-port ?team-color ?port)
  (private-peer-key ?key ?cipher)
  =>
  (printout t "Enabling remote peer (cyan only)" crlf)
  (bind ?peer-id (pb-peer-create-crypto ?address ?port ?key ?cipher))
  (assert (private-peer-enabled)
	  (peer-id private ?peer-id))
)

(defrule net-send-BeaconSignal
  (time $?now)
  ?f <- (timer (name beacon) (time $?t&:(timeout ?now ?t ?*BEACON-PERIOD*)) (seq ?seq))
  (team-color ?team-color&CYAN|MAGENTA)
  (peer-id private ?peer)
  (Position3DInterface (id "Pose") (translation $?trans) (rotation $?ori) (time $?ptime))
  =>
  (modify ?f (time ?now) (seq (+ ?seq 1)))
  (if (debug 3) then (printout t "Sending beacon" crlf))
  (bind ?beacon (pb-create "llsf_msgs.BeaconSignal"))
  (bind ?beacon-time (pb-field-value ?beacon "time"))
  (pb-set-field ?beacon-time "sec" (nth$ 1 ?now))
  (pb-set-field ?beacon-time "nsec" (* (nth$ 2 ?now) 1000))
  (pb-set-field ?beacon "time" ?beacon-time) ; destroys ?beacon-time!
  (pb-set-field ?beacon "seq" ?seq)
  (pb-set-field ?beacon "team_name" ?*TEAM-NAME*)
  (pb-set-field ?beacon "peer_name" ?*ROBOT-NAME*)
  (pb-set-field ?beacon "team_color" ?team-color)
  (pb-set-field ?beacon "number" ?*ROBOT-NUMBER*)

  (bind ?beacon-pose (pb-field-value ?beacon "pose"))
  (pb-set-field ?beacon-pose "x" (nth$ 1 ?trans))
  (pb-set-field ?beacon-pose "y" (nth$ 2 ?trans))
  (pb-set-field ?beacon-pose "ori" (tf-yaw-from-quat ?ori))
  (bind ?beacon-pose-time (pb-field-value ?beacon-pose "timestamp"))
  (pb-set-field ?beacon-pose-time "sec" (nth$ 1 ?ptime))
  (pb-set-field ?beacon-pose-time "nsec" (* (nth$ 2 ?ptime) 1000))
  (pb-set-field ?beacon-pose "timestamp" ?beacon-pose-time)
  (pb-set-field ?beacon "pose" ?beacon-pose)

  (pb-broadcast ?peer ?beacon)
  (pb-destroy ?beacon)
)

(defrule net-recv-RingInfo
  ?pf <- (protobuf-msg (type "llsf_msgs.RingInfo") (ptr ?p))
  =>
  (foreach ?r (pb-field-list ?p "rings")
    (bind ?color (utils-remove-prefix (pb-field-value ?r "ring_color") RING_))
    (if (not (any-factp ((?ring ring)) (eq ?ring:color ?color)))
      then
        (bind ?req-bases (pb-field-value ?r "raw_material"))
        (assert (ring (color ?color) (req-bases ?req-bases)))
    )
  )
)

(defrule net-recv-GameState
  (phase ?phase)
  ?gt <- (game-time $?)
  (refbox-state ?state)
  ?pf <- (protobuf-msg (type "llsf_msgs.GameState") (ptr ?p) (rcvd-from ?host ?port))
  ?tc <- (team-color ?team-color)
  ?pm <- (points-magenta ?)
  ?pc <- (points-cyan ?)
  =>
  (retract ?pf ?gt ?pm ?pc)
  (bind ?new-state (pb-field-value ?p "state"))
  (bind ?new-phase (pb-field-value ?p "phase"))
  (bind ?new-team-color ?team-color)
  (if (and (pb-has-field ?p "team_cyan")
					 (eq (pb-field-value ?p "team_cyan") ?*TEAM-NAME*))
		then (bind ?new-team-color CYAN))
  (if (and (pb-has-field ?p "team_magenta")
					 (eq (pb-field-value ?p "team_magenta") ?*TEAM-NAME*))
		then (bind ?new-team-color MAGENTA))
  (if (neq ?new-team-color ?team-color) then
    (printout warn "Switching team color from " ?team-color " to " ?new-team-color crlf)
    (retract ?tc)
    (assert (team-color ?new-team-color))
  )
  ; (printout t "GameState received from " ?host ":" ?port ": "
  ; 	    ?state " <> " ?new-state "  " ?phase " <> " ?new-phase crlf)
  (if (neq ?phase ?new-phase) then
    (assert (change-phase ?new-phase))
  )
  (if (neq ?state ?new-state) then
    (assert (change-state ?new-state))
  )
  (bind ?time (pb-field-value ?p "game_time"))
  (bind ?sec (pb-field-value ?time "sec"))
  (bind ?nsec (pb-field-value ?time "nsec"))
  (assert (game-time (create$ ?sec (/ ?nsec 1000)))
	  (points-magenta (pb-field-value ?p "points_magenta"))
	  (points-cyan (pb-field-value ?p "points_cyan"))
  )
)

(defrule net-recv-BeaconSignal
  ?pf <- (protobuf-msg (type "llsf_msgs.BeaconSignal") (ptr ?p))
  (time $?now)
  =>
  (bind ?beacon-name (pb-field-value ?p "peer_name"))
  (do-for-all-facts ((?active-robot active-robot)) (eq ?active-robot:name (sym-cat ?beacon-name))
      ;no modify because the active-robot-fact may not be there before
      (retract ?active-robot)
  )
  (bind ?x 0.0)
  (bind ?y 0.0)
  (if (pb-has-field ?p "pose")
    then
    (bind ?beacon-pose (pb-field-value ?p "pose"))
    (bind ?x (pb-field-value ?beacon-pose "x"))
    (bind ?y (pb-field-value ?beacon-pose "y"))
  )
  (assert (active-robot (name (sym-cat ?beacon-name)) (last-seen ?now) (x ?x) (y ?y)))     
  (retract ?pf) 
)

(defrule net-recv-order
  ; Assert orders sent by the refbox. Only done by master to avoid conflicting random ids of linked products
  ?pf <- (protobuf-msg (type "llsf_msgs.OrderInfo") (ptr ?p))
  (team-color ?team)
  (lock-role MASTER)
  =>
  (foreach ?o (pb-field-list ?p "orders")
    ;check if this order is for our team and if the order is new
    (if (not (any-factp ((?order order)) (eq ?order:id (pb-field-value ?o "id"))))
        then
          (bind ?product-id (random-id))
          (bind ?id (pb-field-value ?o "id"))
          (bind ?complexity (pb-field-value ?o "complexity"))
          (bind ?delivery-gate (pb-field-value ?o "delivery_gate"))
          (bind ?quantity-requested (pb-field-value ?o "quantity_requested"))
          (bind ?begin (pb-field-value ?o "delivery_period_begin"))
          (bind ?end (pb-field-value ?o "delivery_period_end"))
          (if (pb-has-field ?o "base_color") then
            (bind ?base (utils-remove-prefix (pb-field-value ?o "base_color") BASE_))
          else
            (bind ?base UNKNOWN)
          )
          (bind ?rings (create$ ))
          (progn$ (?ring (pb-field-list ?o "ring_colors"))
            (bind ?rings (append$ ?rings 
            (utils-remove-prefix ?ring RING_)))
          )
          (bind ?cap (utils-remove-prefix (pb-field-value ?o "cap_color") CAP_))
          (synced-assert (str-cat
                          "(order (id " ?id ")(product-id " ?product-id ")(complexity " ?complexity
                          ")(delivery-gate " ?delivery-gate ")(quantity-requested " ?quantity-requested
                          ")(begin " ?begin ")(end " ?end "))"))
          (synced-assert (str-cat
                          "(product (id " ?product-id ")(base " ?base
                          ")(rings " (implode$ ?rings) ")(cap " ?cap "))"))
          (printout t "Added order " ?id " with " (pb-field-value ?o "cap_color") crlf)
      else
      (do-for-fact ((?order order)) (eq ?order:id (pb-field-value ?o "id"))
          (if (eq ?team CYAN) then
            (bind ?quantity-delivered (pb-field-value ?o "quantity_delivered_cyan"))
          else
            (bind ?quantity-delivered (pb-field-value ?o "quantity_delivered_magenta"))
          )
          (synced-modify ?order quantity-delivered ?quantity-delivered)
      )
    )
  )
  (retract ?pf)
)
