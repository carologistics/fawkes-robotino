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


