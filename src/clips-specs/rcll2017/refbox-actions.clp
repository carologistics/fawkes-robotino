;---------------------------------------------------------------------------
;  custom-actions.clp - Actions that interact with the RefBox
;
;  Created:: Mon 10 Jan 2017 13:21:21 CET
;  Copyright  2017  Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file in the doc directory.
;---------------------------------------------------------------------------

(defglobal
  ; network sending periods; seconds
  ?*BEACON-PERIOD* = 1.0
)

(defrule action-send-beacon-signal
	(time $?now)
  	?f <- (timer (name beacon) (time $?t&:(timeout ?now ?t ?*BEACON-PERIOD*)) (seq ?seq))
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (status PENDING)
                      (action-name send-beacon) (executable TRUE)
                      (param-names $?param-names)
                      (param-values $?param-values))
  	(wm-fact (id "/config/rcll/team-name")  (value ?team-name) )
  	(wm-fact (id "/config/rcll/robot-name")  (value ?robot-name) )
  	(wm-fact (id "/config/rcll/robot-number")  (value ?robot-number) )
  	(wm-fact (id "/refbox/team-color") (value ?team-color&:(neq ?team-color nil)))
  	(wm-fact (id "/refbox/comm/peer-id/public") (value ?peer) (type INT))
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
	(pb-set-field ?beacon "team_name" ?team-name)
	(pb-set-field ?beacon "peer_name" ?robot-name)
	(pb-set-field ?beacon "team_color" ?team-color)
	(pb-set-field ?beacon "number" ?robot-number)

	(pb-broadcast ?peer ?beacon)
	(pb-destroy ?beacon)
	(modify ?pa (status FINAL))
)
