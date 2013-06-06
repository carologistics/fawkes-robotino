
;---------------------------------------------------------------------------
;  net.clp - Robotino agent -- network functions
;
;  Created: Wed Apr 24 21:50:12 2013 (Magdeburg)
;  Copyright  2013  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defrule net-enable-local
  (confval (path "/clips-agent/llsf2013/peer-address") (value ?peer-address))
  (confval (path "/clips-agent/llsf2013/peer-send-port") (value ?peer-send-port))
  (confval (path "/clips-agent/llsf2013/peer-recv-port") (value ?peer-recv-port))
  (not (peer-enabled))
  =>
  (printout t "Enabling local peer" crlf)
  (pb-peer-enable ?peer-address ?peer-send-port ?peer-recv-port)
  (assert (peer-enabled))
)

(defrule net-enable
  (confval (path "/clips-agent/llsf2013/peer-address") (value ?peer-address))
  (confval (path "/clips-agent/llsf2013/peer-port") (value ?peer-port))
  (not (peer-enabled))
  =>
  (printout t "Enabling remote peer" crlf)
  (pb-peer-enable ?peer-address ?peer-port ?peer-port)
  (assert (peer-enabled))
)

(defrule net-send-BeaconSignal
  (time $?now)
  ?f <- (signal (type beacon) (time $?t&:(timeout ?now ?t ?*BEACON-PERIOD*)) (seq ?seq))
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
  (pb-set-field ?beacon "number" ?*ROBOT-NUMBER*)

  (do-for-fact ((?pose Position3DInterface)) (eq ?pose:id "Pose")
    (bind ?beacon-pose (pb-field-value ?beacon "pose"))
    (pb-set-field ?beacon-pose "x" (nth$ 1 ?pose:translation))
    (pb-set-field ?beacon-pose "y" (nth$ 2 ?pose:translation))
    (pb-set-field ?beacon-pose "ori" (* 2 (acos (nth$ 3 ?pose:rotation))))
    (bind ?beacon-pose-time (pb-field-value ?beacon-pose "timestamp"))
    (pb-set-field ?beacon-pose-time "sec" (nth$ 1 ?pose:time))
    (pb-set-field ?beacon-pose-time "nsec" (* (nth$ 2 ?pose:time) 1000))
    (pb-set-field ?beacon-pose "timestamp" ?beacon-pose-time)
    (pb-set-field ?beacon "pose" ?beacon-pose)
  )

  (pb-broadcast ?beacon)
  (pb-destroy ?beacon)
)

(defrule net-recv-GameState
  (phase ?phase)
  (refbox-state ?state)
  ?pf <- (protobuf-msg (type "llsf_msgs.GameState") (ptr ?p) (rcvd-via BROADCAST) (rcvd-from ?host ?port))
  =>
  (retract ?pf)
  (bind ?new-state (pb-field-value ?p "state"))
  (bind ?new-phase (pb-field-value ?p "phase"))
  ;(printout t "GameState received from " ?host ":" ?port ": "
  ;	    ?state " <> " ?new-state "  " ?phase " <> " ?new-phase crlf)

  (if (neq ?phase ?new-phase) then
    (assert (change-phase ?new-phase))
  )
  (if (neq ?state ?new-state) then
    (assert (change-state ?new-state))
  )
)

(defrule net-recv-BeaconSignal
  ?pf <- (protobuf-msg (type "llsf_msgs.BeaconSignal") (ptr ?p) (rcvd-via BROADCAST))
  ?ar <- (active-robot (name ?name) (last-seen $?last-seen) (x ?x) (y ?y))
  =>
  (bind ?beacon-name (pb-field-value ?p "peer_name"))
  (if (eq ?name (sym-cat ?beacon-name))
      then
      (retract ?pf)
      (bind ?beacon-time (pb-field-value ?p "time"))
      (modify ?ar (last-seen ?beacon-time))
      (if (pb-has-field ?p "pose")
	  then
          (bind ?beacon-pose (pb-field-value ?p "pose"))
          (modify ?ar (x (nth$ 1 ?beacon-pose)) (y (nth$ 2 ?beacon-pose)))
	  (printout t "Got Beacon Signal from " ?beacon-name "with Pose" crlf)
  
    )
  )
)

