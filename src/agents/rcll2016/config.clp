;---------------------------------------------------------------------------
;  config.clp - Read config values and add them to the factbase
;
;  Created: Mon May 05 16:26:42 2014
;  Copyright  2014  Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

;Determine the role of the agent
(defrule general-determine-role
  (confval (path "/clips-agent/rcll2016/agent-role") (value ?role))
  =>
  (assert (role (sym-cat ?role)))
  (printout t "I have the role " (str-cat ?role) crlf)
)

(defrule conf-figure-out-waiting-points
  "Reads waiting-point names (identical to navgraph names) from cfg/conf.d/clips-agent.yaml"
   (confval (path "/clips-agent/rcll2016/waiting-points/ins-1-robotino1") (value ?ins-wait-point-ins1-robotino1))
   (confval (path "/clips-agent/rcll2016/waiting-points/ins-1-robotino2") (value ?ins-wait-point-ins1-robotino2))
   (confval (path "/clips-agent/rcll2016/waiting-points/ins-1-robotino3") (value ?ins-wait-point-ins1-robotino3))
   (confval (path "/clips-agent/rcll2016/waiting-points/ins-2-robotino1") (value ?ins-wait-point-ins2-robotino1))
   (confval (path "/clips-agent/rcll2016/waiting-points/ins-2-robotino2") (value ?ins-wait-point-ins2-robotino2))
   (confval (path "/clips-agent/rcll2016/waiting-points/ins-2-robotino3") (value ?ins-wait-point-ins2-robotino3))
   (confval (path "/clips-agent/rcll2016/waiting-points/deliver1") (value ?deliver1-wait-point))
   (confval (path "/clips-agent/rcll2016/waiting-points/deliver2") (value ?deliver2-wait-point))
   =>
  (assert (wait-point Ins1 "R-1" ?ins-wait-point-ins1-robotino1) 
          (wait-point Ins1Sec "R-1" ?ins-wait-point-ins1-robotino1) 
          (wait-point Ins1 "R-2" ?ins-wait-point-ins1-robotino2)
          (wait-point Ins1Sec "R-2" ?ins-wait-point-ins1-robotino2)
          (wait-point Ins1 "R-3" ?ins-wait-point-ins1-robotino3)
          (wait-point Ins1Sec "R-3" ?ins-wait-point-ins1-robotino3)
          (wait-point Ins2 "R-1" ?ins-wait-point-ins2-robotino1)
          (wait-point Ins2Sec "R-1" ?ins-wait-point-ins2-robotino1)
          (wait-point Ins2 "R-2" ?ins-wait-point-ins2-robotino2)
          (wait-point Ins2Sec "R-2" ?ins-wait-point-ins2-robotino2)
          (wait-point Ins2 "R-3" ?ins-wait-point-ins2-robotino3)
          (wait-point Ins2Sec "R-3" ?ins-wait-point-ins2-robotino3)
          (wait-point deliver1 "R-1" ?deliver1-wait-point) 
          (wait-point deliver1 "R-2" ?deliver1-wait-point) 
          (wait-point deliver1 "R-3" ?deliver1-wait-point) 
          (wait-point deliver2 "R-1" ?deliver2-wait-point)
          (wait-point deliver2 "R-2" ?deliver2-wait-point)
          (wait-point deliver2 "R-3" ?deliver2-wait-point)
  )
)

(defrule conf-machine-proc-times
  "Reads production times from cfg/conf.d/clips-agent.yaml"
  (phase PRODUCTION)
  (confval (path "/clips-agent/rcll2016/production-times/t1-min") (value ?proc-min-time-t1))
  (confval (path "/clips-agent/rcll2016/production-times/t1-max") (value ?proc-max-time-t1))
  (confval (path "/clips-agent/rcll2016/production-times/t2-min") (value ?proc-min-time-t2))
  (confval (path "/clips-agent/rcll2016/production-times/t2-max") (value ?proc-max-time-t2))
  (confval (path "/clips-agent/rcll2016/production-times/t3-min") (value ?proc-min-time-t3))
  (confval (path "/clips-agent/rcll2016/production-times/t3-max") (value ?proc-max-time-t3))
  (confval (path "/clips-agent/rcll2016/production-times/t4-min") (value ?proc-min-time-t4))
  (confval (path "/clips-agent/rcll2016/production-times/t4-max") (value ?proc-max-time-t4))
  (confval (path "/clips-agent/rcll2016/production-times/t5-min") (value ?proc-min-time-t5))
  (confval (path "/clips-agent/rcll2016/production-times/t5-max") (value ?proc-max-time-t5))
  (confval (path "/clips-agent/rcll2016/production-times/recycle-min") (value ?proc-min-time-recycle))
  (confval (path "/clips-agent/rcll2016/production-times/recycle-max") (value ?proc-max-time-recycle))
  =>
  (assert (production-time T1 ?proc-min-time-t1 ?proc-max-time-t1)
    (production-time T2 ?proc-min-time-t2 ?proc-max-time-t2)
    (production-time T3 ?proc-min-time-t3 ?proc-max-time-t3)
    (production-time T4 ?proc-min-time-t4 ?proc-max-time-t4)
    (production-time T5 ?proc-min-time-t5 ?proc-max-time-t5)
    (production-time RECYCLE ?proc-min-time-recycle ?proc-max-time-recycle)
  )
)

(defrule conf-read-out-of-order-times
  "Reads team specific ports for encrypted communication in the simulation from cfg/conf.d/clips-agent.yaml"
  (confval (path "/clips-agent/rcll2016/out-of-order-times/min") (value ?min))
  (confval (path "/clips-agent/rcll2016/out-of-order-times/max") (value ?max))
  (confval (path "/clips-agent/rcll2016/out-of-order-times/recycle-min") (value ?recycle-min))
  (confval (path "/clips-agent/rcll2016/out-of-order-times/recycle-max") (value ?recycle-max))
  =>
  (assert (out-of-order-time min ?min)
	  (out-of-order-time max ?max)
	  (out-of-order-time recycle-min ?recycle-min)
	  (out-of-order-time recycle-max ?recycle-max))
)

(defrule conf-read-comm-config
  "Reads general values needed for communication from cfg/conf.d/clips-agent.yaml"
  (confval (path "/clips-agent/rcll2016/peer-address") (value ?address))
  (confval (path "/clips-agent/rcll2016/crypto-key") (value ?key))
  (confval (path "/clips-agent/rcll2016/cipher") (value ?cipher))
  =>
  (assert (peer-address ?address)
	  (private-peer-key ?key ?cipher))
)

(defrule conf-team-specific-ports-remote
  "Reads team specific ports for encrypted communication from cfg/conf.d/clips-agent.yaml"
  (confval (path "/clips-agent/rcll2016/peer-address") (value ?address))
  (confval (path "/clips-agent/rcll2016/cyan-port") (value ?cyan-port))
  (confval (path "/clips-agent/rcll2016/magenta-port") (value ?magenta-port))
  =>
  (assert (private-peer-address ?address)
	  (private-peer-port CYAN ?cyan-port)
	  (private-peer-port MAGENTA ?magenta-port))
)

(defrule conf-team-specific-ports-local
  "Reads team specific ports for encrypted communication in the simulation from cfg/conf.d/clips-agent.yaml"
  (confval (path "/clips-agent/rcll2016/peer-address") (value ?address))
  (confval (path "/clips-agent/rcll2016/cyan-send-port") (value ?cyan-send-port))
  (confval (path "/clips-agent/rcll2016/cyan-recv-port") (value ?cyan-recv-port))
  (confval (path "/clips-agent/rcll2016/magenta-send-port") (value ?magenta-send-port))
  (confval (path "/clips-agent/rcll2016/magenta-recv-port") (value ?magenta-recv-port))
  =>
  (assert (private-peer-address ?address)
	  (private-peer-ports CYAN ?cyan-send-port ?cyan-recv-port)
	  (private-peer-ports MAGENTA ?magenta-send-port ?magenta-recv-port))
)

(defrule conf-get-puck-storage-points
  "Read configuration for puck-storage-points"
  (declare (salience ?*PRIORITY-WM*))
  (team-color ?team-color&CYAN|MAGENTA)
  (confval (path "/clips-agent/rcll2016/puck-storage-points/cyan") (list-value $?psp-cyan))
  (confval (path "/clips-agent/rcll2016/puck-storage-points/magenta") (list-value $?psp-magenta))
  =>
  (bind $?storage-points (create$))
  (if (eq ?team-color CYAN)
    then
    (bind $?storage-points $?psp-cyan)
    else
    (bind $?storage-points $?psp-magenta)
  )

  (progn$ (?p ?storage-points)
    (assert (puck-storage (name (sym-cat ?p)) (team ?team-color)))
  )
)

(defrule conf-assign-cap-colors-to-machines
  "Read configuration for which cap station provides which cap-color"
  (declare (salience ?*PRIORITY-WM*))
  ?c0 <- (confval (path "/clips-agent/rcll2016/cap-station/assigned-color/C-CS1") (value ?cap-color-ccs1))
  ?c1 <- (confval (path "/clips-agent/rcll2016/cap-station/assigned-color/C-CS2") (value ?cap-color-ccs2))
  ?c2 <- (confval (path "/clips-agent/rcll2016/cap-station/assigned-color/M-CS1") (value ?cap-color-mcs1))
  ?c3 <- (confval (path "/clips-agent/rcll2016/cap-station/assigned-color/M-CS2") (value ?cap-color-mcs2))
  ?ccs1 <- (cap-station (name C-CS1))
  ?ccs2 <- (cap-station (name C-CS2))
  ?mcs1 <- (cap-station (name M-CS1))
  ?mcs2 <- (cap-station (name M-CS2))
  =>
  (modify ?ccs1 (assigned-cap-color (sym-cat ?cap-color-ccs1)))
  (modify ?ccs2 (assigned-cap-color (sym-cat ?cap-color-ccs2)))
  (modify ?mcs1 (assigned-cap-color (sym-cat ?cap-color-mcs1)))
  (modify ?mcs2 (assigned-cap-color (sym-cat ?cap-color-mcs2)))
  ; retract the confvals so the rule does not fire again after modifying the cap-station facts
  (retract ?c0 ?c1 ?c2 ?c3)
)

(defrule conf-get-move-into-field-waittime
  ?cr1 <- (confval (path "/clips-agent/rcll2016/move-into-field-waittime/R-1") (type UINT) (value ?wait-r-1))
  ?cr2 <- (confval (path "/clips-agent/rcll2016/move-into-field-waittime/R-2") (type UINT) (value ?wait-r-2))
  ?cr3 <- (confval (path "/clips-agent/rcll2016/move-into-field-waittime/R-3") (type UINT) (value ?wait-r-3))
  =>
  (if (eq ?*ROBOT-NAME* "R-1")
    then
    (assert (move-into-field-waittime ?wait-r-1))
  )
  (if (eq ?*ROBOT-NAME* "R-2")
    then
    (assert (move-into-field-waittime ?wait-r-2))
  )
  (if (eq ?*ROBOT-NAME* "R-3")
    then
    (assert (move-into-field-waittime ?wait-r-3))
  )
  (retract ?cr1 ?cr2 ?cr3)
)

