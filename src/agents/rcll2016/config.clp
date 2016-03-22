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
  ?cr4 <- (confval (path "/clips-agent/rcll2016/move-into-field-waittime/R-1") (type UINT) (value ?wait-r-4))
  ?cr5 <- (confval (path "/clips-agent/rcll2016/move-into-field-waittime/R-2") (type UINT) (value ?wait-r-5))
  ?cr6 <- (confval (path "/clips-agent/rcll2016/move-into-field-waittime/R-3") (type UINT) (value ?wait-r-6))
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
  (if (eq ?*ROBOT-NAME* "R-4")
    then
    (assert (move-into-field-waittime ?wait-r-4))
  )
  (if (eq ?*ROBOT-NAME* "R-5")
    then
    (assert (move-into-field-waittime ?wait-r-5))
  )
  (if (eq ?*ROBOT-NAME* "R-6")
    then
    (assert (move-into-field-waittime ?wait-r-6))
  )
  (retract ?cr1 ?cr2 ?cr3 ?cr4 ?cr5 ?cr6)
)

