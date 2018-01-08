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

(defrule  refbox-comm-enable-public
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


