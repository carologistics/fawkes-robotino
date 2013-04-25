
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
