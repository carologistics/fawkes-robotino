;---------------------------------------------------------------------------
;  refbox-comm-init.clp - Initialize RefBox communication
;
;  Created: Thu 11 Jan 2018 14:47:31 CET
;  Copyright  2018  Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>
;                   Till Hofmann <hofmann@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file in the doc directory.
;---------------------------------------------------------------------------

; This program is free software; you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation; either version 2 of the License, or
; (at your option) any later version.
;
; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU Library General Public License for more details.
;
; Read the full text in the LICENSE.GPL file in the doc directory.
;

(defrule refbox-init
  "Initialization of refbox related facts."
  (executive-init)
  =>
  (assert
    (wm-fact (id "/refbox/team-color") )
    (wm-fact (id "/refbox/points/magenta") (type UINT) (value 0) )
    (wm-fact (id "/refbox/points/cyan") (type UINT) (value 0) )
    (wm-fact (id "/refbox/phase")  (value PRE_GAME) )
    (wm-fact (id "/refbox/state")  (value WAIT_START) )
    (wm-fact (id "/game/state")  (value WAIT_START) )
    (wm-fact (id "/refbox/game-time")  (type UINT) (is-list TRUE) (values 0 0))
    (wm-fact (key refbox beacon seq) (type UINT) (value 1))
  )
)

(defrule refbox-comm-enable-public
  "Enable peer connection to the unencrypted refbox channel"
  ; (declare (salience ?*PRIORITY-LOW*))
  (executive-init)
  (wm-fact (id "/config/rcll/peer-address") (value ?peer-address))
  (wm-fact (id "/config/rcll/peer-port") (value ?peer-port&~0))
  (not (wm-fact (id "/refbox/comm/peer-enabled") (value TRUE)))
  =>
  (printout t "Enabling remote peer (public)" crlf)
  (bind ?peer-id (pb-peer-create ?peer-address ?peer-port))
  (assert (wm-fact (id "/refbox/comm/peer-enabled") (value TRUE) (type BOOL))
          (wm-fact (id "/refbox/comm/peer-id/public") (value ?peer-id) (type INT))
   )
)

(defrule refbox-comm-enable-local-public
  "Enable local peer connection to the unencrypted refbox channel"
  (executive-init)
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

(defrule refbox-comm-close-local-public
  "Disable the local peer connection on finalize"
  (executive-finalize)
  ?pe <- (wm-fact (id "/refbox/comm/peer-enabled") (value TRUE))
  (wm-fact (id "/refbox/comm/peer-id/public") (value ?peer-id) (type INT))
  =>
  (printout t "Closing local peer (public)" crlf)
  (pb-peer-destroy ?peer-id)
  (modify ?pe (value FALSE))
)


(defrule refbox-comm-close-public
  "Disable the remote peer connection on finalize"
  (executive-finalize)
  ?pe <- (wm-fact (id "/refbox/comm/peer-enabled") (value TRUE) (type BOOL))
  (wm-fact (id "/refbox/comm/peer-id/public") (value ?peer-id) (type INT))
  =>
  (printout t "Closing remote peer (public)" crlf)
  (pb-peer-destroy ?peer-id)
  (modify ?pe (value FALSE))
)

(defrule refbox-comm-enable-local-team-private
  "Enable local peer connection to the encrypted team channel"
  (executive-init)
  (wm-fact (id "/refbox/comm/peer-enabled") (value TRUE))
  (wm-fact (id "/refbox/team-color") (value ?team-color&:(neq ?team-color nil)))
  (wm-fact (id "/config/rcll/peer-address") (value ?address))
  (wm-fact (id "/config/rcll/crypto-key") (value ?key))
  (wm-fact (id "/config/rcll/cipher") (value ?cipher))
  (wm-fact (id "/config/rcll/cyan-recv-port") (value ?cyan-recv-port))
  (wm-fact (id "/config/rcll/cyan-send-port") (value ?cyan-send-port))
  (wm-fact (id "/config/rcll/magenta-recv-port") (value ?magenta-recv-port))
  (wm-fact (id "/config/rcll/magenta-send-port") (value ?magenta-send-port))
  (not (wm-fact (id "/refbox/comm/private-peer-enabled") (value TRUE) ))
  =>
  (if (eq ?team-color CYAN)
    then
      (printout t "Enabling local peer (cyan only)" crlf)
      (bind ?peer-id (pb-peer-create-local-crypto ?address ?cyan-send-port ?cyan-recv-port ?key ?cipher))
      else
      (printout t "Enabling local peer (magenta only)" crlf)
      (bind ?peer-id (pb-peer-create-local-crypto ?address ?magenta-send-port ?magenta-recv-port ?key ?cipher))
    )
  (assert (wm-fact (id "/refbox/comm/private-peer-enabled") (value TRUE) (type BOOL))
          (wm-fact (id "/refbox/comm/peer-id/private") (value ?peer-id) (type INT))
    )
)

(defrule refbox-comm-enable-team-private
  "Enable local peer connection to the encrypted team channel"
  (executive-init)
  (wm-fact (id "/refbox/comm/peer-enabled") (value TRUE))
  (wm-fact (id "/refbox/team-color") (value ?team-color&:(neq ?team-color nil)))
  (wm-fact (id "/config/rcll/peer-address") (value ?address))
  (wm-fact (id "/config/rcll/crypto-key") (value ?key))
  (wm-fact (id "/config/rcll/cipher") (value ?cipher))
  (wm-fact (id "/config/rcll/cyan-port") (value ?cyan-port&~0))
  (wm-fact (id "/config/rcll/magenta-port") (value ?magenta-port&~0))
  (not (wm-fact (id "/refbox/comm/private-peer-enabled") (value TRUE) ))
  =>
  (if (eq ?team-color CYAN)
    then
      (printout t "Enabling remote peer (cyan only)" crlf)
      (bind ?peer-id (pb-peer-create-crypto ?address ?cyan-port ?key ?cipher))
      else
      (printout t "Enabling remote peer (magenta only)" crlf)
      (bind ?peer-id (pb-peer-create-crypto ?address ?magenta-port ?key ?cipher))
    )
  (assert (wm-fact (id "/refbox/comm/private-peer-enabled") (value TRUE) (type BOOL))
          (wm-fact (id "/refbox/comm/peer-id/private") (value ?peer-id) (type INT))
    )
)

(defrule refbox-comm-close-private
  "Disable the local private peer connection on finalize"
  (executive-finalize)
  ?pe <- (wm-fact (id "/refbox/comm/private-peer-enabled") (value TRUE) (type BOOL))
  (wm-fact (id "/refbox/comm/peer-id/private") (value ?peer-id) (type INT))
  =>
  (printout t "Closing local private peer" crlf)
  (pb-peer-destroy ?peer-id)
  (modify ?pe (value FALSE))
)
