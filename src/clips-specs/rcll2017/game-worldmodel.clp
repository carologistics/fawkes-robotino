;---------------------------------------------------------------------------
;  game-worldmodel.clp - Initialize RefBox communication
;
;  Created: Thu 20 April 2018 14:47:31 CET
;  Copyright  2018  Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>
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

(defrule change-state-ignore
  "State changes are only allowed during play"
  (wm-fact (key refbox phase ~PRODUCTION&~EXPLORATION&~POST_GAME))
  (wm-fact (key refbox state) (value ?refbox-state))
  (not (wm-fact (key game state) (value ?refbox-state)))
  =>
  (printout warn "***** Ignoring state change *****" crlf)
)

(defrule enable-motor-on-start
  "Enables the motor on game start. Motor is not started in clips-simulation."
  ; (declare (salience ?*PRIORITY-HIGH*))
  ; (phase EXPLORATION|PRODUCTION|WHACK_A_MOLE_CHALLENGE)
  ; (state WAIT_START)
  ; (change-state RUNNING)
  (wm-fact (key refbox phase) (value  EXPLORATION|PRODUCTION))
  (wm-fact (key refbox state) (value RUNNING))
  ?sf <- (wm-fact (key game state) (value ~RUNNING))
  ; (not (simulation-is-running))
  =>
  (printout warn "***** Enabling motor *****" crlf)
  (retract ?sf)
  (assert (wm-fact (key game state) (type UNKNOWN) (value RUNNING)))
  ; (motor-enable)
)

(defrule pause
  "If game state is something in the middle of the game (not PAUSED or WAIT_START) and the requested change state is not RUNNING change states to PAUSED. The motor is disabled. Rule is not applied in clips-simulation."
  ; ?sf <- (state ?state&~PAUSED&~WAIT_START)
  ; ?cf <- (change-state ?cs&~RUNNING)
  ; ?rf <- (refbox-state ~?cs)
  (wm-fact (key refbox phase PRODUCTION|EXPLORATION|POST_GAME))
  (wm-fact (key game state) (value ?refbox-state&~RUNNING))
  ?sf <- (wm-fact (key game state) (value ?game-state&~PAUSED&~WAIT_START))
  ; (not (simulation-is-running))
  =>
  (printout warn "Paused, disabling motor" crlf)
  (retract ?sf)
  (assert  (wm-fact (key game state) (type UNKNOWN) (value PAUSED)))
  ; (motor-disable)
)
