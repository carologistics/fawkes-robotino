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

(defrule game-change-state-ignore
  "State changes are only allowed during play"
  (wm-fact (key refbox phase) (value ~PRODUCTION&~EXPLORATION&~POST_GAME))
  (wm-fact (key refbox state) (value ?refbox-state))
  (not (wm-fact (key game state) (value ?refbox-state)))
=>
  (printout warn "***** Ignoring state change *****" crlf)
)


(defrule game-enable-motor-on-start
  "Enables the motor on game start. Motor is not started in clips-simulation."
  (wm-fact (key refbox phase) (value  EXPLORATION|PRODUCTION))
  (wm-fact (key refbox state) (value RUNNING))
  ?sf <- (wm-fact (key game state) (value ~RUNNING))
  (wm-fact (key central agent robot args? r ?robot))
=>
  (printout warn "***** Enabling motor *****" crlf)
  (modify ?sf (value RUNNING))
  (motors-enable ?robot)
)


(defrule game-paused
  "If game state is something in the middle of the game (not PAUSED or WAIT_START) and the requested change state is not RUNNING change states to PAUSED. The motor is disabled. Rule is not applied in clips-simulation."
  (wm-fact (key refbox phase) (value PRODUCTION|EXPLORATION|POST_GAME))
  (wm-fact (key refbox state) (value ?refbox-state&~RUNNING))
  ?sf <- (wm-fact (key game state) (value ?game-state&~PAUSED&~WAIT_START))
  (wm-fact (key central agent robot args? r ?robot))
=>
  (printout warn "***** Paused, Disabling motor *****" crlf)
  (modify ?sf (value PAUSED))
  (do-for-all-facts ((?agent-wm wm-fact))
                    (wm-key-prefix ?agent-wm:key (create$ central agent robot))
    (motors-disable (wm-key-arg ?agent-wm:key r))
  )
)


(defrule game-add-ground-truth-for-tags
  "Integrate incoming field-ground-truth facts into our world model."
  ?gt <-  (wm-fact (key refbox field-ground-truth name args? m ?mps))
  ?gt-y <-(wm-fact (key refbox field-ground-truth yaw args? m ?mps) (value ?yaw))
  ?gt-z <-(wm-fact (key refbox field-ground-truth zone args? m ?mps) (value ?zone))
  ?gt-t <-(wm-fact (key refbox field-ground-truth mtype args? m ?mps) (value ?mtype))
  ?gt-o <-(wm-fact (key refbox field-ground-truth orientation args? m ?mps) (value ?ori))
  (not (wm-fact (key game found-tag name args? m ?mps)))
=>
  (bind ?mirror-yaw ?yaw)
  (bind ?mirror-name (mirror-name ?mps))
  (bind ?mirror-rot (tf-quat-from-yaw ?mirror-yaw))
  (bind ?mirror-zone ?zone)
  (bind ?mirror-trans (tag-offset ?zone ?mirror-yaw 0.17))

  (if TRUE then
    (bind ?mirror-yaw (deg-to-rad (mirror-orientation ?mtype ?zone ?ori)))
    (bind ?mirror-rot (tf-quat-from-yaw ?mirror-yaw))
    (bind ?mirror-zone (mirror-name ?zone))
    (bind ?mirror-trans (tag-offset (mirror-name ?zone) ?mirror-yaw 0.17))
  )

  (assert
    (wm-fact (key game found-tag name args? m ?mps) (type BOOL) (value TRUE))
    (wm-fact (key game found-tag side args? m ?mps) (value INPUT))
    (wm-fact (key game found-tag frame args? m ?mps) (type STRING) (value "map"))
    (wm-fact (key game found-tag trans args? m ?mps) (type FLOAT) (is-list TRUE) (values (tag-offset ?zone ?yaw 0.17)))
    (wm-fact (key game found-tag rot args? m ?mps) (type FLOAT) (is-list TRUE) (values (tf-quat-from-yaw ?yaw )))
    (wm-fact (key game found-tag zone args? m ?mps) (value ?zone))
    ; add tags for the other team
    (wm-fact (key game found-tag name args? m (mirror-name ?mps)) (type BOOL) (value TRUE))
    (wm-fact (key game found-tag side args? m (mirror-name ?mps)) (value INPUT))
    (wm-fact (key game found-tag frame args? m (mirror-name ?mps)) (type STRING) (value "map"))
    (wm-fact (key game found-tag trans args? m (mirror-name ?mps)) (type FLOAT) (is-list TRUE) (values ?mirror-trans))
    (wm-fact (key game found-tag rot args? m (mirror-name ?mps)) (type FLOAT) (is-list TRUE) (values ?mirror-rot))
    (wm-fact (key game found-tag zone args? m (mirror-name ?mps)) (value ?mirror-zone))
  )

  (retract ?gt ?gt-t ?gt-z ?gt-y ?gt-o)
)

(defrule game-generate-waitpoints-for-machines
  "Generate waitpoint objects for each machine input/output side that is free based on game logic"
  (wm-fact (key game found-tag name args? m ?mps))
  (wm-fact (key domain fact mps-type args? m ?mps t ?type))
  =>
  (if (neq ?type DS) then
    (assert (domain-object (name (sym-cat "WAIT-" ?mps -O)) (type waitpoint)))
  )
  (assert (domain-object (name (sym-cat "WAIT-" ?mps -I)) (type waitpoint)))
)
