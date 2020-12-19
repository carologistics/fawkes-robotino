;---------------------------------------------------------------------------
;  fixed-sequence.clp - Goal expander for RCLL goals
;
;  Created: Tue 09 Jan 2018 17:03:31 CET
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

(defrule goal-expander-send-beacon-signal
  ?p <- (goal (mode DISPATCHED) (id ?parent-id))
  ?g <- (goal (id ?goal-id) (class SEND-BEACON) (mode SELECTED)
              (parent ?parent-id))
=>
  (assert
    (plan (id BEACONPLAN) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id BEACONPLAN) (goal-id ?goal-id)
      (action-name send-beacon)))
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-refill-shelf
  ?p <- (goal (mode DISPATCHED) (id ?parent-id))
  ?g <- (goal (id ?goal-id) (class REFILL-SHELF) (mode SELECTED)
              (params mps ?mps) (parent ?parent-id))
  (wm-fact (key domain fact cs-color args? m ?mps col ?col))
  =>
  (assert
    (plan (id REFILL-PLAN) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id REFILL-PLAN) (goal-id ?goal-id)
                 (action-name lock) (param-values ?mps))
    (plan-action (id 2) (plan-id REFILL-PLAN) (goal-id ?goal-id)
                 (action-name refill-shelf)
                 (param-values ?mps LEFT (sym-cat CC- (random-id)) ?col))
    (plan-action (id 3) (plan-id REFILL-PLAN) (goal-id ?goal-id)
                 (action-name refill-shelf)
                 (param-values ?mps MIDDLE (sym-cat CC- (random-id)) ?col))
    (plan-action (id 4) (plan-id REFILL-PLAN) (goal-id ?goal-id)
                 (action-name refill-shelf)
                 (param-values ?mps RIGHT (sym-cat CC- (random-id)) ?col))
    (plan-action (id 5) (plan-id REFILL-PLAN) (goal-id ?goal-id)
                 (action-name unlock) (param-values ?mps))
  )
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-enter-field
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class ENTER-FIELD)
              (params r ?robot team-color ?team-color))
=>
(bind ?planid (sym-cat ENTER-FIELD-PLAN- (gensym*)))
  (assert
    (plan (id ?planid) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ?planid) (goal-id ?goal-id)
                                 (action-name enter-field)
                                 (skiller (remote-skiller ?robot))
                                 (param-names r team-color)
                                 (param-values ?robot ?team-color))
    )
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-visit-machine
  ?p <- (goal (mode DISPATCHED) (id ?parent))
  ?g <- (goal (id ?goal-id) (parent ?parent) (class VISIT-MACHINE) (mode SELECTED) (params machine ?machine side ?side))
  (wm-fact (key domain fact entered-field args? r ?robot))
  (wm-fact (key domain fact at args? r ?robot m ?robot-location side ?robot-side))
  (not(goal (parent ?parent) (mode EXPANDED|DISPATCHED) (params machine ?somemachine side ?someside robot ?robot)))
  =>
  (bind ?planid (sym-cat VISIT-MACHINE- (gensym*)))
  (assert
    (plan (id ?planid) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ?planid) (goal-id ?goal-id)
                  (action-name move)
                  (skiller (remote-skiller ?robot))
                  (param-names r from from-side to to-side )
                  (param-values ?robot ?robot-location ?robot-side ?machine ?side))
  )
  (modify ?g (mode EXPANDED) (params machine ?machine side ?side robot ?robot))
)

;(defrule visit-machine-complement-retractor
;  ?g <- (goal (class VISIT-MACHINE) (mode EXPANDED|DISPATCHED) (params machine ?machine side ?side robot ?robot))
;  ?gcomp <- (goal (class VISIT-MACHINE) (params machine ?machine side ?sidecomp))
;  =>
;  (modify ?gcomp (mode RETRACTED))
;)