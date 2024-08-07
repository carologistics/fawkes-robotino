;---------------------------------------------------------------------------
;  refbox-agent-task.clp - maintain task info for each robot
;
;  Created: Tue 09 Jan 2018 17:03:31 CET
;  Copyright  2018  Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>
;             2021  Tarik Viehmann <viehmann@kbsg.rwth-aachen.de>
;             2021  Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
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

(deftemplate refbox-agent-task
  (slot task-id (type INTEGER))
  (slot robot (type SYMBOL))
  (slot task-type (type SYMBOL)
    (allowed-values Move Retrieve Deliver BufferStation ExploreWaypoint))
  (slot machine (type SYMBOL)
   (allowed-values UNSET
    C-BS C-CS1 C-CS2 C-RS1 C-RS2 C-DS C-SS
    M-BS M-CS1 M-CS2 M-RS1 M-RS2 M-DS M-SS
   )
   (default UNSET))
  (slot order (type SYMBOL))
  (slot side (type SYMBOL)
    (allowed-values UNSET INPUT OUTPUT LEFT MIDDLE RIGHT SLIDE)
    (default UNSET))
  (slot waypoint (type SYMBOL) (default UNSET))
  (slot workpiece (type SYMBOL) (default nil))
  (multislot workpiece-colors (type SYMBOL) (default (create$)))
  (slot outcome (type SYMBOL) (allowed-values UNKNOWN FAILED CANCELLED SUCCESSFUL))
  (slot goal-id (type SYMBOL))
  (slot plan-id (type SYMBOL))
  (slot action-id (type INTEGER))
)

(defrule task-set-wp-on-move-task
  (declare (salience ?*MONITORING-SALIENCE*))
  (wm-fact (key refbox robot task seq args? r ?robot) (value ?seq))
  ?task <- (refbox-agent-task (task-id ?seq) (robot ?robot) (task-type Move)
    (workpiece nil))
  (wm-fact (key domain fact holding args? r ?robot wp ?wp))
  =>
  (modify ?task (workpiece ?wp))
)

(defrule task-set-wp-info
  (declare (salience ?*MONITORING-SALIENCE*))
  (wm-fact (key refbox robot task seq args? r ?robot) (value ?seq))
  ?task <- (refbox-agent-task (task-id ?seq) (robot ?robot) (workpiece ?wp&~nil)
    (workpiece-colors))
  (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-col))
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?col-r1))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?col-r2))
  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?col-r3))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col ?cap-col))
  =>
  (bind ?order-id nil)
  (do-for-fact ((?wp-for-order wm-fact))
    (and (wm-key-prefix ?wp-for-order:key (create$ order meta wp-for-order))
         (eq (wm-key-arg ?wp-for-order:key wp) ?wp)
    )
    (bind ?order-id (wm-key-arg ?wp-for-order:key ord))
  )
  (modify ?task (workpiece-colors (create$ ?base-col ?col-r1 ?col-r2 ?col-r3 ?cap-col))
                (order ?order-id)
  )
)

(defrule task-set-order-info
  (declare (salience ?*MONITORING-SALIENCE*))
  (wm-fact (key refbox robot task seq args? r ?robot) (value ?seq))
  ?task <- (refbox-agent-task (task-id ?seq) (robot ?robot) (order nil))
  (goal (id ?g-id) (mode DISPATCHED))
  (goal-meta (goal-id ?g-id) (assigned-to ?robot) (order-id ?order-id&~nil))
  =>
  (modify ?task (order ?order-id))
)

(defrule task-set-task-finished
  (declare (salience ?*MONITORING-SALIENCE*))
  ?counter <- (wm-fact (key refbox robot task seq args? r ?robot) (value ?seq))
  ?task <- (refbox-agent-task (task-id ?seq) (robot ?robot)
  (goal-id ?goal-id) (plan-id ?plan-id) (action-id ?action-id)
  )
  (plan-action (id ?action-id) (goal-id ?goal-id) (plan-id ?plan-id)
   (state ?state&:(member$ ?state (create$ FINAL FAILED)))
  )
  =>
  (bind ?outcome ?state)
  (if (eq ?state FINAL) then
   (bind ?outcome SUCCESSFUL)
  )
  (modify ?task (outcome ?outcome))
  (modify ?counter (value (+ ?seq 1)))
)

(defrule register-new-agent-task-move-machine
  (declare (salience ?*MONITORING-SALIENCE*))
  (wm-fact (key refbox robot task seq args? r ?robot) (value ?seq))
  (plan-action (id ?id) (action-name move)
    (param-values ?robot
                  ?from
                  ?from-side
                  ?mps
                  ?side&:(member$ ?side (create$ INPUT OUTPUT)))
    (state WAITING|RUNNING) (goal-id ?goal-id) (plan-id ?plan-id)
  )
  (test (or (neq ?from ?mps) (neq ?from-side ?side)))
  (not (refbox-agent-task (robot ?robot) (task-id ?seq)))
  =>
  (assert (refbox-agent-task (task-id ?seq) (robot ?robot) (task-type Move)
   (machine ?mps) (side ?side) (goal-id ?goal-id) (plan-id ?plan-id)
   (action-id ?id)
  ))
)

(defrule register-new-agent-task-move-go-wait-nav-pos
  ; TODO: fix! temporarily changed to use machine position instead of wait position
  ; as go-wait might be re-implemented and navgraph removed
  (declare (salience ?*MONITORING-SALIENCE*))
  (wm-fact (key refbox robot task seq args? r ?robot) (value ?seq))
  (plan-action (id ?id) (action-name go-wait)
    (param-values ?robot
                  ?from
                  ?from-side
                  ?target)
    (state WAITING|RUNNING) (goal-id ?goal-id) (plan-id ?plan-id)
  )
  (test (or (neq ?from ?target) (neq ?from-side WAIT)))
  (not (refbox-agent-task (robot ?robot) (task-id ?seq)))
  (wm-fact (key game found-tag name args? m ?mps))
  (test (str-index (str-cat ?mps) (str-cat ?target)))
  =>
  (if (str-index (str-cat "-I") (str-cat ?target))
   then
    (bind ?side INPUT)
    else
    (bind ?side OUTPUT)
  )

  (assert (refbox-agent-task (task-id ?seq) (robot ?robot) (task-type Move)
   (machine ?mps) (side ?side) (goal-id ?goal-id) (plan-id ?plan-id)
   (action-id ?id)
  ))
)

(defrule register-new-agent-task-move-go-wait-wait-zone
  (declare (salience ?*MONITORING-SALIENCE*))
  (wm-fact (key refbox robot task seq args? r ?robot) (value ?seq))
  (plan-action (id ?id) (action-name go-wait)
    (param-values ?robot
                  ?from
                  ?from-side
                  ?zone)
    (state WAITING|RUNNING) (goal-id ?goal-id) (plan-id ?plan-id)
  )
  (not (refbox-agent-task (robot ?robot) (task-id ?seq)))
  (domain-fact (name zone-content) (param-values ?zone ?))
  (test (or (neq ?from ?zone) (neq ?from-side WAIT)))
  =>
  (assert (refbox-agent-task (task-id ?seq) (robot ?robot) (task-type Move)
    (waypoint (sym-cat (protobuf-name ?zone))) (goal-id ?goal-id) (plan-id ?plan-id) (action-id ?id)
  ))
)

(defrule register-new-agent-task-retrieve-wp-get
  (declare (salience ?*MONITORING-SALIENCE*))
  (wm-fact (key refbox robot task seq args? r ?robot) (value ?seq))
  (plan-action (id ?id) (action-name wp-get)
    (param-values ?robot
                  ?wp
                  ?mps
                  ?side $?)
    (state RUNNING|WAITING) (goal-id ?goal-id) (plan-id ?plan-id)
  )
  (wm-fact (key domain fact mps-state args? m ?mps s PROCESSED|READY-AT-OUTPUT|IDLE))
  (not (refbox-agent-task (robot ?robot) (task-id ?seq)))
  =>
  (assert (refbox-agent-task (task-id ?seq) (robot ?robot) (task-type Retrieve)
    (machine ?mps) (side ?side) (workpiece ?wp) (goal-id ?goal-id)
    (plan-id ?plan-id) (action-id ?id)
  ))
)

(defrule register-new-agent-task-retrieve-wp-get-shelf
  (declare (salience ?*MONITORING-SALIENCE*))
  (wm-fact (key refbox robot task seq args? r ?robot) (value ?seq))
  (plan-action (id ?id) (action-name wp-get-shelf)
    (param-values ?robot
                  ?wp
                  ?mps
                  ?shelf-spot $?)
    (state RUNNING|WAITING) (goal-id ?goal-id) (plan-id ?plan-id)
  )
  (not (refbox-agent-task (robot ?robot) (task-id ?seq)))
  =>
  (assert (refbox-agent-task (task-id ?seq) (robot ?robot) (task-type Retrieve)
    (machine ?mps) (side ?shelf-spot) (workpiece ?wp) (goal-id ?goal-id)
    (plan-id ?plan-id) (action-id ?id)
  ))
  ; TODO: is it a good idea to save the shelf spot here?
)


(defrule register-new-agent-task-deliver-wp-put
  (declare (salience ?*MONITORING-SALIENCE*))
  (wm-fact (key refbox robot task seq args? r ?robot) (value ?seq))
  (plan-action (id ?id) (action-name wp-put)
    (param-values ?robot
                  ?wp
                  ?mps
                  ?side $?)
    (state RUNNING|WAITING) (goal-id ?goal-id) (plan-id ?plan-id)
  )
  (not (refbox-agent-task (robot ?robot) (task-id ?seq)))
  =>
  (assert (refbox-agent-task (task-id ?seq) (robot ?robot) (task-type Deliver)
    (machine ?mps) (side ?side) (workpiece ?wp) (goal-id ?goal-id)
    (plan-id ?plan-id) (action-id ?id)
  ))
)

(defrule register-new-agent-task-deliver-wp-put-slide-cc
  (declare (salience ?*MONITORING-SALIENCE*))
  (wm-fact (key refbox robot task seq args? r ?robot) (value ?seq))
  (plan-action (id ?id) (action-name wp-put-slide-cc)
    (param-values ?robot
                  ?wp
                  ?mps
                  $?)
    (state WAITING|RUNNING) (goal-id ?goal-id) (plan-id ?plan-id)
  )
  (not (refbox-agent-task (robot ?robot) (task-id ?seq)))
  =>
  (assert (refbox-agent-task (task-id ?seq) (robot ?robot) (task-type Deliver)
    (machine ?mps) (side SLIDE) (workpiece ?wp) (goal-id ?goal-id)
    (plan-id ?plan-id) (action-id ?id)
  ))
)
