;---------------------------------------------------------------------------
;  goal-lock-expiration.clp - Periodically check for expired locks
;
;  Created: Sun 17 Jun 2018 14:24:51 EDT
;  Copyright  2018  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

(defglobal
  ?*LOCK-EXPIRATION-PERIOD* = 15
)

(defrule goal-lock-expiration-create-maintain
  (not (goal (class LOCKEXPIRE)))
  (wm-fact (key refbox phase) (value EXPLORATION|PRODUCTION))
  =>
  (assert (goal (id (sym-cat LOCKEXPIRE-MAINTAIN- (gensym*)))
                (type MAINTAIN) (class LOCKEXPIRE-MAINTAIN)))
)

(defrule goal-lock-expiration-create-achieve
  ?g <- (goal (id ?maintain-id) (class LOCKEXPIRE-MAINTAIN) (mode SELECTED))
  (not (goal (class LOCKEXPIRE-ACHIEVE)))
  (time $?now)
  (goal-meta (goal-id ?maintain-id)
    (last-achieve $?last&:(timeout ?now ?last ?*LOCK-EXPIRATION-PERIOD*)))
  =>
  (assert (goal (id (sym-cat LOCKEXPIRE-ACHIEVE- (gensym*)))
                (class LOCKEXPIRE-ACHIEVE) (parent ?maintain-id)))
)

(defrule goal-lock-expiration-create-plan
  (wm-fact (key cx identity) (value ?self))
  ?p <- (goal (id ?parent-id) (class LOCKEXPIRE-MAINTAIN) (mode EXPANDED))
  ?g <- (goal (id ?goal-id) (parent ?parent-id)
              (class LOCKEXPIRE-ACHIEVE) (mode SELECTED))
  =>
  (assert
    (plan (id LOCKEXPIRE-PLAN) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id LOCKEXPIRE-PLAN) (goal-id ?goal-id)
      (action-name expire-locks) (param-values ?self))
  )
  (modify ?g (mode EXPANDED))
)
