;---------------------------------------------------------------------------
;  goal-maintain-production.clp - Generate production goals of RCLL
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
(defrule goal-production-create-beacon-maintain
" The parent goal for beacon signals. Allows formulation of
  goals that periodically communicate with the refbox.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (not (goal (class BEACON-MAINTAIN)))
  (or (domain-facts-loaded)
      (wm-fact (key refbox phase) (value ~SETUP&~PRE_GAME)))
  =>
  (bind ?goal (goal-tree-assert-run-endless BEACON-MAINTAIN 1))
  (modify ?goal (verbosity QUIET) (params frequency 1))
)


(defrule goal-production-create-beacon-achieve
" Send a beacon signal whenever at least one second has elapsed since it
  last one got sent.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (time $?now)
  ?g <- (goal (id ?maintain-id) (class BEACON-MAINTAIN) (mode SELECTED))
  ; TODO: make interval a constant
  =>
  (assert (goal (id (sym-cat SEND-BEACON- (gensym*))) (sub-type SIMPLE)
                (class SEND-BEACON) (parent ?maintain-id) (verbosity QUIET)))
)

(defrule goal-production-create-refill-shelf-maintain
" The parent goal to refill a shelf. Allows formulation of goals to refill
  a shelf only if the game is in the production phase and the domain is loaded.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (domain-facts-loaded)
  (not (goal (class REFILL-SHELF-MAINTAIN)))
  (not (mutex (name ?n&:(eq ?n (resource-to-mutex refill-shelf))) (state LOCKED)))
  (wm-fact (key refbox phase) (value PRODUCTION))
  =>
  (bind ?goal (goal-tree-assert-run-endless REFILL-SHELF-MAINTAIN 1))
  (modify ?goal (required-resources refill-shelf)
                (params frequency 1 retract-on-REJECTED)
                (verbosity QUIET))
)


(defrule goal-production-create-refill-shelf-achieve
  "Refill a shelf whenever it is empty."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?g <- (goal (id ?maintain-id) (class REFILL-SHELF-MAINTAIN) (mode SELECTED))
  (not (goal (class REFILL-SHELF)))
  (wm-fact (key refbox phase) (value PRODUCTION))
  (wm-fact (key game state) (value RUNNING))
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (not (wm-fact (key domain fact wp-on-shelf args? wp ?wp m ?mps spot ?spot)))
  =>
  (assert (goal (id (sym-cat REFILL-SHELF- (gensym*)))
                (class REFILL-SHELF) (sub-type SIMPLE)
                (parent ?maintain-id) (verbosity QUIET)
                (params mps ?mps)))
)


(defrule goal-production-navgraph-compute-wait-positions-finished
  "Add the waiting points to the domain once their generation is finished."
  (NavGraphWithMPSGeneratorInterface (final TRUE))
  (not (NavGraphWithMPSGeneratorInterface (final ~TRUE)))
=>
  (printout t "Navgraph generation of waiting-points finished. Getting waitpoints." crlf)
  (do-for-all-facts ((?waitzone navgraph-node)) (str-index "WAIT-" ?waitzone:name)
    (assert
      (domain-object (name (sym-cat ?waitzone:name)) (type waitpoint))
      (wm-fact (key navgraph waitzone args? name (sym-cat ?waitzone:name)) (is-list TRUE) (type INT) (values (nth$ 1 ?waitzone:pos) (nth$ 2 ?waitzone:pos)))
    )
  )
  (assert (wm-fact (key navgraph waitzone generated) (type BOOL) (value TRUE)))
)

(defrule goal-production-create-enter-field
  "Enter the field (drive outside of the starting box)."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (not (goal (class ENTER-FIELD)))
  (wm-fact (key domain fact robot-waiting args? r ?robot))
  (wm-fact (key refbox state) (value RUNNING))
  (wm-fact (key refbox phase) (value PRODUCTION|EXPLORATION))
  (wm-fact (key navgraph waitzone generated) (type BOOL) (value TRUE))
  (wm-fact (key refbox team-color) (value ?team-color))
  ; (NavGraphGeneratorInterface (final TRUE))
  ; (not (wm-fact (key domain fact entered-field args? r ?robot)))
  =>
  (printout t "Goal " ENTER-FIELD " formulated for " ?robot crlf)
  (goal-tree-assert-retry ENTER-FIELD-LOOP 999 (assert (goal (id (sym-cat ENTER-FIELD- (gensym*)))
                (class ENTER-FIELD) (sub-type SIMPLE)
                (params r ?robot team-color ?team-color))))
)


(defrule goal-production-create
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (wm-fact (key refbox phase) (value PRODUCTION))
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  (wm-fact (key refbox team-color) (value ?team-color))
  ; (wm-fact (key domain fact quantity-delivered args? ord ?order team ?team-color) (value ?qd&:(> ?qr ?qd)))
  ; (not (goal (class BUILD) (params ?order)))
  (not (order-processed ?order))
  =>
  (loop-for-count (?i 1 ?qr) 
    (assert
        (goal (id (sym-cat BUILD- ?order - ?i))
              (class BUILD) (type ACHIEVE) (sub-type RUN-SUBGOALS-IN-PARALLEL) (params ?order))
    )
  )
  (assert (order-processed ?order))
)


(defrule goal-expander-build-C0
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?g <- (goal (id ?parent-id) (mode SELECTED) (class BUILD) (params ?order))
  (wm-fact (key domain fact order-complexity args? ord ?order com C0))
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
  => 
   (assert
        (goal (id (sym-cat DELIVER- (gensym*))) (parent ?parent-id)
                  (class DELIVER) (type ACHIEVE) (sub-type SIMPLE) (mode SELECTED)
                  (params order ?order))
    )
    (modify ?g (mode EXPANDED) (meta ?order))
)


(defrule goal-production-transport-deliver
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (parent ?parent-id) (mode SELECTED) (class DELIVER) (params order ?order))
  ; Order facts
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?ring2-color))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?ring3-color))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
  ; DS facts
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-type args? m ?ds t DS))
  (wm-fact (key domain fact mps-team args? m ?ds col ?team-color))
  ; wp facts
  (not (exists
    (wm-fact (key domain fact wp-at args? wp ?wp m ?ds side INPUT))
    (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
    (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))
    (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?ring2-color))
    (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?ring3-color))
    (wm-fact (key domain fact wp-cap-color args? wp ?wp col ?cap-color))
  ))
  ; Subgoal does not exist yet
  (not (goal (class TRANSPORT) (parent ?parent-id) (params mps-to ?ds base-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color ring3-color ?ring3-color cap-color ?cap-color)))
  => 
   (assert
        (goal (id (sym-cat TRANSPORT- (gensym*))) (parent ?parent-id)
                  (class TRANSPORT) (type ACHIEVE) (sub-type SIMPLE) (mode SELECTED)
                  (params mps-to ?ds base-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color ring3-color ?ring3-color cap-color ?cap-color))
    )
)


(defrule goal-production-mount-cap
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (parent ?parent-id) (mode SELECTED) (class TRANSPORT) (params mps-to ?ds base-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color ring3-color ?ring3-color cap-color ?cap_color))
  (goal (id ?parent-id) (meta ?order))
  ; Order facts
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?ring2-color))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?ring3-color))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
  ; wp facts
  (not (exists
    (wm-fact (key domain fact wp-at args? wp ?wp m ?mps-from side ?mps-from-side))
    (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
    (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))
    (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?ring2-color))
    (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?ring3-color))
    (wm-fact (key domain fact wp-cap-color args? wp ?wp col ?cap-color))
  ))
  ; Subgoal does not exist yet
  (not (goal (class MOUNT-CAP) (parent ?parent-id) (params base-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color ring3-color ?ring3-color cap-color ?cap-color)))
  => 
   (assert
        (goal (id (sym-cat MOUNT-CAP- (gensym*))) (parent ?parent-id)
                  (class MOUNT-CAP) (type ACHIEVE) (sub-type SIMPLE) (mode SELECTED)
                  (params base-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color ring3-color ?ring3-color cap-color ?cap-color))
    )
)


(defrule goal-production-transport-base-cs
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (parent ?parent-id) (mode SELECTED) (class MOUNT-CAP) (params base-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color ring3-color ?ring3-color cap-color ?cap-color))
  ; CS facts
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-type args? m ?cs t CS))
  (wm-fact (key domain fact mps-team args? m ?cs col ?team-color))
  (wm-fact (key domain fact cs-buffered args? m ?cs col ?cap-color))
  ; wp facts
  (not (exists
    (wm-fact (key domain fact wp-at args? wp ?wp m ?cs side INPUT))
    (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
    (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))
    (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?ring2-color))
    (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?ring3-color))
    (wm-fact (key domain fact wp-cap-color args? wp ?wp col CAP_NONE))
  ))
  ; Subgoal does not exist yet
  (not (goal (class TRANSPORT) (parent ?parent-id) (params mps-to ?cs base-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color ring3-color ?ring3-color cap-color CAP_NONE)))
  => 
   (assert
        (goal (id (sym-cat TRANSPORT- (gensym*))) (parent ?parent-id)
                  (class TRANSPORT) (type ACHIEVE) (sub-type SIMPLE) (mode SELECTED)
                  (params mps-to ?cs base-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color ring3-color ?ring3-color cap-color CAP_NONE))
    )
)


(defrule goal-production-prepare-cap
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (parent ?parent-id) (mode SELECTED) (class MOUNT-CAP) (params base-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color ring3-color ?ring3-color cap-color ?cap-color))
  ; CS facts
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-type args? m ?cs t CS))
  (wm-fact (key domain fact mps-team args? m ?cs col ?team-color))
  (wm-fact (key domain fact cs-color args? m ?cs col ?cap-color))
  (not (wm-fact (key domain fact cs-buffered args? m ?cs col ?cap-color)))
  ; Subgoal does not exist yet
  (not (goal (class PREPARE-CAP) (parent ?parent-id) (params cap-color ?cap-color)))
  => 
   (assert
        (goal (id (sym-cat PREPARE-CAP- (gensym*))) (parent ?parent-id)
                  (class PREPARE-CAP) (type ACHIEVE) (sub-type SIMPLE) (mode SELECTED)
                  (params cap-color ?cap-color))
    )
)


(defrule goal-production-discard-base
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (parent ?parent-id) (mode SELECTED) (class MOUNT-CAP) (params base-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color ring3-color ?ring3-color cap-color ?cap-color))
  ; CS facts
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-type args? m ?cs t CS))
  (wm-fact (key domain fact mps-team args? m ?cs col ?team-color))
  (wm-fact (key domain fact cs-color args? m ?cs col ?cap-color))
  ; CS is buffered
  (wm-fact (key domain fact cs-buffered args? m ?cs col ?cap-color))
  ; WP is there
  (wm-fact (key domain fact wp-at args? wp ?wp m ?cs side INPUT))
  (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?ring2-color))
  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?ring3-color))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col CAP_NONE))
  ; Output not free
  (not (wm-fact (key domain fact mps-side-free args? m ?cs side OUTPUT)))
  ; Subgoal does not exist yet
  (not (goal (class DISCARD-BASE) (parent ?parent-id) (params cs ?cs)))
  => 
   (assert
        (goal (id (sym-cat DISCARD-BASE- (gensym*))) (parent ?parent-id)
                  (class DISCARD-BASE) (type ACHIEVE) (sub-type SIMPLE) (mode SELECTED)
                  (params cs ?cs))
    )
)


; Create base for transport single base goal
(defrule goal-production-create-base
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (parent ?parent-id) (mode SELECTED) (class TRANSPORT) (params mps-to ?mps base-color ?base-color ring1-color RING_NONE ring2-color RING_NONE ring3-color RING_NONE cap-color CAP_NONE))
  ; wp facts
  (not (exists
    (wm-fact (key domain fact wp-at args? wp ?wp m ?mps-from side ?mps-from-side))
    (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
    (wm-fact (key domain fact wp-ring1-color args? wp ?wp col RING_NONE))
    (wm-fact (key domain fact wp-ring2-color args? wp ?wp col RING_NONE))
    (wm-fact (key domain fact wp-ring3-color args? wp ?wp col RING_NONE))
    (wm-fact (key domain fact wp-cap-color args? wp ?wp col CAP_NONE))
  ))
  ; Subgoal does not exist yet
  (not (goal (class MOUNT-CAP) (parent ?parent-id) (params base-color ?base-color)))
  => 
   (assert
        (goal (id (sym-cat CREATE-BASE- (gensym*))) (parent ?parent-id)
                  (class CREATE-BASE) (type ACHIEVE) (sub-type SIMPLE) (mode SELECTED)
                  (params base-color ?base-color))
    )
)


; (defrule goal-expander-build-C1
;   (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
;   ?g <- (goal (id ?parent-id) (mode SELECTED) (class BUILD) (params ?order))
;   (wm-fact (key domain fact order-complexity args? ord ?order com C1))
;   (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
;   (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
;   (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
;   (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
;   (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?cs spot ?shelf-spot))
;   (wm-fact (key domain fact rs-filled-with args? m ?rs n ?bases-filled))
;   (wm-fact (key domain fact rs-ring-spec args? m ?rs r ?ring1-color rn ?bases-needed))
;   => 
;     (printout t "Expanding BUILD-C1, formulating subgoals bases-needed" ?bases-needed crlf)
;     (bind ?spawned-wp (sym-cat WP- (random-id)))
;     (assert 
;       (goal (id (sym-cat FETCH-BASE- (gensym*))) (parent ?parent-id)
;                   (class FETCH-BASE) (type ACHIEVE) (sub-type SIMPLE)
;                   (params base-color ?base-color spawned-wp ?spawned-wp))
;       (goal (id (sym-cat PREPARE-CAP- (gensym*))) (parent ?parent-id)
;                   (class PREPARE-CAP) (type ACHIEVE) (sub-type SIMPLE)
;                   (params cc ?cc))
;       (goal (id (sym-cat DISCARD-BASE- (gensym*))) (parent ?parent-id)
;                   (class DISCARD-BASE) (type ACHIEVE) (sub-type SIMPLE)
;                   (params cc ?cc))
;     )
;     (if (and (eq ?bases-needed TWO) (eq ?bases-filled ZERO)) then
;       (assert
;         ; Feed two rings
;         (goal (id (sym-cat FEED-RS- (gensym*))) (parent ?parent-id)
;                     (class FEED-RS) (type ACHIEVE) (sub-type SIMPLE)
;                     (params rs ?rs))
;         (goal (id (sym-cat FEED-RS- (gensym*))) (parent ?parent-id)
;                     (class FEED-RS) (type ACHIEVE) (sub-type SIMPLE)
;                     (params rs ?rs))
;       )
;     )
;     (if (or (and (eq ?bases-needed TWO) (eq ?bases-filled ONE))
;             (and (eq ?bases-needed ONE) (eq ?bases-filled ZERO))) then
;       (assert
;         ; Feed one rings
;         (goal (id (sym-cat FEED-RS- (gensym*))) (parent ?parent-id)
;                     (class FEED-RS) (type ACHIEVE) (sub-type SIMPLE)
;                     (params rs ?rs))
;       )
;     )
;     (assert
;       (goal (id (sym-cat TRANSPORT- (gensym*))) (parent ?parent-id)
;                   (class TRANSPORT) (type ACHIEVE) (sub-type SIMPLE)
;                   (params wp ?spawned-wp mps-from C-BS mps-from-side INPUT mps-to ?rs))
;       (goal (id (sym-cat MOUNT-RING1- (gensym*))) (parent ?parent-id)
;                   (class MOUNT-RING1) (type ACHIEVE) (sub-type SIMPLE)
;                   (params wp ?spawned-wp ring-color ?ring1-color))
;       (goal (id (sym-cat TRANSPORT- (gensym*))) (parent ?parent-id)
;                   (class TRANSPORT) (type ACHIEVE) (sub-type SIMPLE)
;                   (params wp ?spawned-wp mps-from ?rs mps-from-side OUTPUT mps-to ?cs))
;       (goal (id (sym-cat MOUNT-CAP- (gensym*))) (parent ?parent-id)
;                   (class MOUNT-CAP) (type ACHIEVE) (sub-type SIMPLE)
;                   (params spawned-wp ?spawned-wp mps ?cs cap-color ?cap-color))
;       (goal (id (sym-cat DELIVER- (gensym*))) (parent ?parent-id)
;                   (class DELIVER) (type ACHIEVE) (sub-type SIMPLE)
;                   (params wp ?spawned-wp m ?cs side OUTPUT order ?order))
;     )
;     (modify ?g (mode EXPANDED) (meta ?order))
; )


(defrule goal-production-create-go-wait
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (wm-fact (key refbox phase) (value PRODUCTION))
  (wm-fact (key game state) (value RUNNING))
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (goal (class GO-WAIT) (params r ?robot)))
  =>
  (assert (goal (id (sym-cat GO-WAIT- ?robot))
                (class GO-WAIT) (sub-type SIMPLE)
                (params r ?robot)))
)
  