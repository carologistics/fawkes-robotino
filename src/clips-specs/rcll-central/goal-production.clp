;---------------------------------------------------------------------------
;  goal-production.clp - Generate production goals of RCLL
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

; ----------------------- Util -------------------------------

(defglobal
  ; defines the spacing between complexity-based prios
  ?*PRODUCTION-PRIO-BASE-STEP* = 10
  ; complexity-based starting prios according to spacing above
  ?*PRODUCTION-C0-PRIORITY* = 30
  ?*PRODUCTION-C1-PRIORITY* = 40
  ?*PRODUCTION-C2-PRIORITY* = 50
  ?*PRODUCTION-C3-PRIORITY* = 60
  ; increas complexity by this for each solved step
  ?*PRODUCTION-PRIORITY-INCREASE* = 100
  ; further bump any delivery goal to most urgent level
  ?*DELIVER-PRIORITY-INCREASE* = 1000
  ; Support priorities
  ; these values should be selected, such that the respective base priorities
  ; are in a range from 1 to ?*PRODUCTION-PRIO-BASE-STEP*.
  ?*PRODUCTION-PAY-PRIORITY* = 1
  ?*PRODUCTION-PAY-CC-PRIORITY-INCREASE* = 2
  ?*PRODUCTION-BUFFER-PRIORITY* = 2
  ?*PRODUCTION-NOTHING-EXECUTABLE-TIMEOUT* = 30
  ?*ROBOT-WAITING-TIMEOUT* = 2
)

(deffunction prio-from-complexity (?com)
  (bind ?priority ?*PRODUCTION-C0-PRIORITY*)
  (if (eq ?com C1) then
    (bind ?priority ?*PRODUCTION-C1-PRIORITY*)
  )
  (if (eq ?com C2) then
    (bind ?priority ?*PRODUCTION-C2-PRIORITY*)
  )
  (if (eq ?com C3) then
    (bind ?priority ?*PRODUCTION-C3-PRIORITY*)
  )
  (return ?priority)
)

(deffunction dynamic-prio-from-complexity (?com ?step)
  (bind ?priority ?*PRODUCTION-C0-PRIORITY*)
  (if (eq ?com C1) then
    (bind ?priority ?*PRODUCTION-C1-PRIORITY*)
  )
  (if (eq ?com C2) then
    (bind ?priority ?*PRODUCTION-C2-PRIORITY*)
  )
  (if (eq ?com C3) then
    (bind ?priority ?*PRODUCTION-C3-PRIORITY*)
  )
  (bind ?priority (- ?priority ?*PRODUCTION-PRIO-BASE-STEP*))
  (if (str-index RING ?step) then
    (bind ?priority (+ ?priority (* (- (string-to-field (sub-string 5 5 ?step)) 1) ?*PRODUCTION-PRIORITY-INCREASE*)))
  )
  (if (eq ?step CAP) then
    (bind ?priority (+ ?priority (* (string-to-field (sub-string 2 2 ?com)) ?*PRODUCTION-PRIORITY-INCREASE*)))
  )
  (if (eq ?step DELIVER) then
    (bind ?priority (+ ?priority (* (+ (string-to-field (sub-string 2 2 ?com)) 1) ?*PRODUCTION-PRIORITY-INCREASE*)))
  )
  (return ?priority)
)

(deffunction goal-meta-assign-robot-to-goal (?goal ?robot)
"Changes an existing goal-meta fact and assign it to the given robot"
  (if (eq (fact-slot-value ?goal id) FALSE) then
    (printout t "Goal has no id! " ?goal crlf)
    (return)
  )
  (if (eq ?robot nil) then (return ))
  (if (not (do-for-fact ((?f goal-meta))
      (and (eq ?f:goal-id (fact-slot-value ?goal id))
           (or (eq ?f:restricted-to ?robot)
               (eq ?f:restricted-to nil)))
      (modify ?f (assigned-to ?robot))))
   then
    (printout t "FAILED assign robot " ?robot " to goal "
      (fact-slot-value ?goal id) crlf)
  )
)

(deffunction goal-meta-assert (?goal ?robot ?order-id ?ring-nr)
"Creates the goal-meta fact, assigns the goal to the robot and to its order"
  (assert (goal-meta (goal-id (fact-slot-value ?goal id))
                     (assigned-to ?robot)
                     (order-id ?order-id)
                     (ring-nr ?ring-nr)))
  (return ?goal)
)

(deffunction goal-meta-assert-restricted (?goal ?robot)
"Creates the goal-meta fact and restricts the goal to the robot"
  (if (neq ?robot nil) then
    (assert (goal-meta (goal-id (fact-slot-value ?goal id))
                       (restricted-to ?robot)))
  )
)

(deffunction goal-production-get-machine-for-color
  (?ring-col)

  (bind ?rs FALSE)
  (do-for-all-facts ((?mps-type wm-fact)) (and (wm-key-prefix ?mps-type:key (create$ domain fact mps-type))
                                               (eq (wm-key-arg ?mps-type:key t) RS))
    (bind ?machine (wm-key-arg ?mps-type:key m))
    (do-for-fact ((?rs-ring-spec wm-fact)) (and (wm-key-prefix ?rs-ring-spec:key (create$ domain fact rs-ring-spec))
                                                (eq (wm-key-arg ?rs-ring-spec:key m) ?machine)
                                                (eq (wm-key-arg ?rs-ring-spec:key r) ?ring-col))
      (bind ?rs ?machine)
    )
  )
  (return ?rs)
)

(deffunction goal-meta-get-goal-category (?goal-class)
  (bind ?production-goals (create$ MOUNT-CAP MOUNT-RING DELIVER-RC21 DELIVER))
  (bind ?maintenance-goals (create$ BUFFER-CAP PAY-FOR-RINGS-WITH-BASE PAY-FOR-RINGS-WITH-CAP-CARRIER PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF))
  (bind ?maintenance-instruct-goals (create$ INSTRUCT-RS-MOUNT-RING INSTRUCT-CS-MOUNT-CAP INSTRUCT-DS-DELIVER))
  (bind ?production-instruct-goals (create$ INSTRUCT-CS-BUFFER-CAP INSTRUCT-DS-DISCARD))
  (bind ?other-goals (create$ MOVE MOVE-OUT-OF-WAY ENTER-FIELD DISCARD WAIT-NOTHING-EXECUTABLE EXPLORATION-MOVE REFILL-SHELF))
  (bind ?other-instruct-goals (create$ INSTRUCT-BS-DISPENSE-BASE))

  (if (member$ ?goal-class ?production-goals) then (return PRODUCTION))
  (if (member$ ?goal-class ?maintenance-goals) then (return MAINTENANCE))
  (if (member$ ?goal-class ?production-instruct-goals) then (return PRODUCTION-INSTRUCT))
  (if (member$ ?goal-class ?maintenance-instruct-goals) then (return MAINTENANCE-INSTRUCT))
  (if (member$ ?goal-class ?other-instruct-goals) then (return OTHER-INSTRUCT))
  (if (member$ ?goal-class ?other-goals) then (return OTHER))

  (return UNKNOWN)
)

(defrule goal-meta-assign-category
  "Assign the category of a simple goal based on its class"
  (goal (id ?goal-id) (sub-type SIMPLE) (class ?class))
  ?gm <- (goal-meta (goal-id ?goal-id) (category nil))
  =>
  (modify ?gm (category (goal-meta-get-goal-category ?class)))
)

(defrule goal-meta-print-error-unknown-category
  (goal (id ?goal-id) (class ?class))
  (goal-meta (goal-id ?goal-id) (category UNKNOWN))
  =>
  (printout error "Simple goal " ?goal-id " of class " ?class " has UNKNOWN category!" crlf)
)

; ----------------------- Maintenance Goals -------------------------------

(defrule goal-production-create-refill-shelf-maintain
" The parent goal to refill a shelf. Allows formulation of goals to refill
  a shelf only if the game is in the production phase and the domain is loaded.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (domain-facts-loaded)
  (not (goal (class REFILL-SHELF-MAINTAIN)))
  (wm-fact (key refbox phase) (value PRODUCTION))
  =>
  (bind ?goal (goal-tree-assert-run-endless REFILL-SHELF-MAINTAIN 1))
  (modify ?goal (required-resources)
                (params frequency 1 retract-on-REJECTED)
                (verbosity QUIET))
)

(defrule goal-production-create-refill-shelf-achieve
  "Refill a shelf whenever it is empty."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?g <- (goal (id ?maintain-id) (class REFILL-SHELF-MAINTAIN) (mode SELECTED))
  (not (goal (class REFILL-SHELF) (mode ~RETRACTED)))
  (wm-fact (key refbox phase) (value PRODUCTION))
  (wm-fact (key game state) (value RUNNING))
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (not (wm-fact (key domain fact wp-on-shelf args? wp ?wp m ?mps spot ?spot)))
  =>
  (bind ?goal (assert (goal (id (sym-cat REFILL-SHELF- (gensym*)))
                (class REFILL-SHELF) (sub-type SIMPLE)
                (parent ?maintain-id) (verbosity QUIET)
                (params mps ?mps) (is-executable TRUE))))
  (goal-meta-assert ?goal central nil nil)
)

; ----------------------- Robot Assignment -------------------------------

(defrule goal-production-assign-robot-to-enter-field
  (wm-fact (key central agent robot args? r ?robot))
  (not (wm-fact (key domain fact entered-field args? r ?robot)))
  (goal (id ?oid) (class ENTER-FIELD)  (sub-type SIMPLE) (mode FORMULATED) (is-executable FALSE))
  ?gm <- (goal-meta (goal-id ?oid) (assigned-to nil))
  (not (goal-meta (assigned-to ?robot)))
  =>
  (goal-meta-assign-robot-to-goal ?oid ?robot)
)

(defrule goal-production-assign-robot-to-simple-goals
" Before checking SIMPLE goals for their executability, pick a waiting robot
  that should get a new goal assigned to it next. "
  (declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
;  "a simple unassigned goal"
  (goal (id ?oid) (sub-type SIMPLE) (mode FORMULATED) (is-executable FALSE))
  (goal-meta (goal-id ?oid) (assigned-to nil))
  (wm-fact (key central agent robot args? r ?robot))
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (goal-meta (assigned-to ?robot)))
  (wm-fact (key central agent robot-waiting args? r ?robot))
	(wm-fact (key refbox game-time) (values $?now))
  ?wt <- (timer (name ?timer-name&:(eq ?timer-name
                                (sym-cat ?robot -waiting-timer)))
	        (time $?t&:(timeout ?now ?t ?*ROBOT-WAITING-TIMEOUT*)))
  =>
  (bind ?longest-waiting 0)
  (bind ?longest-waiting-robot ?robot)
  (delayed-do-for-all-facts ((?waiting wm-fact))
    (wm-key-prefix ?waiting:key (create$ central agent robot-waiting))
    (if (or (eq ?longest-waiting 0) (< (fact-index ?waiting) ?longest-waiting))
     then
      (bind ?longest-waiting-robot (wm-key-arg ?waiting:key r))
      (bind ?longest-waiting (fact-index ?waiting))
    )
  )
  (delayed-do-for-all-facts ((?g goal))
    (and (eq ?g:is-executable FALSE)
         (eq ?g:sub-type SIMPLE) (eq ?g:mode FORMULATED)
         (or (not (any-factp ((?gm  goal-meta))
                            (eq ?gm:goal-id ?g:id)))
             (any-factp ((?gm goal-meta))
                (and (eq ?gm:goal-id ?g:id)
                     (eq ?gm:assigned-to nil)))))
    (goal-meta-assign-robot-to-goal ?g ?robot)
  )
  (modify ?longest-waiting)
  (retract ?wt)
)

(defrule goal-production-unassign-robot-from-finished-goals
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?g <- (goal (id ?id) (sub-type SIMPLE) (mode RETRACTED)
        (parent ?parent))
  (not (goal (id ?parent) (type MAINTAIN)))
  (goal-meta (goal-id ?id) (assigned-to ?robot&~nil))
  =>
  (remove-robot-assignment-from-goal-meta ?g)
)

; ----------------------- Assert Goal Functions -------------------------------

(deffunction goal-production-assert-buffer-cap
  (?mps ?cap-color ?order-id)

  (bind ?goal (assert (goal (class BUFFER-CAP)
        (id (sym-cat BUFFER-CAP- (gensym*))) (sub-type SIMPLE)
        (verbosity NOISY) (is-executable FALSE)
        (params target-mps ?mps
                cap-color ?cap-color)
        (priority (float ?*PRODUCTION-BUFFER-PRIORITY*))
  )))
  (goal-meta-assert ?goal nil ?order-id nil)
  (return ?goal)
)

(deffunction goal-production-assert-pick-and-place
  (?mps ?robot)
  (bind ?goal (assert (goal (class PICK-AND-PLACE)
        (id (sym-cat PICK-AND-PLACE- (gensym*))) (sub-type SIMPLE)
        (verbosity NOISY) (is-executable FALSE)
        (params target-mps ?mps)
  )))
  (goal-meta-assert-restricted ?goal ?robot)
  (return ?goal)
)

(deffunction goal-production-assert-move-robot-to-output
  (?mps ?robot)
  (bind ?goal (assert (goal (class MOVE)
        (id (sym-cat MOVE- (gensym*))) (sub-type SIMPLE)
        (verbosity NOISY) (is-executable FALSE)
        (params target-mps ?mps)
  )))
  (goal-meta-assert-restricted ?goal ?robot)
  (return ?goal)
)

(deffunction goal-production-assert-mount-cap
  (?wp ?mps ?cap-color ?order-id)

  (bind ?goal (assert (goal (class MOUNT-CAP)
        (id (sym-cat MOUNT-CAP- (gensym*))) (sub-type SIMPLE)
         (verbosity NOISY) (is-executable FALSE)
        (params wp ?wp
                target-mps ?mps
                cap-color ?cap-color)
  )))
  (goal-meta-assert ?goal nil ?order-id nil)
  (return ?goal)
)

(deffunction goal-production-assert-mount-ring
  (?wp ?rs ?ring-color ?order-id ?ring-nr)
  (bind ?goal (assert (goal (class MOUNT-RING)
        (id (sym-cat MOUNT-RING- (gensym*))) (sub-type SIMPLE)
        (verbosity NOISY) (is-executable FALSE)
        (params  wp ?wp
                 target-mps ?rs
                 ring-color ?ring-color
                 )
  )))
  (goal-meta-assert ?goal nil ?order-id ?ring-nr)
  (return ?goal)
)

(deffunction goal-production-assert-discard
  (?wp ?target-mps ?order-id)

  (bind ?goal (assert (goal (class DISCARD)
        (id (sym-cat DISCARD- (gensym*))) (sub-type SIMPLE)
        (verbosity NOISY) (is-executable FALSE)
        (params wp ?wp target-mps ?target-mps)
  )))
  (goal-meta-assert ?goal nil ?order-id nil)
  (return ?goal)
)

(deffunction goal-production-assert-deliver-rc21
  (?wp ?order-id)

  (bind ?goal (assert (goal (class DELIVER-RC21)
        (id (sym-cat DELIVER-RC21- (gensym*))) (sub-type SIMPLE)
        (verbosity NOISY) (is-executable FALSE)
        (params wp ?wp)
  )))
  (goal-meta-assert ?goal nil ?order-id nil)
  (return ?goal)
)

(deffunction goal-production-assert-instruct-ds-deliver
  (?wp ?order-id ?ds)

  (bind ?goal (assert (goal (class INSTRUCT-DS-DELIVER)
    (id (sym-cat INSTRUCT-DS-DELIVER- (gensym*))) (sub-type SIMPLE)
    (verbosity NOISY) (is-executable FALSE)
    (params wp ?wp
            target-mps ?ds)
  )))
  (goal-meta-assert ?goal central ?order-id nil)
  (return ?goal)
)

(deffunction goal-production-assert-instruct-ds-discard
  (?wp ?ds)

  (bind ?goal (assert (goal (class INSTRUCT-DS-DISCARD)
    (id (sym-cat INSTRUCT-DS-DISCARD- (gensym*))) (sub-type SIMPLE)
    (verbosity NOISY) (is-executable FALSE)
    (params wp ?wp
            target-mps ?ds)
  )))
  (goal-meta-assert ?goal central nil nil)
  (return ?goal)
)

(deffunction goal-production-assert-deliver
  "If there is a DS, do a normal delivery, otherwise do a RoboCup 2021 delivery. "
  (?wp ?order-id ?instruct-parent ?ds)

  (bind ?goal nil)
  (if (any-factp ((?state domain-fact)) (and (eq ?state:name mps-state)
                                             (member$ ?ds ?state:param-values))
      )
  then

    (bind ?instruct-goal (goal-production-assert-instruct-ds-deliver ?wp ?order-id ?ds))
    (modify ?instruct-goal (parent ?instruct-parent))

    (bind ?goal
      (goal-meta-assert (assert (goal (class DELIVER)
        (id (sym-cat DELIVER- (gensym*))) (sub-type SIMPLE)
        (verbosity NOISY) (is-executable FALSE)
        (params wp ?wp
            target-mps ?ds)
      )) nil ?order-id nil)
    )
  else
    (bind ?goal (goal-production-assert-deliver-rc21 ?wp ?order-id))
  )

  (return ?goal)
)

(deffunction goal-production-assert-pay-for-rings-with-base
  (?wp ?target-mps ?order-id)
  (bind ?goal (assert (goal (class PAY-FOR-RINGS-WITH-BASE)
        (id (sym-cat PAY-FOR-RINGS-WITH-BASE- (gensym*))) (sub-type SIMPLE)
        (verbosity NOISY) (is-executable FALSE)
        (params  wp ?wp
                 target-mps ?target-mps
                 )
        (priority (float ?*PRODUCTION-PAY-PRIORITY*))
  )))
  (goal-meta-assert ?goal nil ?order-id nil)
  (return ?goal)
)

(deffunction goal-production-assert-pay-for-rings-with-cap-carrier
  (?target-mps ?order-id)

  (bind ?goal (assert (goal (class PAY-FOR-RINGS-WITH-CAP-CARRIER)
        (id (sym-cat PAY-FOR-RINGS-WITH-CAP-CARRIER- (gensym*))) (sub-type SIMPLE)
        (verbosity NOISY) (is-executable FALSE)
        (params target-mps ?target-mps
        )
  )))
  (goal-meta-assert ?goal nil ?order-id nil)
  (return ?goal)
)

(deffunction goal-production-assert-pay-for-rings-with-cap-carrier-from-shelf
  (?target-mps ?cap-color ?order-id)

  (bind ?goal (assert (goal (class PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF)
        (id (sym-cat PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF- (gensym*))) (sub-type SIMPLE)
        (verbosity NOISY) (is-executable FALSE)
        (params target-mps ?target-mps
                cap-color ?cap-color
        )
  )))
  (goal-meta-assert ?goal nil ?order-id nil)
  (return ?goal)
)

(deffunction goal-production-assert-instruct-cs-buffer-cap
  (?mps ?cap-color ?order-id)

  (bind ?goal (assert (goal (class INSTRUCT-CS-BUFFER-CAP)
        (id (sym-cat INSTRUCT-CS-BUFFER-CAP- (gensym*))) (sub-type SIMPLE)
        (verbosity NOISY) (is-executable FALSE)
        (params target-mps ?mps
                cap-color ?cap-color)
  )))
  (goal-meta-assert ?goal central ?order-id nil)
  (return ?goal)
)

(deffunction goal-production-assert-instruct-bs-dispense-base
  (?wp ?base-color ?side ?order-id ?bs)

  (bind ?goal (assert (goal (class INSTRUCT-BS-DISPENSE-BASE)
    (id (sym-cat INSTRUCT-BS-DISPENSE-BASE- (gensym*))) (sub-type SIMPLE)
    (verbosity NOISY) (is-executable FALSE)
        (params wp ?wp
                target-mps ?bs
                target-side ?side
                base-color ?base-color)
  )))
  (goal-meta-assert ?goal central ?order-id nil)
  (return ?goal)
)

(deffunction goal-production-assert-instruct-cs-mount-cap
  (?mps ?cap-color ?order-id)
  (bind ?goal (assert (goal (class INSTRUCT-CS-MOUNT-CAP)
        (id (sym-cat INSTRUCT-CS-MOUNT-CAP- (gensym*))) (sub-type SIMPLE)
        (verbosity NOISY) (is-executable FALSE)
        (params target-mps ?mps
                cap-color ?cap-color)
  )))
  (goal-meta-assert ?goal central ?order-id nil)
  (return ?goal)
)

(deffunction goal-production-assert-instruct-rs-mount-ring
  (?mps ?ring-col ?order-id ?ring-nr)
  (bind ?goal (assert (goal (class INSTRUCT-RS-MOUNT-RING)
        (id (sym-cat INSTRUCT-RS-MOUNT-RING- (gensym*))) (sub-type SIMPLE)
        (verbosity NOISY) (is-executable FALSE)
              (params target-mps ?mps
                      ring-color ?ring-col
               )
  )))
  (goal-meta-assert ?goal central ?order-id ?ring-nr)
  (return ?goal)
)

(deffunction goal-production-assert-enter-field
  (?robot ?zone)

  (bind ?goal (assert (goal (class ENTER-FIELD)
              (id (sym-cat ENTER-FIELD- (gensym*)))
              (sub-type SIMPLE)
              (verbosity NOISY) (is-executable FALSE)
              (params zone ?zone)
              (meta-template goal-meta)
  )))
  (goal-meta-assert-restricted ?goal ?robot)
  (return ?goal)
)


(deffunction goal-production-assert-move-out-of-way
	(?location)
	(bind ?goal (assert (goal (class MOVE-OUT-OF-WAY)
	            (id (sym-cat MOVE-OUT-OF-WAY- (gensym*)))
	            (sub-type SIMPLE)
	            (verbosity NOISY) (is-executable FALSE)
	            (meta-template goal-meta)
	            (params target-pos ?location)
	)))
	(return ?goal)
)

(deffunction goal-production-assign-order-and-prio-to-goal (?goal ?order-id ?prio)
  (bind ?goal-id (fact-slot-value ?goal id))
  (modify ?goal (priority ?prio))
  (do-for-fact ((?goal-meta goal-meta)) (eq ?goal-meta:goal-id ?goal-id)
    (modify ?goal-meta (root-for-order ?order-id))
  )
)

(deffunction goal-production-assert-requests
  ;assert requests for an order's support goals (i.e., payment, buffer, discard)
  (?rs ?cs ?ring-cols ?cap-col ?order ?prio)

  (bind ?index 1)
  (bind ?seq 1)
  (loop-for-count (length$ ?rs)
    (bind ?price 0)
    (do-for-fact ((?rs-ring-spec wm-fact))
      (and (wm-key-prefix ?rs-ring-spec:key (create$ domain fact rs-ring-spec))
          (eq (wm-key-arg ?rs-ring-spec:key r ) (nth$ ?index ?ring-cols))
      )
      (bind ?price (sym-to-int (wm-key-arg ?rs-ring-spec:key rn)))
    )
    (loop-for-count ?price
      (assert (wm-fact (key request pay args? ord ?order m (nth$ ?index ?rs) ring (sym-cat RING ?index) seq ?seq prio ?prio) (is-list TRUE) (type SYMBOL) (values status ACTIVE)))
      (bind ?seq (+ ?seq 1))
    )
    (bind ?index (+ ?index 1))
  )
  (assert (wm-fact (key request buffer args? ord ?order col ?cap-col prio ?prio) (is-list TRUE) (type SYMBOL) (values status ACTIVE)))
  (assert (wm-fact (key request discard args? ord ?order cs ?cs prio ?prio) (is-list TRUE) (type SYMBOL) (values status ACTIVE)))
)


(deffunction goal-production-assert-c0
  (?root-id ?order-id ?wp-for-order ?cs ?ds ?bs ?cap-col ?base-col)

  ;assert the instruct goals
  (bind ?instruct-goals
    (goal-tree-assert-central-run-parallel-prio INSTRUCT-ORDER ?*PRODUCTION-C0-PRIORITY*
      (goal-production-assert-instruct-bs-dispense-base ?wp-for-order ?base-col INPUT ?order-id ?bs)
      (goal-production-assert-instruct-cs-mount-cap ?cs ?cap-col ?order-id)
    )
  )
  (bind ?instruct-parent (fact-slot-value ?instruct-goals id))
  (bind ?instruct-goals (modify ?instruct-goals (parent ?root-id) (priority ?*PRODUCTION-C0-PRIORITY*)))

  ;assert the main production tree
  (bind ?goal
    (goal-tree-assert-central-run-all-incremental-prio PRODUCE-ORDER ?*PRODUCTION-C0-PRIORITY* ?*PRODUCTION-PRIORITY-INCREASE*
      (goal-production-assert-deliver ?wp-for-order ?order-id ?instruct-parent ?ds)
      (goal-production-assert-mount-cap ?wp-for-order ?cs ?cap-col ?order-id)
    )
  )

  (goal-production-assign-order-and-prio-to-goal ?goal ?order-id ?*PRODUCTION-C0-PRIORITY*)

  (goal-production-assert-requests (create$ ) ?cs (create$ ) ?cap-col ?order-id ?*PRODUCTION-C0-PRIORITY*)
)

(deffunction goal-production-assert-c1
  (?root-id ?order-id ?wp-for-order ?cs ?ds ?bs ?rs1 ?cap-col ?base-col ?ring-col1)

  ;assert the instruct goals
  (bind ?instruct-goals
    (goal-tree-assert-central-run-parallel-prio INSTRUCT-ORDER ?*PRODUCTION-C1-PRIORITY*
      (goal-production-assert-instruct-bs-dispense-base ?wp-for-order ?base-col INPUT ?order-id ?bs)
      (goal-production-assert-instruct-cs-mount-cap ?cs ?cap-col ?order-id)
      (goal-production-assert-instruct-rs-mount-ring ?rs1 ?ring-col1 ?order-id ONE)
    )
  )
  (bind ?instruct-parent (fact-slot-value ?instruct-goals id))
  (bind ?instruct-goals (modify ?instruct-goals (parent ?root-id) (priority ?*PRODUCTION-C1-PRIORITY*)))

  ;assert the main production tree
  (bind ?goal
    (goal-tree-assert-central-run-all-incremental-prio PRODUCE-ORDER ?*PRODUCTION-C1-PRIORITY* ?*PRODUCTION-PRIORITY-INCREASE*
      (goal-production-assert-deliver ?wp-for-order ?order-id ?instruct-parent ?ds)
      (goal-production-assert-mount-cap ?wp-for-order ?cs ?cap-col ?order-id)
      (goal-production-assert-mount-ring ?wp-for-order ?rs1 ?ring-col1 ?order-id ONE)
    )
  )

  (goal-production-assign-order-and-prio-to-goal ?goal ?order-id ?*PRODUCTION-C1-PRIORITY*)

  (goal-production-assert-requests (create$ ?rs1) ?cs (create$ ?ring-col1) ?cap-col ?order-id ?*PRODUCTION-C1-PRIORITY*)
)

(deffunction goal-production-assert-c2
  (?root-id ?order-id ?wp-for-order ?cs ?ds ?bs ?rs1 ?rs2 ?cap-col ?base-col ?ring-col1 ?ring-col2)

  (bind ?instruct-goals
    (goal-tree-assert-central-run-parallel-prio INSTRUCT-ORDER ?*PRODUCTION-C2-PRIORITY*
      (goal-production-assert-instruct-bs-dispense-base ?wp-for-order ?base-col INPUT ?order-id ?bs)
      (goal-production-assert-instruct-cs-mount-cap ?cs ?cap-col ?order-id)
      (goal-production-assert-instruct-rs-mount-ring ?rs1 ?ring-col1 ?order-id ONE)
      (goal-production-assert-instruct-rs-mount-ring ?rs2 ?ring-col2 ?order-id TWO)
    )
  )
  (bind ?instruct-parent (fact-slot-value ?instruct-goals id))
  (bind ?instruct-goals (modify ?instruct-goals (parent ?root-id) (priority ?*PRODUCTION-C2-PRIORITY*)))

  (bind ?goal
    (goal-tree-assert-central-run-all-incremental-prio PRODUCE-ORDER ?*PRODUCTION-C2-PRIORITY* ?*PRODUCTION-PRIORITY-INCREASE*
      (goal-production-assert-deliver ?wp-for-order ?order-id ?instruct-parent ?ds)
      (goal-production-assert-mount-cap ?wp-for-order ?cs ?cap-col ?order-id)
      (goal-production-assert-mount-ring ?wp-for-order ?rs2 ?ring-col2 ?order-id TWO)
      (goal-production-assert-mount-ring ?wp-for-order ?rs1 ?ring-col1 ?order-id ONE)
    )
  )

  (goal-production-assign-order-and-prio-to-goal ?goal ?order-id ?*PRODUCTION-C2-PRIORITY*)

  (goal-production-assert-requests (create$ ?rs1 ?rs2) ?cs (create$ ?ring-col1 ?ring-col2) ?cap-col ?order-id ?*PRODUCTION-C2-PRIORITY*)
)

(deffunction goal-production-assert-c3
  (?root-id ?order-id ?wp-for-order ?cs ?ds ?bs ?rs1 ?rs2 ?rs3 ?cap-col ?base-col ?ring-col1 ?ring-col2 ?ring-col3)

  (bind ?instruct-goals
    (goal-tree-assert-central-run-parallel-prio INSTRUCT-ORDER ?*PRODUCTION-C3-PRIORITY*
      (goal-production-assert-instruct-bs-dispense-base ?wp-for-order ?base-col INPUT ?order-id ?bs)
      (goal-production-assert-instruct-cs-mount-cap ?cs ?cap-col ?order-id)
      (goal-production-assert-instruct-rs-mount-ring ?rs1 ?ring-col1 ?order-id ONE)
      (goal-production-assert-instruct-rs-mount-ring ?rs2 ?ring-col2 ?order-id TWO)
      (goal-production-assert-instruct-rs-mount-ring ?rs3 ?ring-col3 ?order-id THREE)
    )
  )
  (bind ?instruct-parent (fact-slot-value ?instruct-goals id))
  (bind ?instruct-goals (modify ?instruct-goals (parent ?root-id) (priority ?*PRODUCTION-C3-PRIORITY*)))

  (bind ?goal
    (goal-tree-assert-central-run-all-incremental-prio PRODUCE-ORDER ?*PRODUCTION-C3-PRIORITY* ?*PRODUCTION-PRIORITY-INCREASE*
      (goal-production-assert-deliver ?wp-for-order ?order-id ?instruct-parent ?ds)
      (goal-production-assert-mount-cap ?wp-for-order ?cs ?cap-col ?order-id)
      (goal-production-assert-mount-ring ?wp-for-order ?rs3 ?ring-col3 ?order-id THREE)
      (goal-production-assert-mount-ring ?wp-for-order ?rs2 ?ring-col2 ?order-id TWO)
      (goal-production-assert-mount-ring ?wp-for-order ?rs1 ?ring-col1 ?order-id ONE)
    )
  )

  (goal-production-assign-order-and-prio-to-goal ?goal ?order-id ?*PRODUCTION-C3-PRIORITY*)

  (goal-production-assert-requests (create$ ?rs1 ?rs2 ?rs3) ?cs (create$ ?ring-col1 ?ring-col2 ?ring-col3) ?cap-col ?order-id ?*PRODUCTION-C3-PRIORITY*)
)

(defrule goal-production-create-instruction-root
  "Create the production root under which all instruction goal trees for the orders
  are asserted"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (domain-facts-loaded)
  (not (goal (class INSTRUCTION-ROOT)))
  (wm-fact (key refbox phase) (value PRODUCTION))
  (wm-fact (key game state) (value RUNNING))
  (wm-fact (key refbox team-color) (value ?color))
  (not (wm-fact (key domain fact rs-ring-spec args? $? rn NA)))
  ; Ensure that a MachineInfo was received already.
  ; So if there are ring stations with specs, then those specs are registered.
  (wm-fact (key domain fact mps-state args? m ?any-mps s IDLE))
  =>
  (bind ?g (goal-tree-assert-central-run-parallel INSTRUCTION-ROOT))
  (modify ?g (meta do-not-finish) (priority 1.0))
)

(defrule goal-production-create-support-root
  "Create the SUPPPORT root, which is parent to all goals that are
   supporting the production, i.e., payment and buffer goals.
  "
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (domain-facts-loaded)
  (not (goal (class SUPPORT-ROOT)))
  (wm-fact (key refbox phase) (value PRODUCTION))
  (wm-fact (key game state) (value RUNNING))
  (wm-fact (key refbox team-color) (value ?color))
  =>
  (bind ?g (goal-tree-assert-central-run-parallel SUPPORT-ROOT))
  (modify ?g (meta do-not-finish) (priority ?*PRODUCTION-C3-PRIORITY*))
)

(defrule goal-production-create-wait-root
  "Create the WAIT root, which has low priority and dispatches WAIT goals if
   nothing else is executable.
  "
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (domain-facts-loaded)
  (not (goal (class WAIT-ROOT)))
  (wm-fact (key refbox phase) (value PRODUCTION))
  (wm-fact (key game state) (value RUNNING))
  (wm-fact (key refbox team-color) (value ?color))
  =>
  (bind ?g (goal-tree-assert-central-run-parallel WAIT-ROOT))
  (modify ?g (meta do-not-finish) (priority 0))
)

(defrule goal-production-assert-wait-nothing-executable
  "When the robot is stuck, assert a new goal that keeps it waiting"
  (goal (id ?p) (class WAIT-ROOT))
  (not (goal (parent ?p) (class WAIT-NOTHING-EXECUTABLE) (mode FORMULATED)))
  =>
  (bind ?goal (assert (goal (class WAIT-NOTHING-EXECUTABLE)
              (id (sym-cat WAIT-NOTHING-EXECUTABLE- (gensym*)))
              (sub-type SIMPLE) (parent ?p) (priority 0.0) (meta-template goal-meta)
              (verbosity NOISY)
  )))
)

(defrule goal-production-create-move-out-of-way-goal-tree
	"Creates a move out of way goal with a subgoal for each possible waiting position.
  As soon as it is completed it's reset"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (class INSTRUCTION-ROOT) (mode FORMULATED|DISPATCHED))
  (goal (id ?root-id) (class WAIT-ROOT))
  (not (goal (class MOVE-OUT-OF-WAY)))
  =>
  (bind ?wait-zones (create$ (goal-production-assert-move-out-of-way WAIT1)
                             (goal-production-assert-move-out-of-way WAIT2)
                             (goal-production-assert-move-out-of-way WAIT3)
                             (goal-production-assert-move-out-of-way WAIT4)))
  (bind ?g (goal-tree-assert-central-run-parallel MOVE-OUT-OF-WAY ?wait-zones))
  (modify ?g (parent ?root-id) (priority -1.0))
)

(defrule goal-production-change-priority-move-out-of-way
  (declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
  ?g <- (goal (id ?goal-id) (class MOVE-OUT-OF-WAY)
              (type ACHIEVE) (sub-type SIMPLE)
              (mode FORMULATED) (parent ?pa-id&~nil)
              (priority ?p&:(> ?p 0))
        )
  (goal-meta (goal-id ?goal-id) (assigned-to ?robot))
  (or (not (wm-fact (key monitoring move-out-of-way high-prio long-wait args? r ?robot)))
      (eq ?robot nil))
  =>
  (printout t "modify priority of " ?goal-id crlf)
  (modify ?g (priority -1.0))
)

(defrule goal-production-re-create-move-out-of-way-simple
" Re-creates a retracted move-out-of-way goal on completion for the
	corresponding target wait positions"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?parent-id) (class MOVE-OUT-OF-WAY) (sub-type CENTRAL-RUN-SUBGOALS-IN-PARALLEL))
  (not (and (goal (class MOVE-OUT-OF-WAY) (sub-type SIMPLE) (parent ?parent-id) (params target WAIT1))
       (goal (class MOVE-OUT-OF-WAY) (sub-type SIMPLE) (parent ?parent-id) (params target WAIT2))
       (goal (class MOVE-OUT-OF-WAY) (sub-type SIMPLE) (parent ?parent-id) (params target WAIT3))
       (goal (class MOVE-OUT-OF-WAY) (sub-type SIMPLE) (parent ?parent-id) (params target WAIT4))
  ))
  =>
  (bind ?wait-pos (create$))
  (delayed-do-for-all-facts ((?g goal))
	(and (eq ?g:class MOVE-OUT-OF-WAY)
	     (eq ?g:sub-type SIMPLE)
	)
	(bind ?wait-pos (append$ ?wait-pos (values-from-name-value-list ?g:params)))
  )
  (foreach ?w (create$ WAIT1 WAIT2 WAIT3 WAIT4)
	(if (not (member$ ?w ?wait-pos))
	    then (bind ?f (goal-production-assert-move-out-of-way ?w))
		  (goal-tree-update-child ?f ?parent-id -1.0)
	)
  )
)

(defrule goal-production-create-cleanup-wp
	"Creates a cleanup-wp goal to get rid of WPs that do not belong to any order,
  or step in the production chain e.g. a workpiece left from stopping to pursue an
  order."
	(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
	(goal (class INSTRUCTION-ROOT) (mode FORMULATED|DISPATCHED))
	(goal (id ?root-id) (class WAIT-ROOT))
	(not (goal (class CLEANUP-WP)))
	=>
	(bind ?g (goal-tree-assert-central-run-parallel CLEANUP-WP))
	(modify ?g (parent ?root-id) (priority 0))
)

(defrule goal-production-debug-cap
  "If there is a mismatch between machines and orders, produce output"
  (wm-fact (key domain fact order-cap-color args? ord ?order-id col ?col))
  (not (wm-fact (key domain fact cs-color args? m ?cs col ?col)))
  (goal (id ?root-id) (class INSTRUCTION-ROOT))
  =>
  (printout error "Can not build order " ?order-id " with cap color " ?col " because there is no capstation for it" crlf)
)

(defrule goal-production-debug-ring1
  "If there is a mismatch between machines and orders, produce output"
  (wm-fact (key domain fact order-ring1-color args? ord ?order-id col ?col&~RING_NONE))
  (not (wm-fact (key domain fact rs-ring-spec args? $? r ?col $?)))
  (goal (id ?root-id) (class INSTRUCTION-ROOT))
  =>
  (printout error "Can not build order " ?order-id " with ring-1 color " ?col " because there is no ringstation for it" crlf)
)

(defrule goal-production-debug-ring2
  "If there is a mismatch between machines and orders, produce output"
  (wm-fact (key domain fact order-ring2-color args? ord ?order-id col ?ring-col&~RING_NONE))
  (not (wm-fact (key domain fact rs-ring-spec args? m ?rs r ?ring-col $?)))
  (goal (id ?root-id) (class INSTRUCTION-ROOT))
  =>
  (printout error "Can not build order " ?order-id " with ring-2 color " ?ring-col " because there is no ringstation for it" crlf)
)

(defrule goal-production-debug-ring3
  "If there is a mismatch between machines and orders, produce output"
  (wm-fact (key domain fact order-ring3-color args? ord ?order-id col ?ring-col&~RING_NONE))
  (not (wm-fact (key domain fact rs-ring-spec args? m ?rs r ?ring-col $?)))
  (goal (id ?root-id) (class INSTRUCTION-ROOT))
  =>
  (printout error "Can not build order " ?order-id " with ring-3 color " ?ring-col " because there is no ringstation for it" crlf)
)

(defrule goal-production-create-produce-for-order
  "Create for each incoming order a grounded production tree with the"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?root-id) (class INSTRUCTION-ROOT) (mode FORMULATED|DISPATCHED))
  (wm-fact (key config rcll pick-and-place-challenge) (value FALSE))
  (wm-fact (key domain fact order-complexity args? ord ?order-id com ?comp))
  (wm-fact (key domain fact order-base-color args? ord ?order-id col ?base-col))
  (wm-fact (key domain fact order-cap-color  args? ord ?order-id col ?cap-col))
  (wm-fact (key domain fact order-ring1-color args? ord ?order-id col ?ring-col1))
  (wm-fact (key domain fact order-ring2-color args? ord ?order-id col ?ring-col2))
  (wm-fact (key domain fact order-ring3-color args? ord ?order-id col ?ring-col3))
  (wm-fact (key domain fact cs-color args? m ?cs col ?cap-col))
  (wm-fact (key domain fact mps-type args? m ?cs t CS))
  (wm-fact (key domain fact mps-type args? m ?bs t BS))
  (not (wm-fact (key order meta wp-for-order args? wp ?something ord ?order-id)))
  (or (wm-fact (key domain fact order-ring1-color args? ord ?order-id col RING_NONE))
      (wm-fact (key domain fact rs-ring-spec args? m ?rs1 r ?ring-col1 $?)))
  (or (wm-fact (key domain fact order-ring2-color args? ord ?order-id col RING_NONE))
      (wm-fact (key domain fact rs-ring-spec args? m ?rs2 r ?ring-col2 $?)))
  (or (wm-fact (key domain fact order-ring3-color args? ord ?order-id col RING_NONE))
      (wm-fact (key domain fact rs-ring-spec args? m ?rs3 r ?ring-col3 $?)))

  (or
    (wm-fact (key strategy meta selected-order args? cond filter) (value ?order-id))
    (and
      (time $?now)
      (timer (name production-strategy-nothing-executable-timer) (time $?t&:(timeout ?now ?t ?*PRODUCTION-NOTHING-EXECUTABLE-TIMEOUT*)))
      (wm-fact (key strategy meta selected-order args? cond fallback) (value ?order-id))
    )
  )
  ?os <- (wm-fact (key order meta started args? ord ?order) (value FALSE))
  (wm-fact (key mps workload needs-update) (value FALSE))
  =>
  ;find the necessary ringstations
  (bind ?rs1 (goal-production-get-machine-for-color ?ring-col1))
  (bind ?rs2 (goal-production-get-machine-for-color ?ring-col2))
  (bind ?rs3 (goal-production-get-machine-for-color ?ring-col3))

  ;bind the ds to NONE - if there is none, or the actual DS - if there is one
  (bind ?ds NONE)
  (do-for-fact ((?do domain-object) (?df domain-fact))
    (and (eq ?do:type mps) (member$ ?do:name ?df:param-values) (member$ DS ?df:param-values))
    (bind ?ds ?do:name)
  )

  ;create facts for workpiece
  (bind ?wp-for-order (sym-cat wp- ?order-id))
  (assert (domain-object (name ?wp-for-order) (type workpiece))
      (domain-fact (name wp-unused) (param-values ?wp-for-order))
      (wm-fact (key domain fact wp-base-color args? wp ?wp-for-order col BASE_NONE) (type BOOL) (value TRUE))
      (wm-fact (key domain fact wp-cap-color args? wp ?wp-for-order col CAP_NONE) (type BOOL) (value TRUE))
      (wm-fact (key domain fact wp-ring1-color args? wp ?wp-for-order col RING_NONE) (type BOOL) (value TRUE))
      (wm-fact (key domain fact wp-ring2-color args? wp ?wp-for-order col RING_NONE) (type BOOL) (value TRUE))
      (wm-fact (key domain fact wp-ring3-color args? wp ?wp-for-order col RING_NONE) (type BOOL) (value TRUE))
      (wm-fact (key order meta wp-for-order args? wp ?wp-for-order ord ?order-id))
  )
  (if (eq ?comp C0)
    then
    (goal-production-assert-c0 ?root-id ?order-id ?wp-for-order ?cs ?ds ?bs ?cap-col ?base-col)
  )
  (if (and (eq ?comp C1) ?rs1)
    then
    (goal-production-assert-c1 ?root-id ?order-id ?wp-for-order ?cs ?ds ?bs ?rs1 ?cap-col ?base-col ?ring-col1)
  )
  (if (and (eq ?comp C2) ?rs1 ?rs2)
    then
    (goal-production-assert-c2 ?root-id ?order-id ?wp-for-order ?cs ?ds ?bs
                ?rs1 ?rs2 ?cap-col ?base-col ?ring-col1 ?ring-col2)
  )
  (if (and (eq ?comp C3) ?rs1 ?rs2 ?rs3)
    then
    (goal-production-assert-c3 ?root-id ?order-id ?wp-for-order ?cs ?ds ?bs
                ?rs1 ?rs2 ?rs3 ?cap-col ?base-col ?ring-col1 ?ring-col2 ?ring-col3)
  )

  ;clean-up needs update facts
  (delayed-do-for-all-facts
    ((?update-fact wm-fact)) (wm-key-prefix ?update-fact:key (create$ mps workload needs-update))
    (retract ?update-fact)
  )
  (assert (wm-fact (key mps workload needs-update) (is-list FALSE) (type BOOL) (value TRUE)))

  (modify ?os (value TRUE))

  ;if a timer exists, retract it to avoid goal creation spamming
  (delayed-do-for-all-facts
    ((?timer timer)) (eq ?timer:name production-strategy-nothing-executable-timer)
    (retract ?timer)
  )
)

(defrule goal-production-fill-in-unknown-wp-discard-from-cs
  "Fill in missing workpiece information into the discard goals from CS"
  ?g <- (goal (id ?goal-id) (class DISCARD) (mode FORMULATED) (parent ?parent)
              (params wp UNKNOWN wp-loc ?mps wp-side ?mps-side))
  ?i <- (goal (id ?i-goal-id) (class INSTRUCT-DS-DISCARD) (mode FORMULATED)
	            (params wp UNKNOWN target-mps ?ds))

  ; there is a wp at the machine
  (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side ?mps-side))
  (not (wm-fact (key order meta wp-for-order args? wp ?wp $?)))

  ; there is not another discard goal bound to this wp
  (not (goal (id ?other-goal-id) (class DISCARD) (outcome ~FAILED) (params wp ?wp wp-loc ?mps wp-side ?mps-side)))

  =>
  (modify ?g (params wp ?wp wp-loc ?mps wp-side ?mps-side))
  (modify ?i (params wp ?wp target-mps ?ds))
)

(defrule goal-production-remove-unused-discard
  "Remove an obsolete discard goal"
  ?g <- (goal (id ?goal-id) (class DISCARD) (mode FORMULATED) (parent ?parent)
              (params wp ?wp&:(neq ?wp UNKNOWN) wp-loc ?mps wp-side ?mps-side))
  (not (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side ?mps-side)))
  (not (wm-fact (key domain fact holding args? r ? wp ?wp)))
  =>
  (retract ?g)
)

(defrule goal-production-remove-unused-instruct-discard
  "Remove an obsolete discard instruct goal"
  ?g <- (goal (id ?i-goal-id) (class INSTRUCT-DS-DISCARD) (mode FORMULATED)
	            (params wp ?wp&:(neq ?wp UNKNOWN) target-mps ?ds))
  (not (wm-fact (key domain fact wp-at args? wp ?wp m ? side ?)))
  (not (wm-fact (key domain fact holding args? r ? wp ?wp)))
  =>
  (retract ?g)
)

(defrule goal-production-assert-enter-field-zones
  (not (wm-fact (key enter-field targets) (values $?zones)))
  (wm-fact (key refbox team-color) (value ?team-color&:(neq ?team-color nil)))
  =>
	(if (eq ?team-color CYAN) then
    (assert (wm-fact (key enter-field targets) (is-list TRUE) (type SYMBOL) (values C-Z33 C-Z23 C-Z43)))
  else
    (assert (wm-fact (key enter-field targets) (is-list TRUE) (type SYMBOL) (values M-Z33 M-Z23 M-Z43)))
  )
)


(defrule goal-production-create-enter-field
  "Enter the field (drive outside of the starting box)."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (wm-fact (key central agent robot args? r ?robot))
  (not (wm-fact (key central agent robot-lost args? r ?robot)))
  (not (wm-fact (key domain fact entered-field args? r ?robot)))
  (not
    (and
      (goal (id ?enter-field) (class ENTER-FIELD))
      (goal-meta (goal-id ?enter-field) (restricted-to ?robot))
    )
  )
  (domain-facts-loaded)
  (wm-fact (key refbox phase) (value PRODUCTION))
  ?targets <- (wm-fact (key enter-field targets) (values ?zone $?zones))
  =>
  (printout t "Goal " ENTER-FIELD " formulated" crlf)
  (goal-production-assert-enter-field ?robot ?zone)
)

(defrule goal-production-remove-enter-field
  "Enter the field (drive outside of the starting box)."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?gf <- (goal (id ?some-goal-id) (class ENTER-FIELD) (mode RETRACTED))
  ?gm <- (goal-meta (goal-id ?some-goal-id))
  =>
  (printout t "Goal " ENTER-FIELD " removed after entering" crlf)
  (retract ?gf ?gm)
)

; ----------------- Support Goals ---------------------------


(defrule goal-production-assert-buffer-goal
  "If there is no bufer goal yet, create one."
  (wm-fact (key domain fact cs-color args? m ?cs col ?cap-col))
  (goal (class INSTRUCTION-ROOT) (id ?instruct-root-id))
  (goal (class SUPPORT-ROOT) (id ?root-id))
  (not (goal (class BUFFER-CAP) (params $? ?cs $?) (mode ~RETRACTED)))
  =>
  (bind ?buffer-goal (goal-production-assert-buffer-cap ?cs ?cap-col nil))
  (bind ?instruct-goal (goal-production-assert-instruct-cs-buffer-cap ?cs ?cap-col nil))
  (modify ?buffer-goal (parent ?root-id))
  (modify ?instruct-goal (parent ?instruct-root-id))
)

(defrule goal-production-assert-pay-with-base-goal
  "Create 2 pay-with-base goals for each ring station"
  (wm-fact (key domain fact mps-type args? m ?bs t BS))
  (wm-fact (key domain fact mps-type args? m ?rs t RS))
  (goal (class SUPPORT-ROOT) (id ?root-id))
  (goal (class INSTRUCTION-ROOT) (id ?instruct-root-id))
  (not (and (goal (id ?some-id) (class PAY-FOR-RINGS-WITH-BASE) (params $? target-mps ?rs $?) (mode ~RETRACTED))
       (goal (id ?some-other-id&:(neq ?some-id ?some-other-id)) (class PAY-FOR-RINGS-WITH-BASE) (params $? target-mps ?rs $?) (mode ~RETRACTED))))
  =>
  (bind ?wp-base-pay (sym-cat BASE-PAY- (gensym*)))
  (bind ?payment-goal (goal-production-assert-pay-for-rings-with-base ?wp-base-pay ?rs nil))
  (bind ?instruct-goal (goal-production-assert-instruct-bs-dispense-base ?wp-base-pay (nth$ (random 1 3) (create$ BASE_RED BASE_BLACK BASE_SILVER)) INPUT nil ?bs))
  (assert
      (domain-object (name ?wp-base-pay) (type workpiece))
      (domain-fact (name wp-unused) (param-values ?wp-base-pay))
      (wm-fact (key domain fact wp-base-color args? wp ?wp-base-pay col BASE_NONE) (type BOOL) (value TRUE))
  )
  (modify ?payment-goal (parent ?root-id))
  (modify ?instruct-goal (parent ?instruct-root-id))
)

(defrule goal-production-assert-discard
  "Create a discard goal for each unused cap-carrier."
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact cs-color args? m ?cs col ?cap-col))
  (wm-fact (key domain fact wp-at args? wp ?wp m ?cs side OUTPUT))
  (not (wm-fact (key order meta wp-for-order args? wp ?wp $?)))
  (wm-fact (key domain fact mps-type args? m ?ds t DS))

  (wm-fact (key domain fact mps-team args? m ?ds col ?team-color))

  (goal (class SUPPORT-ROOT) (id ?root-id))
  (goal (class INSTRUCTION-ROOT) (id ?instruct-root-id))
  (not (goal (class DISCARD) (params $? ?cs $?) (mode ~RETRACTED)))
  =>
  (bind ?discard-goal (goal-production-assert-discard ?wp ?ds nil))
  (bind ?instruct-goal (goal-production-assert-instruct-ds-discard ?wp ?ds))
  (do-for-all-facts ((?mtype domain-fact)) (and (eq ?mtype:name mps-type) (member$ RS ?mtype:param-values))
    (bind ?pay-goal-fact (goal-production-assert-pay-for-rings-with-base ?wp (nth$ 1 ?mtype:param-values) nil))
    (modify ?pay-goal-fact (parent ?root-id) (priority (float (+ ?*PRODUCTION-PAY-PRIORITY* ?*PRODUCTION-PAY-CC-PRIORITY-INCREASE*))))
  )
  (modify ?discard-goal (parent ?root-id))
  (modify ?instruct-goal (parent ?instruct-root-id))
)

(defrule goal-production-remove-goal-for-unavailable-workpiece
  " Clean up payment goals that operate on workpieces that can't be used at all."
  (domain-object (name ?wp))
  ?g <- (goal (id ?goal-id) (class PAY-FOR-RINGS-WITH-BASE) (params $? wp ?wp $?) (mode FORMULATED))
  ?gm <- (goal-meta (goal-id ?goal-id))
  (not (domain-fact (name wp-usable) (param-values ?wp)))
  (not (domain-fact (name wp-unused) (param-values ?wp)))
  =>
  (retract ?g ?gm)
)
