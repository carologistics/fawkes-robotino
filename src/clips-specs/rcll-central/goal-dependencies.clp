;---------------------------------------------------------------------------
;  goal-dependencies.clp - Defines dependency-assignments and grounds
;                          them for executability
;
;  Created: Sat 22 May 2021 13:07:31 CET
;  Copyright  2021  Matteo Tschesche <matteo.tschesche@rwth-aachen.de>
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

(deftemplate dependency-assignment
	; id of goal that might rely on another goal/dependence-goal
	(slot goal-id (type SYMBOL))

	; goal class of dependency-goal
	; in case of CLEAR-OUTPUT: DELIVER or DISCARD
	(slot class (type SYMBOL))

	; defines if goal waits for dependency before wp-get (wait-for-wp),
	; before wp-put (wait-for-free-side), or not at all (nil)
	; set when asserting
	(slot wait-for (type SYMBOL) (allowed-values nil WP FREE-SIDE))

	; necessary parameters used for goal-expander of dependency-goal,
	; set in execution-check
	; for deliver-mount-cap:    wp, wp-loc, wp-side
	; for mount-cap-mount-ring: wp, wp-loc, wp-side
	; for discard-buffer-cap:   wp, wp-loc, wp-side
	(multislot params (type SYMBOL))

	; id of dependence-goal, nil if ungrounded
	(slot grounded-with (type SYMBOL))

	;(slot priority (type float) (default 0.0))
)

; ---------------------------- Create Dependencies ----------------------------
; A goal depends on a class of a dependency-goal if such a dependency-goal can
; be required for executing this goal

(defrule goal-dependencies-mount-cap-buffer-cap
" Every mount-cap goal depends on the buffer-cap class.
  Per default, no buffer-cap goal is grounded. "
	; needs to be higher than SALIENCE-GOAL-EXECUTABLE-CHECK
	(declare (salience (+ ?*SALIENCE-GOAL-EXECUTABLE-CHECK* 1)))
	?g <- (goal (id ?goal-id) (class MOUNT-CAP) (mode FORMULATED))
	(not (dependency-assignment (goal-id ?goal-id)
	                            (class BUFFER-CAP)))
	(not (dependency-assignment (goal-id ?goal-id)
	                            (class INSTRUCT-CS-BUFFER-CAP)))
	=>
	(printout t "Goal " ?goal-id
	            " depends on class BUFFER-CAP " crlf)
	(assert (dependency-assignment (goal-id ?goal-id)
	                               (class BUFFER-CAP)
	                               (wait-for FREE-SIDE)
	                               (grounded-with nil)))
	(printout t "Goal " ?goal-id
	            " depends on class INSTRUCT-CS-BUFFER-CAP " crlf)
	(assert (dependency-assignment (goal-id ?goal-id)
	                               (class INSTRUCT-CS-BUFFER-CAP)
	                               (wait-for FREE-SIDE)
	                               (grounded-with nil)))
)

(defrule goal-dependencies-mount-cap-clear-output
" A Mount-cap goal can also depend on the deliver of discard
  class to clear the output.
  Per default, no buffer-cap goal is grounded. "
	; needs to be higher than SALIENCE-GOAL-EXECUTABLE-CHECK
	(declare (salience (+ ?*SALIENCE-GOAL-EXECUTABLE-CHECK* 1)))
	?g <- (goal (id ?goal-id) (class MOUNT-CAP) (mode FORMULATED))
	(not (dependency-assignment (goal-id ?goal-id)
	                            (class CLEAR-OUTPUT)))
	=>
	(printout t "Goal " ?goal-id
	            " depends on class CLEAR-OUTPUT " crlf)
	(assert (dependency-assignment (goal-id ?goal-id)
	                               (class CLEAR-OUTPUT)
	                               (grounded-with nil)))
)

(defrule goal-dependencies-deliver-mount-cap
" Every deliver goal depends on the mount-cap class.
  Per default, no mount-cap goal is grounded. "
	; needs to be higher than SALIENCE-GOAL-EXECUTABLE-CHECK
	(declare (salience (+ ?*SALIENCE-GOAL-EXECUTABLE-CHECK* 1)))
	?g <- (goal (id ?goal-id) (class DELIVER|DELIVER-RC21) (mode FORMULATED))
	(not (dependency-assignment (goal-id ?goal-id)
	                            (class MOUNT-CAP)))
	(not (dependency-assignment (goal-id ?goal-id)
	                            (class INSTRUCT-CS-MOUNT-CAP)))
	=>
	(printout t "Goal " ?goal-id
	            " depends on class MOUNT-CAP " crlf)
	(assert (dependency-assignment (goal-id ?goal-id)
	                               (class MOUNT-CAP)
	                               (wait-for WP)
	                               (grounded-with nil)))
	(printout t "Goal " ?goal-id
	            " depends on class INSTRUCT-CS-MOUNT-CAP " crlf)
	(assert (dependency-assignment (goal-id ?goal-id)
	                               (class INSTRUCT-CS-MOUNT-CAP)
	                               (wait-for WP)
	                               (grounded-with nil)))
)

(defrule goal-dependencies-discard-buffer-cap
" Every discard goal depends on the buffer-cap class.
  Per default, no buffer-cap goal is grounded. "
	; needs to be higher than SALIENCE-GOAL-EXECUTABLE-CHECK
	(declare (salience (+ ?*SALIENCE-GOAL-EXECUTABLE-CHECK* 1)))
	?g <- (goal (id ?goal-id) (class DISCARD) (mode FORMULATED))
	(not (dependency-assignment (goal-id ?goal-id)
	                            (class BUFFER-CAP)))
	(not (dependency-assignment (goal-id ?goal-id)
	                            (class INSTRUCT-CS-BUFFER-CAP)))
	=>
	(printout t "Goal " ?goal-id
	            " depends on class BUFFER-CAP " crlf)
	(assert (dependency-assignment (goal-id ?goal-id)
	                               (class BUFFER-CAP)
	                               (wait-for WP)
	                               (grounded-with nil)))
	(printout t "Goal " ?goal-id
	            " depends on class INSTRUCT-CS-BUFFER-CAP " crlf)
	(assert (dependency-assignment (goal-id ?goal-id)
	                               (class INSTRUCT-CS-BUFFER-CAP)
	                               (wait-for WP)
	                               (grounded-with nil)))
)

(defrule goal-dependencies-pay-with-cc-buffer-cap
" Every discard goal depends on the buffer-cap class.
  Per default, no buffer-cap goal is grounded. "
	; needs to be higher than SALIENCE-GOAL-EXECUTABLE-CHECK
	(declare (salience (+ ?*SALIENCE-GOAL-EXECUTABLE-CHECK* 1)))
	?g <- (goal (id ?goal-id)
	            (class PAY-FOR-RINGS-WITH-CAP-CARRIER)
	            (mode FORMULATED))
	(not (dependency-assignment (goal-id ?goal-id)
	                            (class BUFFER-CAP)))
	(not (dependency-assignment (goal-id ?goal-id)
	                            (class INSTRUCT-CS-BUFFER-CAP)))
	=>
	(printout t "Goal " ?goal-id
	            " depends on class BUFFER-CAP " crlf)
	(assert (dependency-assignment (goal-id ?goal-id)
	                               (class BUFFER-CAP)
	                               (wait-for WP)
	                               (grounded-with nil)))
	(printout t "Goal " ?goal-id
	            " depends on class INSTRUCT-CS-BUFFER-CAP " crlf)
	(assert (dependency-assignment (goal-id ?goal-id)
	                               (class INSTRUCT-CS-BUFFER-CAP)
	                               (wait-for WP)
	                               (grounded-with nil)))
)

(defrule goal-dependencies-mount-cap-mount-ring
" Every mount-cap goal depends on the mount-ring class.
  Per default, no mount-ring goal is grounded. "
	; needs to be higher than SALIENCE-GOAL-EXECUTABLE-CHECK
	(declare (salience (+ ?*SALIENCE-GOAL-EXECUTABLE-CHECK* 1)))
	?g <- (goal (id ?goal-id) (class MOUNT-CAP) (mode FORMULATED))
	(not (dependency-assignment (goal-id ?goal-id)
	                            (class MOUNT-RING)))
	(not (dependency-assignment (goal-id ?goal-id)
	                            (class INSTRUCT-RS-MOUNT-RING)))
	(not (dependency-assignment (goal-id ?goal-id)
	                            (class PAYMENT-1)))
	(not (dependency-assignment (goal-id ?goal-id)
	                            (class PAYMENT-2)))
	=>
	(printout t "Goal " ?goal-id
	            " depends on class MOUNT-RING " crlf)
	(assert (dependency-assignment (goal-id ?goal-id)
	                               (class MOUNT-RING)
	                               (wait-for WP)
	                               (grounded-with nil)))
	(printout t "Goal " ?goal-id
	            " depends on class INSTRUCT-RS-MOUNT-RING " crlf)
	(assert (dependency-assignment (goal-id ?goal-id)
	                               (class INSTRUCT-RS-MOUNT-RING)
	                               (wait-for WP)
	                               (grounded-with nil)))
	(printout t "Goal " ?goal-id
	            " depends on class PAYMENT-1 " crlf)
	(assert (dependency-assignment (goal-id ?goal-id)
	                               (class PAYMENT-1)
	                               (wait-for WP)
	                               (grounded-with nil)))
	(printout t "Goal " ?goal-id
	            " depends on class PAYMENT-2 " crlf)
	(assert (dependency-assignment (goal-id ?goal-id)
	                               (class PAYMENT-2)
	                               (wait-for WP)
	                               (grounded-with nil)))
)

(defrule goal-dependencies-mount-ring-mount-ring
" Every mount-ring goal depends on the mount-ring class.
  Per default, no mount-ring goal is grounded. "
	; needs to be higher than SALIENCE-GOAL-EXECUTABLE-CHECK
	(declare (salience (+ ?*SALIENCE-GOAL-EXECUTABLE-CHECK* 1)))
	?g <- (goal (id ?goal-id) (class MOUNT-RING) (mode FORMULATED))
	(not (dependency-assignment (goal-id ?goal-id)
	                            (class MOUNT-RING)))
	(not (dependency-assignment (goal-id ?goal-id)
	                            (class INSTRUCT-RS-MOUNT-RING)))
	(not (dependency-assignment (goal-id ?goal-id)
	                            (class PAYMENT-1)))
	(not (dependency-assignment (goal-id ?goal-id)
	                            (class PAYMENT-2)))
	=>
	(printout t "Goal " ?goal-id
	            " depends on class MOUNT-RING " crlf)
	(assert (dependency-assignment (goal-id ?goal-id)
	                               (class MOUNT-RING)
	                               (wait-for WP)
	                               (grounded-with nil)))
	(printout t "Goal " ?goal-id
	            " depends on class INSTRUCT-RS-MOUNT-RING " crlf)
	(assert (dependency-assignment (goal-id ?goal-id)
	                               (class INSTRUCT-RS-MOUNT-RING)
	                               (wait-for WP)
	                               (grounded-with nil)))
	(printout t "Goal " ?goal-id
	            " depends on class PAYMENT-1 " crlf)
	(assert (dependency-assignment (goal-id ?goal-id)
	                               (class PAYMENT-1)
	                               (wait-for WP)
	                               (grounded-with nil)))
	(printout t "Goal " ?goal-id
	            " depends on class PAYMENT-2 " crlf)
	(assert (dependency-assignment (goal-id ?goal-id)
	                               (class PAYMENT-2)
	                               (wait-for WP)
	                               (grounded-with nil)))
)

; ---------------------------- Executability Check ---------------------------
; ----------------------------- under Dependencies ---------------------------

(defrule goal-dependencies-mount-cap-buffer-cap-executable
" Even if the CS is not buffered, mount-cap can be executable if the output is
  free and there is a buffer-cap goal running for the CS. "
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class MOUNT-CAP)
	                          (mode FORMULATED)
	                          (params  wp ?wp
	                                   target-mps ?target-mps
	                                   target-side ?target-side
	                                   $?)
	                          (is-executable FALSE))

	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil) (order-id ?order-id))
	; Robot CEs
	(wm-fact (key central agent robot args? r ?robot))
	(wm-fact (key refbox team-color) (value ?team-color))

	; MPS-CS CEs
	(wm-fact (key domain fact mps-type args? m ?target-mps t CS))
	(wm-fact (key domain fact mps-state args? m ?target-mps s ~BROKEN))
	(wm-fact (key domain fact mps-team args? m ?target-mps col ?team-color))

	(not (wm-fact (key domain fact cs-buffered args? m ?target-mps col ?)))
	(wm-fact (key domain fact mps-side-free args? m ?target-mps side INPUT))
	(wm-fact (key domain fact mps-side-free args? m ?target-mps side OUTPUT))

	; CS is not buffered, but with the following dependency, we can assume it
	; will soon: A feasible (same parameter) buffer-cap or
	; instruct-cs-buffer-cap goal is running...
	(goal (id ?dependency-goal-id)
	      (class ?dependency-class&BUFFER-CAP|INSTRUCT-CS-BUFFER-CAP)
	      (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
	      (params target-mps ?target-mps $?))
	(goal-meta (goal-id ?dependency-goal-id) (order-id ?order-id))

	; depend on buffer-cap and instruct-buffer-cap goal:
	(goal (id ?buffer-goal-id) (class BUFFER-CAP))
	(goal-meta (goal-id ?buffer-goal-id) (order-id ?order-id))
	?buffer-da <- (dependency-assignment (goal-id ?goal-id) (class BUFFER-CAP))

	(goal (id ?instruct-goal-id) (class INSTRUCT-CS-BUFFER-CAP))
	(goal-meta (goal-id ?instruct-goal-id) (order-id ?order-id))
	?instruct-da <- (dependency-assignment (goal-id ?goal-id) (class INSTRUCT-CS-BUFFER-CAP))

	; WP CEs
	(wm-fact (key wp meta next-step args? wp ?wp) (value CAP))
	; MPS-Source CEs
	(wm-fact (key domain fact mps-type args? m ?wp-loc t ?))
	(wm-fact (key domain fact mps-team args? m ?wp-loc col ?team-color))

	(or (and ; Either the workpiece needs to be picked up...
	         (not (wm-fact (key domain fact holding args?
	                                        r ?robot
	                                        wp ?any-wp)))
	           ; ... and it is a fresh base located in a base station
	         (or (and (wm-fact (key domain fact mps-type args? m ?wp-loc t BS))
	                  (wm-fact (key domain fact wp-unused args? wp ?wp))
	                  (wm-fact (key domain fact wp-base-color args?
	                                            wp ?wp
	                                            col BASE_NONE)))
	               ; ... or is already at some machine
	             (wm-fact (key domain fact wp-at args?
	                                       wp ?wp
	                                       m ?wp-loc
	                                       side ?wp-side))
	         )
	    )
	      ; or the workpiece is already being held
	    (wm-fact (key domain fact holding args? r ?robot wp ?wp))
	)
	(domain-fact (name zone-content) (param-values ?zz1 ?target-mps))
	(domain-fact (name zone-content) (param-values ?zz2 ?wp-loc))
	=>
	(printout t "Goal " ?goal-id " executable for " ?robot
	            " depending on goal " ?buffer-goal-id
	            " and " ?instruct-goal-id crlf)
	(modify ?g (is-executable TRUE))
	(modify ?buffer-da (grounded-with ?buffer-goal-id))
	(modify ?instruct-da (grounded-with ?instruct-goal-id))
)

(defrule goal-dependencies-mount-cap-buffer-cap-output-blocked-executable
" Even if the CS is not buffered and a wp is blocking its output, mount-cap can
  be executable if there is a running deliver goal for the blocking wp and
  a running buffer-cap goal for the CS. "
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class MOUNT-CAP)
	                          (mode FORMULATED)
	                          (params  wp ?wp
	                                   target-mps ?target-mps
	                                   target-side ?target-side
	                                   $?)
	                          (is-executable FALSE))

	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil) (order-id ?order-id))
	; Robot CEs
	(wm-fact (key central agent robot args? r ?robot))
	(wm-fact (key refbox team-color) (value ?team-color))

	; MPS-CS CEs
	(wm-fact (key domain fact mps-type args? m ?target-mps t CS))
	(wm-fact (key domain fact mps-state args? m ?target-mps s ~BROKEN))
	(wm-fact (key domain fact mps-side-free args? m ?target-mps side INPUT))
	(wm-fact (key domain fact mps-team args? m ?target-mps col ?team-color))

	(not (wm-fact (key domain fact cs-buffered args?
	                               m ?target-mps
	                               col ?cap-color)))

	(wm-fact (key domain fact wp-at args?
	                          wp ?blocking-wp
	                          m ?target-mps
	                          side OUTPUT))
	; CS output is not free, but with this goal executing, we can assume it
	; will soon
	(goal (id ?clear-goal-id)
	      (class ?clear-class&DELIVER|DISCARD)
	      (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
	      (params  wp ?blocking-wp $?))
	?clear-da <- (dependency-assignment (goal-id ?clear-goal-id) (class CLEAR-OUTPUT))

	; CS is not buffered, but with the following dependency, we can assume it
	; will soon: A feasible (same parameter) buffer-cap or
	; instruct-cs-buffer-cap goal is running...
	(goal (id ?dependency-goal-id)
	      (class ?dependency-class&BUFFER-CAP|INSTRUCT-CS-BUFFER-CAP)
	      (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
	      (params target-mps ?target-mps $?))
	(goal-meta (goal-id ?dependency-goal-id) (order-id ?order-id))

	; depend on buffer-cap and instruct-buffer-cap goal:
	(goal (id ?buffer-goal-id) (class BUFFER-CAP))
	(goal-meta (goal-id ?buffer-goal-id) (order-id ?order-id))
	?buffer-da <- (dependency-assignment (goal-id ?goal-id) (class BUFFER-CAP))

	(goal (id ?instruct-goal-id) (class INSTRUCT-CS-BUFFER-CAP))
	(goal-meta (goal-id ?instruct-goal-id) (order-id ?order-id))
	?instruct-da <- (dependency-assignment (goal-id ?goal-id) (class INSTRUCT-CS-BUFFER-CAP))

	; WP CEs
	(wm-fact (key wp meta next-step args? wp ?wp) (value CAP))
	; MPS-Source CEs
	(wm-fact (key domain fact mps-type args? m ?wp-loc t ?))
	(wm-fact (key domain fact mps-team args? m ?wp-loc col ?team-color))

	(or (and ; Either the workpiece needs to be picked up...
	         (not (wm-fact (key domain fact holding args?
	                                        r ?robot
	                                        wp ?)))
	         ; ... and it is a fresh base located in a base station
	         (or (and (wm-fact (key domain fact mps-type args? m ?wp-loc t BS))
	                  (wm-fact (key domain fact wp-unused args? wp ?wp))
	                  (wm-fact (key domain fact wp-base-color
	                            args? wp ?wp col BASE_NONE)))
	             ; ... or is already at some machine
	             (wm-fact (key domain fact wp-at
	                       args? wp ?wp m ?wp-loc side ?wp-side))
	         )
	    )
	    ; or the workpiece is already being held
	    (wm-fact (key domain fact holding args? r ?robot wp ?wp))
	)
	(domain-fact (name zone-content) (param-values ?zz1 ?target-mps))
	(domain-fact (name zone-content) (param-values ?zz2 ?wp-loc))
	=>
	(printout t "Goal " ?goal-id " executable for " ?robot
	            " depending on goal " ?buffer-goal-id
	            ", " ?instruct-goal-id
	            " and " ?clear-goal-id " to clear output" crlf)
	(modify ?g (is-executable TRUE))
	(modify ?buffer-da (grounded-with ?buffer-goal-id))
	(modify ?instruct-da (grounded-with ?instruct-goal-id))
	(modify ?clear-da (grounded-with ?clear-goal-id))
)

(defrule goal-dependencies-deliver-mount-cap-executable
" Even if mount-cap is still executing, deliver can already be executable
  if the CS is buffered and its output is free. "
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class DELIVER)
	                          (mode FORMULATED)
	                          (params  wp ?unknown-wp
	                                   target-mps ?target-mps
	                                   target-side ?target-side
	                                   $?)
	                          (is-executable FALSE))

	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil) (order-id ?order-id))
	; Robot CEs
	(wm-fact (key central agent robot args? r ?robot))
	(wm-fact (key refbox team-color) (value ?team-color))

	; MPS-CS CEs
	(wm-fact (key domain fact mps-type args? m ?target-mps t DS))
	(wm-fact (key domain fact mps-state args? m ?target-mps s ~BROKEN))
	(not (wm-fact (key domain fact wp-at args?
	                               wp ?any-wp
	                               m ?target-mps
	                               side INPUT)))
	(wm-fact (key domain fact mps-team args? m ?target-mps col ?team-color))

	; wp is not at CS OUTPUT, but after these goals finished, it will be
	(goal (id ?mount-goal-id)
	      (class MOUNT-CAP)
	      (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
	      (params wp ?wp
	              target-mps ?mps
	              $?))
	(goal-meta (goal-id ?mount-goal-id) (order-id ?order-id))
	?mount-da <- (dependency-assignment (goal-id ?goal-id) (class MOUNT-CAP))

	(wm-fact (key domain fact cs-buffered args? m ?mps col ?cap-color))
	(wm-fact (key domain fact mps-side-free args? m ?mps side OUTPUT))

	; get instruct mount-cap goal
	(goal (id ?instruct-goal-id) (class INSTRUCT-CS-MOUNT-CAP))
	(goal-meta (goal-id ?instruct-goal-id) (order-id ?order-id))
	?instruct-da <- (dependency-assignment (goal-id ?goal-id)
	                                       (class INSTRUCT-CS-MOUNT-CAP))

	(not (wm-fact (key domain fact holding args? r ?robot wp ?some-wp)))
	(domain-fact (name zone-content) (param-values ?zz1 ?target-mps))
	(domain-fact (name zone-content) (param-values ?zz2 ?wp-loc))
	=>
	(printout t "Goal " ?goal-id " executable for " ?robot
	            " depending on goal " ?mount-goal-id
	            " and goal " ?instruct-goal-id crlf)
	(modify ?g (is-executable TRUE))
	(modify ?mount-da (params  wp ?wp
	                           wp-loc ?mps
	                           wp-side OUTPUT)
	                  (grounded-with ?mount-goal-id))
	(modify ?instruct-da (grounded-with ?instruct-goal-id))
)

(defrule goal-dependencies-deliver-rc21-mount-cap-executable
" Even if mount-cap is still executing, deliver-rc21 can already be executable
  if the CS is buffered and its output is free. "
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class DELIVER-RC21)
	                          (mode FORMULATED)
	                          (is-executable FALSE))

	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil) (order-id ?order-id))
	; Robot CEs
	(wm-fact (key central agent robot args? r ?robot))
	(wm-fact (key refbox team-color) (value ?team-color))

	; wp is not at CS OUTPUT, but after these goals finished, it will be
	(goal (id ?mount-goal-id)
	      (class MOUNT-CAP)
	      (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
	      (params wp ?wp
	              target-mps ?mps
	              $?))
	?mount-da <- (dependency-assignment (goal-id ?goal-id) (class MOUNT-CAP))

	(wm-fact (key domain fact cs-buffered args? m ?mps col ?cap-color))
	(wm-fact (key domain fact mps-side-free args? m ?mps side OUTPUT))

	; get instruct mount-cap goal
	(goal (id ?instruct-goal-id) (class INSTRUCT-CS-MOUNT-CAP))
	(goal-meta (goal-id ?instruct-goal-id) (order-id ?order-id))
	?instruct-da <- (dependency-assignment (goal-id ?goal-id)
	                                       (class INSTRUCT-CS-MOUNT-CAP))

	(not (wm-fact (key domain fact holding args? r ?robot wp ?some-wp)))
	(domain-fact (name zone-content) (param-values ?zz ?wp-loc))
	=>
	(printout t "Goal " ?goal-id " executable for " ?robot
	            " depending on goal " ?mount-goal-id
	            " and goal " ?instruct-goal-id crlf)
	(modify ?g (is-executable TRUE))
	(modify ?mount-da (params  wp ?wp
	                           wp-loc ?mps
	                           wp-side OUTPUT)
	                  (grounded-with ?mount-goal-id))
	(modify ?instruct-da (grounded-with ?instruct-goal-id))
)

(defrule goal-dependencies-mount-cap-mount-ring-executable
" Even if mount-ring is still executing, mount-cap can already be executable
  if the CS is buffered and all payments are about to be done. "
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class MOUNT-CAP)
	                          (mode FORMULATED)
	                          (params  wp ?wp
	                                   target-mps ?cs
	                                   target-side ?cs-side
	                                   $?)
	                          (is-executable FALSE))

	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil) (order-id ?order-id))
	; Robot CEs
	(wm-fact (key central agent robot args? r ?robot))
	(wm-fact (key refbox team-color) (value ?team-color))

	; wp is not at RS OUTPUT, but after these goals finished, it will be
	(goal (id ?mount-goal-id)
	      (class MOUNT-RING)
	      (params  wp ?wp
	               target-mps ?rs
	               $?
	               ring-color ?ring-color
	               $?))
	(or (goal (id ?mount-goal-id) (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED))
	    (and (goal (id ?mount-goal-id) (outcome COMPLETED))
	         (wm-fact (key domain fact wp-at args? wp ?wp m ?rs side INPUT))
	    )
	)
	(goal-meta (goal-id ?mount-goal-id) (order-id ?order-id) (ring-nr ?ring-nr))

	(wm-fact (key domain fact order-complexity args?
	                          ord ?order-id
	                          com ?complexity&C1|C2|C3))
	(or (test (eq ?complexity C1))
	    (and (test (eq ?complexity C2))
	         (test (eq ?ring-nr TWO)))
	    (and (test (eq ?complexity C3))
	         (test (eq ?ring-nr THREE))))
	?mount-da <- (dependency-assignment (goal-id ?goal-id) (class MOUNT-RING))

	(wm-fact (key domain fact cs-buffered args? m ?cs col ?cap-color))

	; get instruct mount-ring goal
	(goal (id ?instruct-goal-id) (class INSTRUCT-RS-MOUNT-RING))
	(goal-meta (goal-id ?instruct-goal-id) (order-id ?order-id) (ring-nr ?ring-nr))
	?instruct-da <- (dependency-assignment (goal-id ?goal-id) (class INSTRUCT-RS-MOUNT-RING))

	(not (wm-fact (key domain fact holding args? r ?robot wp ?some-wp)))

	; payments are or will be enough to mount-ring
	(wm-fact (key domain fact rs-filled-with args? m ?rs n ?bases-filled))
	(wm-fact (key domain fact rs-ring-spec args?
	                          m ?rs
	                          r ?ring-color
	                          rn ?bases-needed))

	; forward declaration, bind to anything
	(goal (id ?feed-id-1))
	(goal (id ?feed-id-2))

	(or ;filled >= needed payments
	    (and (wm-fact (key domain fact rs-sub args? minuend ?bases-filled
	                                           subtrahend ?bases-needed
	                                           difference ?bases-remaining&ZERO|ONE|TWO|THREE))
	         ; forward declaration, bind ?bases-missing to ZERO
	         (wm-fact (key domain fact rs-sub args? minuend ZERO
	                                                subtrahend ZERO
	                                                difference ?bases-missing)))
	    ;or filled + depending fill goals >= needed payments
	    (and (wm-fact (key domain fact rs-sub args? minuend ?bases-needed
	                                                subtrahend ?bases-filled
	                                                difference ?bases-missing&ZERO|ONE|TWO))
	         (or (test (eq ?bases-missing ZERO))
	             (and (test (> (sym-to-int ?bases-missing) 0))
	                  (goal (id ?feed-id-1)
	                        (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
	                        (class PAY-FOR-RINGS-WITH-BASE|
	                               PAY-FOR-RINGS-WITH-CAP-CARRIER|
	                               PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF)
	                        (params $? ?rs $?))
	                  (goal-meta (goal-id ?feed-id-1) (order-id ?order-id))
	             ))
	         (or (test (< (sym-to-int ?bases-missing) 2))
	             (and (test (eq ?bases-missing TWO))
	                  (goal (id ?feed-id-2)
	                        (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
	                        (class PAY-FOR-RINGS-WITH-BASE|
	                               PAY-FOR-RINGS-WITH-CAP-CARRIER|
	                               PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF)
	                        (params $? ?rs $?))
	                  (goal-meta (goal-id ?feed-id-2&:(neq ?feed-id-2 ?feed-id-1)) (order-id ?order-id))
	             )
	         )
	    )
	)
	(domain-fact (name zone-content) (param-values ?zz1 ?target-mps))
	(domain-fact (name zone-content) (param-values ?zz2 ?wp-loc))
	=>
	(printout t "Goal " ?goal-id " executable for " ?robot
	            " depending on goal " ?mount-goal-id
	            " and goal " ?instruct-goal-id crlf)
	(modify ?g (is-executable TRUE))
	(modify ?mount-da (params  wp ?wp
	                           wp-loc ?rs
	                           wp-side OUTPUT)
	                  (grounded-with ?mount-goal-id))
	(modify ?instruct-da (grounded-with ?instruct-goal-id))

	(if (> (sym-to-int ?bases-missing) 0) then
		(do-for-fact ((?pay-da-1 dependency-assignment))
		              (and (eq ?pay-da-1:class PAYMENT-1)
		                   (eq ?pay-da-1:goal-id ?goal-id))
		              (modify ?pay-da-1 (grounded-with ?feed-id-1)))
		(printout t "Goal " ?goal-id " executable for " ?robot
		            " also depending on goal " ?feed-id-1 crlf))
	(if (eq ?bases-missing TWO) then
		(do-for-fact ((?pay-da-1 dependency-assignment))
		              (and (eq ?pay-da-1:class PAYMENT-1)
		                   (eq ?pay-da-1:goal-id ?goal-id))
		              (modify ?pay-da-1 (grounded-with ?feed-id-1)))
		(printout t "Goal " ?goal-id " executable for " ?robot
		            " also depending on goal " ?feed-id-2 crlf))
)

(defrule goal-dependencies-mount-ring-mount-ring-executable
" Even if mount-ring is still executing, mount-ring can already be executable
  if all payments are about to be done. "
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class MOUNT-RING)
	                          (mode FORMULATED)
	                          (params  wp ?wp
	                                   target-mps ?other-rs
	                                   $?)
	                          (is-executable FALSE))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil) (order-id ?order-id) (ring-nr ?ring-nr))

	; Robot CEs
	(wm-fact (key central agent robot args? r ?robot))
	(wm-fact (key refbox team-color) (value ?team-color))

	; wp is not at RS OUTPUT, but after these goals finished, it will be
	(goal (id ?mount-goal-id)
	      (class MOUNT-RING)
	      (params  wp ?wp
	               target-mps ?rs
	               $?
	               ring-color ?ring-color
	               $?))
	(or (goal (id ?mount-goal-id) (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED))
	    (and (goal (id ?mount-goal-id) (outcome COMPLETED))
	         (wm-fact (key domain fact wp-at args? wp ?wp m ?rs side INPUT))
	    )
	)
	(goal-meta (goal-id ?mount-goal-id) (order-id ?order-id) (ring-nr ?mount-ring-nr))
	(wm-fact (key domain fact rs-sub args? minuend ?ring-nr
	                                       subtrahend ?mount-ring-nr
	                                       difference ONE))
	?mount-da <- (dependency-assignment (goal-id ?goal-id) (class MOUNT-RING))

	; get instruct mount-ring goal
	(goal (id ?instruct-goal-id) (class INSTRUCT-RS-MOUNT-RING))
	(goal-meta (goal-id ?instruct-goal-id) (order-id ?order-id) (ring-nr ?mount-ring-nr))
	?instruct-da <- (dependency-assignment (goal-id ?goal-id) (class INSTRUCT-RS-MOUNT-RING))

	(not (wm-fact (key domain fact holding args? r ?robot wp ?some-wp)))

	; payments are or will be enough to mount-ring
	(wm-fact (key domain fact rs-filled-with args? m ?rs n ?bases-filled))
	(wm-fact (key domain fact rs-ring-spec args?
	                          m ?rs
	                          r ?ring-color
	                          rn ?bases-needed))

	; forward declaration, bind to anything
	(goal (id ?feed-id-1))
	(goal (id ?feed-id-2))

	(or ;filled >= needed payments
	    (and (wm-fact (key domain fact rs-sub args? minuend ?bases-filled
	                                           subtrahend ?bases-needed
	                                           difference ?bases-remaining&ZERO|ONE|TWO|THREE))
	         ; forward declaration, bind ?bases-missing to ZERO
	         (wm-fact (key domain fact rs-sub args? minuend ZERO
	                                                subtrahend ZERO
	                                                difference ?bases-missing)))
	    ;or filled + depending fill goals >= needed payments
	    (and (wm-fact (key domain fact rs-sub args? minuend ?bases-needed
	                                                subtrahend ?bases-filled
	                                                difference ?bases-missing&ZERO|ONE|TWO))
	         (or (test (eq ?bases-missing ZERO))
	             (and (test (> (sym-to-int ?bases-missing) 0))
	                  (goal (id ?feed-id-1)
	                        (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
	                        (class PAY-FOR-RINGS-WITH-BASE|
	                               PAY-FOR-RINGS-WITH-CAP-CARRIER|
	                               PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF)
	                        (params $? ?rs $?))
	                  (goal-meta (goal-id ?feed-id-1) (order-id ?order-id))
	             ))
	         (or (test (< (sym-to-int ?bases-missing) 2))
	             (and (test (eq ?bases-missing TWO))
	                  (goal (id ?feed-id-2)
	                        (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
	                        (class PAY-FOR-RINGS-WITH-BASE|
	                               PAY-FOR-RINGS-WITH-CAP-CARRIER|
	                               PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF)
	                        (params $? ?rs $?))
	                  (goal-meta (goal-id ?feed-id-2&:(neq ?feed-id-2 ?feed-id-1)) (order-id ?order-id))
	             )
	         )
	    )
	)
	(domain-fact (name zone-content) (param-values ?zz1 ?target-mps))
	(domain-fact (name zone-content) (param-values ?zz2 ?wp-loc))
	=>
	(printout t "Goal " ?goal-id " executable for " ?robot
	            " depending on goal " ?mount-goal-id
	            " and goal " ?instruct-goal-id crlf)
	(modify ?g (is-executable TRUE))
	(modify ?mount-da (params  wp ?wp
	                           wp-loc ?rs
	                           wp-side OUTPUT)
	                  (grounded-with ?mount-goal-id))
	(modify ?instruct-da (grounded-with ?instruct-goal-id))

	(if (> (sym-to-int ?bases-missing) 0) then
		(do-for-fact ((?pay-da-1 dependency-assignment))
		              (and (eq ?pay-da-1:class PAYMENT-1)
		                   (eq ?pay-da-1:goal-id ?goal-id))
		              (modify ?pay-da-1 (grounded-with ?feed-id-1)))
		(printout t "Goal " ?goal-id " executable for " ?robot
		            " also depending on goal " ?feed-id-1 crlf))
	(if (eq ?bases-missing TWO) then
		(do-for-fact ((?pay-da-1 dependency-assignment))
		              (and (eq ?pay-da-1:class PAYMENT-1)
		                   (eq ?pay-da-1:goal-id ?goal-id))
		              (modify ?pay-da-1 (grounded-with ?feed-id-1)))
		(printout t "Goal " ?goal-id " executable for " ?robot
		            " also depending on goal " ?feed-id-2 crlf))
)

(defrule goal-dependencies-discard-buffer-cap-executable
" Discard can be executable while buffer-cap is still running. "
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class DISCARD)
	                          (mode FORMULATED)
	                          (params  $? wp-loc ?wp-loc wp-side ?wp-side)
	                          (is-executable FALSE))

	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil) (order-id ?order-id))
	; Robot CEs
	(wm-fact (key central agent robot args? r ?robot))
	(wm-fact (key refbox team-color) (value ?team-color))
	(not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))

	; MPS-Source CEs
	(wm-fact (key domain fact mps-type args? m ?wp-loc t ?))
	(wm-fact (key domain fact mps-team args? m ?wp-loc col ?team-color))

	; wp is not at CS OUTPUT, but after these goals finished, it will be
	(goal (id ?buffer-goal-id)
	      (class BUFFER-CAP)
	      (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
	      (params target-mps ?cs $?))
	(goal-meta (goal-id ?buffer-goal-id) (order-id ?order-id))
	?buffer-da <- (dependency-assignment (goal-id ?goal-id) (class BUFFER-CAP))

	; get cc through buffer actions
	(plan-action (goal-id ?buffer-goal-id)
	             (action-name wp-put)
	             (param-values ?r ?cc $?))

	; get instruct buffer-cap goal
	(goal (id ?instruct-goal-id) (class INSTRUCT-CS-BUFFER-CAP))
	(goal-meta (goal-id ?instruct-goal-id) (order-id ?order-id))
	?instruct-da <- (dependency-assignment (goal-id ?goal-id) (class INSTRUCT-CS-BUFFER-CAP))
	(domain-fact (name zone-content) (param-values ?zz ?wp-loc))
	=>
	(printout t "Goal " ?goal-id " executable for " ?robot
	            " depending on goal " ?buffer-goal-id
	            " and goal " ?instruct-goal-id crlf)
	(modify ?g (params  wp ?cc wp-loc ?wp-loc wp-side ?wp-side)
	           (is-executable TRUE))
	(modify ?buffer-da (params  wp ?cc
	                            wp-loc ?cs
	                            wp-side OUTPUT)
	                   (grounded-with ?buffer-goal-id))
	(modify ?instruct-da (grounded-with ?instruct-goal-id))
)

(defrule goal-dependencies-pay-with-cc-buffer-cap-executable
" Discard can be executable while buffer-cap is still running. "
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class PAY-FOR-RINGS-WITH-CAP-CARRIER)
	                          (mode FORMULATED)
	                          (params  wp ?wp
	                                   wp-loc ?wp-loc
	                                   wp-side ?wp-side
	                                   target-mps ?target-mps
	                                   target-side ?target-side
	                                   $?)
	                          (is-executable FALSE))

	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil) (order-id ?order-id))
	(wm-fact (key refbox team-color) (value ?team-color))

	;MPS-RS CEs
	(wm-fact (key domain fact mps-type args? m ?target-mps t RS))
	(wm-fact (key domain fact mps-team args? m ?target-mps col ?team-color))

	;check ring payment - prevention of overfilling rs
	(wm-fact (key domain fact rs-filled-with args?
	                          m ?target-mps
	                          n ?rs-before&ZERO|ONE|TWO))
	;check that not too many robots try to fill the rs at the same time
	(or (not (goal (class PAY-FOR-RINGS-WITH-BASE|
	                      PAY-FOR-RINGS-WITH-CAP-CARRIER|
	                      PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF)
	               (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
	               (params $? target-mps ?target-mps $?)))
	    (and (goal (class PAY-FOR-RINGS-WITH-BASE|
	                      PAY-FOR-RINGS-WITH-CAP-CARRIER|
	                      PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF)
	               (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
	               (params $? target-mps ?target-mps $?))
	         (test (< (+ (length$ (find-all-facts ((?other-goal goal))
	                                              (and (or (eq ?other-goal:class
	                                                           PAY-FOR-RINGS-WITH-BASE)
	                                                       (eq ?other-goal:class
	                                                           PAY-FOR-RINGS-WITH-CAP-CARRIER)
	                                                       (eq ?other-goal:class
	                                                           PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF))
	                                                   (is-goal-running ?other-goal:mode)
	                                                   (member$ ?target-mps ?other-goal:params)
	                                              )))
	                     (sym-to-int ?rs-before)) 3))
	   )
	)
	; Robot CEs
	(wm-fact (key central agent robot args? r ?robot))
	(not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))

	; MPS-Source CEs
	(wm-fact (key domain fact mps-type args? m ?wp-loc t ?))
	(wm-fact (key domain fact mps-team args? m ?wp-loc col ?team-color))

	; wp is not at CS OUTPUT, but after these goals finished, it will be
	(goal (id ?buffer-goal-id)
	      (class BUFFER-CAP)
	      (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
	      (params target-mps ?cs $?))
	(goal-meta (goal-id ?buffer-goal-id) (order-id ?order-id))
	?buffer-da <- (dependency-assignment (goal-id ?goal-id) (class BUFFER-CAP))

	; get cc through buffer actions
	(plan-action (goal-id ?buffer-goal-id)
	             (action-name wp-put)
	             (param-values ?r ?cc $?))

	; get instruct buffer-cap goal
	(goal (id ?instruct-goal-id) (class INSTRUCT-CS-BUFFER-CAP))
	(goal-meta (goal-id ?instruct-goal-id) (order-id ?order-id))
	?instruct-da <- (dependency-assignment (goal-id ?goal-id)
	                                       (class INSTRUCT-CS-BUFFER-CAP))
	(domain-fact (name zone-content) (param-values ?zz1 ?target-mps))
	(domain-fact (name zone-content) (param-values ?zz2 ?wp-loc))
	=>
	(printout t "Goal " ?goal-id " executable for " ?robot
	            " depending on goal " ?buffer-goal-id
	            " and goal " ?instruct-goal-id crlf)
	(modify ?g (params  wp ?cc
	                    wp-loc ?cs
	                    wp-side OUTPUT
	                    target-mps ?target-mps
	                    target-side ?target-side)
	           (is-executable TRUE))
	(modify ?buffer-da (params  wp ?cc
	                            wp-loc ?cs
	                            wp-side OUTPUT)
	                   (grounded-with ?buffer-goal-id))
	(modify ?instruct-da (grounded-with ?instruct-goal-id))
)
