;---------------------------------------------------------------------------
;  goal-dependencies.clp - Defines dependency-assignments and grounds
;						   them for executability
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

	; goal class of dependence-goal
	(slot class (type SYMBOL))

	; defines if goal waits for dependency before wp-get (wait-for-wp)
	; or before wp-put (wait-for-free-side), set when asserting
	; values: WP, FREE-SIDE
	(slot wait-for (type SYMBOL))

	; necessary parameters used for goal-expander of dependency-goal, set in execution-check
	; for deliver-mount-cap:	wp, wp-loc, wp-side
	(multislot params (type SYMBOL))

	; id of dependence-goal, nil if ungrounded
	(slot grounded-with (type SYMBOL))

	;(slot priority (type float) (default 0.0))
)

; ---------------------------- Class Dependencies ----------------------------
; A goal depends on a class of a dependency-goal if such a dependency-goal is
; always required for executing this goal under dependencies

(defrule goal-dependencies-mount-cap-buffer-cap
" Every mount-cap goal depends on the buffer-cap class. Per default, no buffer-cap goal is grounded. "
	; needs to be higher than SALIENCE-GOAL-EXECUTABLE-CHECK
	(declare (salience (+ ?*SALIENCE-GOAL-EXECUTABLE-CHECK* 1)))
	?g <- (goal (id ?goal-id)
	            (class MOUNT-CAP)
	            (mode FORMULATED)
	            (params $? target-mps ?target-mps $?))
	(not (dependency-assignment (goal-id ?goal-id) (class BUFFER-CAP)))
	(not (dependency-assignment (goal-id ?goal-id) (class INSTRUCT-CS-BUFFER-CAP)))
	=>
	(printout t "Goal " ?goal-id " depends on class BUFFER-CAP " crlf)
	(assert (dependency-assignment (goal-id ?goal-id)
	                               (class BUFFER-CAP)
	                               (wait-for FREE-SIDE)
	                               (grounded-with nil)))
	(printout t "Goal " ?goal-id " depends on class INSTRUCT-CS-BUFFER-CAP " crlf)
	(assert (dependency-assignment (goal-id ?goal-id)
	                               (class INSTRUCT-CS-BUFFER-CAP)
	                               (wait-for FREE-SIDE)
	                               (grounded-with nil)))
)

(defrule goal-dependencies-deliver-mount-cap
" Every deliver goal depends on the mount-cap class. Per default, no mount-cap goal is grounded. "
	; needs to be higher than SALIENCE-GOAL-EXECUTABLE-CHECK
	(declare (salience (+ ?*SALIENCE-GOAL-EXECUTABLE-CHECK* 1)))
	?g <- (goal (id ?goal-id)
	            (class DELIVER)
	            (mode FORMULATED)
	            (params wp ?wp $?))
	(not (dependency-assignment (goal-id ?goal-id) (class MOUNT-CAP)))
	(not (dependency-assignment (goal-id ?goal-id) (class INSTRUCT-CS-MOUNT-CAP)))
	=>
	(printout t "Goal " ?goal-id " depends on class MOUNT-CAP " crlf)
	(assert (dependency-assignment (goal-id ?goal-id)
	                               (class MOUNT-CAP)
	                               (wait-for WP)
	                               (grounded-with nil)))
	(printout t "Goal " ?goal-id " depends on class INSTRUCT-CS-MOUNT-CAP " crlf)
	(assert (dependency-assignment (goal-id ?goal-id)
	                               (class INSTRUCT-CS-MOUNT-CAP)
	                               (wait-for WP)
	                               (grounded-with nil)))
)

(defrule goal-dependencies-discard-buffer-cap
" Every discard goal depends on the buffer-cap class. Per default, no buffer-cap goal is grounded. "
	; needs to be higher than SALIENCE-GOAL-EXECUTABLE-CHECK
	(declare (salience (+ ?*SALIENCE-GOAL-EXECUTABLE-CHECK* 1)))
	?g <- (goal (id ?goal-id) (class DISCARD) (mode FORMULATED))
	(not (dependency-assignment (goal-id ?goal-id) (class BUFFER-CAP)))
	(not (dependency-assignment (goal-id ?goal-id) (class INSTRUCT-CS-BUFFER-CAP)))
	=>
	(printout t "Goal " ?goal-id " depends on class BUFFER-CAP " crlf)
	(assert (dependency-assignment (goal-id ?goal-id)
	                               (class BUFFER-CAP)
	                               (wait-for WP)
	                               (grounded-with nil)))
	(printout t "Goal " ?goal-id " depends on class INSTRUCT-CS-BUFFER-CAP " crlf)
	(assert (dependency-assignment (goal-id ?goal-id)
	                               (class INSTRUCT-CS-BUFFER-CAP)
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
	                          (meta $? assigned-to ?robot $?)
	                          (is-executable FALSE))

	; Robot CEs
	(wm-fact (key central agent robot args? r ?robot))
	(wm-fact (key refbox team-color) (value ?team-color))

	; MPS-CS CEs
	(wm-fact (key domain fact mps-type args? m ?target-mps t CS))
	(wm-fact (key domain fact mps-state args? m ?target-mps s ~BROKEN))
	(not (wm-fact (key domain fact wp-at args? wp ?any-wp m ?target-mps side INPUT)))
	(wm-fact (key domain fact mps-team args? m ?target-mps col ?team-color))

	(not (wm-fact (key domain fact cs-buffered args? m ?target-mps col ?)))
	;(wm-fact (key domain fact cs-can-perform args? m ?target-mps op MOUNT_CAP))
	(wm-fact (key domain fact mps-side-free args? m ?target-mps side OUTPUT))

	; CS is not buffered, but with the following dependency, we can assume it will soon
	(and  ; A feasible (same parameter) buffer-cap or instruct-cs-buffer-cap goal is executing...
	     (goal (id ?dependency-goal-id)
	           (class ?dependency-class&BUFFER-CAP|INSTRUCT-CS-BUFFER-CAP)
	           (parent ?parent)
	           (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
	           (params target-mps ?target-mps $?))
	       ; ... and it is not already grounded...
	     (not (and   ; ... meaning there is another dependency...
	               (dependency-assignment (goal-id ?other-goal-id) (grounded-with ?dependency-goal-id))
	                 ; ... and the other goal is already executing
	               (goal (id ?other-goal-id)
	                     (class MOUNT-CAP)
	                     (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
	                     (params $? target-mps ?target-mps $?))
	          )
	     )
	)
	?da <- (dependency-assignment (goal-id ?goal-id) (class ?dependency-class))

	; WP CEs
	(wm-fact (key wp meta next-step args? wp ?wp) (value CAP))
	; MPS-Source CEs
	(wm-fact (key domain fact mps-type args? m ?wp-loc t ?))
	(wm-fact (key domain fact mps-team args? m ?wp-loc col ?team-color))

	(or (and ; Either the workpiece needs to picked up...
	         (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
	           ; ... and it is a fresh base located in a base station
	         (or (and (wm-fact (key domain fact mps-type args? m ?wp-loc t BS))
	                  (wm-fact (key domain fact wp-unused args? wp ?wp))
	                  (wm-fact (key domain fact wp-base-color args? wp ?wp col BASE_NONE)))
	               ; ... or is already at some machine
	             (wm-fact (key domain fact wp-at args? wp ?wp m ?wp-loc side ?wp-side))
	         )
	    )
	      ; or the workpiece is already being held
	    (wm-fact (key domain fact holding args? r ?robot wp ?wp))
	)
	=>
	(printout t "Goal " ?goal-id " executable for " ?robot
	            " depending on goal " ?dependency-goal-id crlf)
	(modify ?g (is-executable TRUE))
	(modify ?da (grounded-with ?dependency-goal-id))

	; If a buffer-cap goal is grounded with ?g, also ground its instruct-cs-buffer-cap goal to ?g
	(if (eq ?dependency-class BUFFER-CAP)
	 then (do-for-fact ((?instruct-goal goal) (?instruct-da dependency-assignment))
		      (and (eq ?instruct-goal:parent ?parent) ; depending on tree: in this case buffer-cap is its sibling
		           (eq ?instruct-goal:class INSTRUCT-CS-BUFFER-CAP)
		           (eq ?instruct-da:goal-id ?goal-id)
		           (eq ?instruct-da:class INSTRUCT-CS-BUFFER-CAP))
		      (modify ?instruct-da (grounded-with ?instruct-goal:id))
		  )
		  (printout t "Goal " ?goal-id " executable for " ?robot
		              " depending on goal INSTRUCT-CS-BUFFER-CAP" crlf)
	)
)

(defrule goal-dependencies-mount-cap-deliver-executable ; output is blocked
" Even if the CS is not buffered and a wp is blocking its output, mount-cap can be executable if the there is
  a running deliver goal for the blocking wp and a running buffer-cap goal for the CS. "
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class MOUNT-CAP)
	                          (mode FORMULATED)
	                          (params  wp ?wp
	                                   target-mps ?target-mps
	                                   target-side ?target-side
	                                   $?)
	                          (meta $? assigned-to ?robot $?)
	                          (is-executable FALSE))

	; Robot CEs
	(wm-fact (key central agent robot args? r ?robot))
	(wm-fact (key refbox team-color) (value ?team-color))

	; MPS-CS CEs
	(wm-fact (key domain fact mps-type args? m ?target-mps t CS))
	(wm-fact (key domain fact mps-state args? m ?target-mps s ~BROKEN))
	(not (wm-fact (key domain fact wp-at args? wp ?any-wp m ?target-mps side INPUT)))
	(wm-fact (key domain fact mps-team args? m ?target-mps col ?team-color))

	(not (wm-fact (key domain fact cs-buffered args? m ?target-mps col ?cap-color)))
	;(wm-fact (key domain fact cs-can-perform args? m ?target-mps op MOUNT_CAP))

	(wm-fact (key domain fact wp-at args? wp ?blocking-wp m ?target-mps side OUTPUT))
	; CS output is not free, but with this goal executing, we can assume it will soon
    (goal (id ?deliver-id) (class DELIVER) (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
	         (params  wp ?blocking-wp $?))

	; CS is not buffered, but with the following dependency, we can assume it will soon
	(and  ; A feasible (same parameter) buffer-cap or instruct-cs-buffer-cap goal is executing...
	     (goal (id ?dependency-goal-id)
	           (class ?dependency-class&BUFFER-CAP|INSTRUCT-CS-BUFFER-CAP)
	           (parent ?parent)
	           (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
	           (params target-mps ?target-mps $?))
	       ; ... and it is not already grounded...
	     (not (and    ; ... meaning there is another dependency...
	               (dependency-assignment (goal-id ?other-goal-id)
	                                      (grounded-with ?dependency-goal-id))
	                  ; ... and the other goal is already executing
	               (goal (id ?other-goal-id)
	                     (class MOUNT-CAP)
	                     (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
	                     (params $? target-mps ?target-mps $?))
	          )
	     )
	)
	?da <- (dependency-assignment (goal-id ?goal-id) (class ?dependency-class))

	; WP CEs
	(wm-fact (key wp meta next-step args? wp ?wp) (value CAP))
	; MPS-Source CEs
	(wm-fact (key domain fact mps-type args? m ?wp-loc t ?))
	(wm-fact (key domain fact mps-team args? m ?wp-loc col ?team-color))

	(or (and ; Either the workpiece needs to picked up...
	         (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
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
	=>
	(printout t "Goal " ?goal-id " executable for " ?robot
	            " depending on goal " ?dependency-goal-id
	            " and goal " ?deliver-id crlf)
	(modify ?g (is-executable TRUE))
	(modify ?da (grounded-with ?dependency-goal-id))
	(assert (dependency-assignment (goal-id ?goal-id) (class DELIVER) (grounded-with ?deliver-id)))

	; If a buffer-cap goal is grounded with ?g, also ground its instruct-cs-buffer-cap goal to ?g
	(if (eq ?dependency-class BUFFER-CAP)
		then (do-for-fact ((?instruct-goal goal) (?instruct-da dependency-assignment))
			(and (eq ?instruct-goal:parent ?parent) ; depending on tree: in this case buffer-cap is its sibling
			     (eq ?instruct-goal:class INSTRUCT-CS-BUFFER-CAP)
			     (eq ?instruct-da:goal-id ?goal-id)
			     (eq ?instruct-da:class INSTRUCT-CS-BUFFER-CAP))
			(modify ?instruct-da (grounded-with ?instruct-goal:id)))
			(printout t "Goal " ?goal-id " executable for " ?robot
			            " depending on goal INSTRUCT-CS-BUFFER-CAP" crlf)
	)
)

(defrule goal-dependencies-deliver-mount-cap-executable
" Bring a product to the delivery station.
"
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class DELIVER)
	                          (mode FORMULATED)
	                          (params  wp ?unknown-wp
	                                   target-mps ?target-mps
	                                   target-side ?target-side
	                                   $?)
	                          (meta $? assigned-to ?robot $?)
	                          (is-executable FALSE))

	; Robot CEs
	(wm-fact (key central agent robot args? r ?robot))
	(wm-fact (key refbox team-color) (value ?team-color))

	; MPS-CS CEs
	(wm-fact (key domain fact mps-type args? m ?target-mps t DS))
	(wm-fact (key domain fact mps-state args? m ?target-mps s ~BROKEN))
	(not (wm-fact (key domain fact wp-at args? wp ?any-wp m ?target-mps side INPUT)))
	(wm-fact (key domain fact mps-team args? m ?target-mps col ?team-color))

	; wp is not at CS OUTPUT, but after these goals finished, it will be
	(goal (id ?mount-goal-id) ;TODO: dafür sorgen, dass das auch das richtige mount-cap goal ist
	      (class MOUNT-CAP)
	      (parent ?parent)
	      (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
	      (params wp ?wp
	              target-mps ?mps
	              $?))
	?mount-da <- (dependency-assignment (goal-id ?goal-id) (class MOUNT-CAP))

	(wm-fact (key domain fact cs-buffered args? m ?mps col ?cap-color))
	(wm-fact (key domain fact mps-side-free args? m ?mps side OUTPUT))

	; get instruct goal through tree relationship to mount-cap, in this case sibling of grandparent
	(goal (id ?parent) (parent ?grandparent))
	(goal (id ?grandparent) (parent ?great-grandparent))
	(goal (id ?instruct-goal-id)
	      (class INSTRUCT-CS-MOUNT-CAP)
	      (parent ?great-grandparent)
	      (mode FORMULATED))
	?instruct-da <- (dependency-assignment (goal-id ?goal-id) (class INSTRUCT-CS-MOUNT-CAP))

	(not (wm-fact (key domain fact holding args? r ?robot wp ?some-wp)))
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

(defrule goal-dependencies-discard-buffer-cap-executable
" Bring a product to a cap station to mount a cap on it.
"
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class DISCARD)
	                          (mode FORMULATED)
	                          (parent ?parent)
	                          (params  $? wp-loc ?wp-loc wp-side ?wp-side)
	                          (meta $? assigned-to ?robot $?)
	                          (is-executable FALSE))

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
	      (parent ?parent)
	      (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
	      (params target-mps ?wp-loc $?))
	?buffer-da <- (dependency-assignment (goal-id ?goal-id) (class BUFFER-CAP))

	; get cc through buffer actions
	(plan-action (goal-id ?buffer-goal-id) (action-name wp-put) (param-values ?r ?cc ?wp-loc))

	(goal (id ?instruct-goal-id)
	      (class INSTRUCT-CS-BUFFER-CAP)
	      (parent ?parent)	; depends on tree structur
	      (mode FORMULATED))
	?instruct-da <- (dependency-assignment (goal-id ?goal-id) (class INSTRUCT-CS-BUFFER-CAP))
	=>
	(printout t "Goal " ?goal-id " executable for " ?robot
	            " depending on goal " ?buffer-goal-id
	            " and goal " ?instruct-goal-id crlf)
	(modify ?g (params  wp ?cc wp-loc ?wp-loc wp-side ?wp-side) (is-executable TRUE))
	(modify ?buffer-da (grounded-with ?buffer-goal-id))
	(modify ?instruct-da (grounded-with ?instruct-goal-id))
)


;TODO:  use noop-action "wait" to test dependency-assignments and wait-for-goal actions reliable:
;			wait is the first action of each tested dependence-goal, is doing nothing and for xxx seconds (set in wait-actions.clp)
;				therefore, the goal, which is executable under the dependency-assignment of the dependence-goal, needs to be
;				 selected while the dependence-goal is still dispatched and needs to wait for it using the wait-for-goal noop-action
;TODO:  write code when flushing executability to also flush grounded-with of all dependency-assignments of formulated goals
;TODO:	write defrule goal-dependencies-mount-ring-feed-rs
;			this defrule creates multiple dependency-assignments for every formulated mount-ring goal depending on the number
;			 of payments with class slot feed-rs
;			my assumption: mount ring is only executable if rs is paid with a minimum number of bases or the number of already paid
;			 bases and feed-rs goals for that rs sum up to this number
;				my reason: even though there should not be a wait-for-goal noop-action stopping moun-ring from blocking the rs input,
;				 performing the mount-ring goal is very commiting to mounting this wp. Whenever there is the choice between
;				 a mount-ring goal, which is not executable with my assumption, and a feed-ring, feed-ring should be chosen since
;				 it is more uncommitle.
;				possible exception: the distance between the wp to mount the ring on and the robot is closer than to a feedable base
;									in RoboCup 2021 there is only 1 order, therefore dependencies are not needed. Mount-ring
;									 can be executed whenever possible
;				a defrule goal-dependencies-mount-ring-executable needs to be written accordingly
;					meaning all dependency-assignments of mount-ring are grounded with finished & completed and
;					 selected|expanded|commited|dispatched feed-rs goals
;					potential problem: while checking executability do not assign multiple dependency-assignment of mount-ring to
;					 the same feed-rs goal
;			clear-output goals for capcarriers at cs output can change their dependency-priority depending on these
;			 dependency-assignments
;IDEA:	dependency-assignments can be used in some way to avoid deadlocks at rs
;			for example: write defrule goal-dependencies-mount-ring-executable-clear-output
;				similar to clear-output goals for capcarriers at cs output, there should be a choice between another mount-ring,
;				 mount-cap and some wait-holding-wp goal just to clear the output whenever a wp is at rs output
;				this defrule checks if there is a wp at output and creates a dependency-assighnment with the clear-output goal
;				it also checks if the wp at output needs to mount another ring at the same rs and sets mount-ring to unexecutable
;				 until wait-holding-wp is executed
