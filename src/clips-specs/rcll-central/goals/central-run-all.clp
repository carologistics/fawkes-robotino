;---------------------------------------------------------------------------
;  central-run-all.clp - CLIPS executive - goal to run all subgoals
;                                          for the central agent
;
;  Created: Sun 16 May 2021 23:34:00 CET
;  Copyright  2021  Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
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
; Sub-type: CENTRAL-RUN-ALL-OF-SUBGOALS
; Perform: one goal at a time, ordered by goal priority
; Succeed: if all sub-goal succeeds
; Fail:    if exactly one sub-goal fails
;
; A CENTRAL-RUN-ALL parent goal will order the executable  goals by priority
; and then start performing them in order. If any goal fails, the parent
; fails. If all goals have been completed successfully, the parent
; goal succeeds.
;
; Through the goal meta a CENTRAL-RUN-ALL goal can be set to sequence mode
; (add sequence-mode to goal meta). Then goal selection behavior changes
; the following way: if there is a formulated goal of higher priority that is
; not executable, do not select any child.
;
; This goal is part of the centralized goal reasoning approach for the
; RCLL 2021 season. It has less modes than normal goals (formulated, selected,
; dispatched, finished, failed). The CENTRAL-RUN-ALL goal is used to implement
; sequential goal progression.
;
; Interactions:
; - User FORMULATES goal
; - AUTOMATIC: SELECT executable sub-goal with lowest ordering number (this happens in
;              goal-reasoner.clp)
; - Automatic: when sub-goal is EVALUATED, outcome determines parent goal:
;   * FAILED: mode FINISHED, outcome FAILED, message
;   * COMPLETED: mode FINISHED, outcome COMPLETED
; User: EVALUATE goal

(defrule central-run-all-goal-expand-failed
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-ALL-OF-SUBGOALS)
	             (mode EXPANDED))
	(not (goal (type ACHIEVE) (parent ?id)))
	=>
	(modify ?gf (mode FINISHED) (outcome FAILED)
	            (error NO-SUB-GOALS)
	            (message (str-cat "No sub-goal for RUN-ALL goal '" ?id "'")))
)

(defrule central-run-all-goal-commit
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-ALL-OF-SUBGOALS)
	             (mode EXPANDED))
	(goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode FORMULATED))
	=>
	(modify ?gf (mode COMMITTED))
)

(defrule central-run-all-goal-dispatch
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-ALL-OF-SUBGOALS)
	             (mode COMMITTED)
	             (required-resources $?req)
	             (acquired-resources $?acq&:(subsetp ?req ?acq)))
	=>
	(modify ?gf (mode DISPATCHED))
)

(defrule central-run-all-goal-subgoal-finished
	"Set the goal to finished when all subgoals are finished."
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-ALL-OF-SUBGOALS)
	             (mode FORMULATED))
	(not  (goal (parent ?id) (type ACHIEVE) (mode ~FINISHED)))
	=>
	(modify ?gf (mode FINISHED) (outcome COMPLETED))
)

(defrule central-run-all-goal-subgoal-failed
	"Fail the goal if any of the child goals fail to propagate error handling."
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-ALL-OF-SUBGOALS)
	             (mode DISPATCHED))
	?sg <- (goal (parent ?id) (type ACHIEVE) (mode FINISHED) (outcome FAILED))
	=>
	(modify ?gf (mode FINISHED) (outcome FAILED)
	            (error SUB-GOAL-FAILED ?sg)
	            (message (str-cat "Sub-goal '" (fact-slot-value ?sg id) "' of CENTRAL-RUN-ALL goal '" ?id "' has failed")))
)

(defrule central-run-all-goal-finish-all-subgoals-finished-completed
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-ALL-OF-SUBGOALS)
	             (mode DISPATCHED) (meta $?meta&:(not (member$ do-not-finish ?meta))))
	(not (goal (parent ?id) (type ACHIEVE) (mode RETRACTED|FINISHED) (outcome ~COMPLETED)))
	(not (goal (parent ?id) (type ACHIEVE) (mode ~FINISHED&~RETRACTED)))
	=>
	(modify ?gf (mode FINISHED) (outcome COMPLETED))
)
