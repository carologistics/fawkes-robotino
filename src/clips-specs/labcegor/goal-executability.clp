;---------------------------------------------------------------------------
;  goal-executability.clp - Check executability of production goals
;
;  Created: Sat 30 Apr 2022 18:44:00 CET
;  Copyright  2021  Tarik Viehmann <viehmann@kbsg.rwth-aachen.de>
;             2021  Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
;             2021  Sonja Ginter <sonja.ginter@rwth-aachen.de>
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
  ?*PRODUCE-C0-AHEAD-TIME* = 150
  ?*PRODUCE-C1-AHEAD-TIME* = 250
  ?*PRODUCE-C2-AHEAD-TIME* = 350
  ?*PRODUCE-C3-AHEAD-TIME* = 450
  ?*DELIVER-AHEAD-TIME* = 60
)


; ----------------------- Production GOALS -------------------------------

(defrule goal-production-enter-field-executable
 " ENTER-FIELD is executable for a robot if it has not entered the field yet."
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?id) (class ENTER-FIELD) (sub-type SIMPLE) (mode FORMULATED)
	      (params team-color ?team-color)
	      (is-executable FALSE))
	(goal-meta (goal-id ?id) (assigned-to ?robot&~nil))
	(wm-fact (key refbox state) (value RUNNING))
	(wm-fact (key refbox phase) (value PRODUCTION|EXPLORATION))
	(wm-fact (key refbox team-color) (value ?team-color))
	; (NavGraphGeneratorInterface (final TRUE))
	(not (wm-fact (key domain fact entered-field
	               args? r ?robot)))
	=>
	(printout t "Goal ENTER-FIELD executable for " ?robot crlf)
	(modify ?g (is-executable TRUE))
)

(defrule goal-production-buffer-cap-executable
" Bring a cap-carrier from a cap stations shelf to the corresponding mps input
  to buffer its cap. "
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?id) (class TESTGOAL) (sub-type SIMPLE)
	            (mode FORMULATED)
	            (params target-cs ?cs
	                    cc ?cc
	            )
	            (is-executable FALSE))
	(goal-meta (goal-id ?id) (assigned-to ?robot&~nil))
	(wm-fact (key refbox team-color) (value ?team-color))
	; Robot CEs
	(wm-fact (key central agent robot args? r ?robot))
	; MPS CEs
	(wm-fact (key domain fact mps-type args? m ?cs t CS))
	(wm-fact (key domain fact mps-state args? m ?cs s ~BROKEN))
	(wm-fact (key domain fact mps-team args? m ?cs col ?team-color))
	(wm-fact (key domain fact cs-can-perform args? m ?cs op RETRIEVE_CAP))
	(not (wm-fact (key domain fact cs-buffered args? m ?cs col ?any-cap-color)))
	(not (wm-fact (key domain fact wp-at args? wp ?wp-a m ?cs side INPUT)))
	; Capcarrier CEs
	(not (wm-fact (key domain fact holding args? r ?robot wp ?wp-h)))
	(wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?mps spot ?spot))
	(domain-fact (name zone-content) (param-values ?zz ?mps))
	=>
	(printout t "Goal TESTGOAL executable for " ?robot crlf)
	(modify ?g (is-executable TRUE))
)

