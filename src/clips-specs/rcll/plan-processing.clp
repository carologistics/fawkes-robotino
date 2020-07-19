;---------------------------------------------------------------------------
;  plan-processing.clp - Process plans for meta info
;
;  Created: Tue 07 July 2020 08:03:31 CET
;  Copyright  2020  Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>
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

(defrule goal-expander-calc-action-duration
 (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
 ?pf <- (plan-action (state FORMULATED)
                   (plan-id ?plan-id)
                   (id ?action-id)
                   (action-name ?action-name)
                   (param-names $?param-names)
                   (param-values $?param-values)
                   (duration ?d1&:(<= ?d1 0)))
 (not (plan-action (action-name ?action-name)
                   (param-names $?param-names )
                   (param-values $?param-values)
                   (duration ?d2&:(> ?d2 0))))
 (skill-action-mapping (name ?action-name))
=>


 ;(bind ?duration (map-action-skill (str-cat ?action-name) ?param-names ?param-values))
 ;(printout error "mapped to " ?duration crlf))
 (bind ?duration (integer (round-up (estimate-action-duration (str-cat ?action-name) ?param-names ?param-values))))
 (if (> ?duration 0)
     then
       (modify ?pf (duration ?duration))
     else
      (printout error "Cant find duration for action id " ?action-id "in plan " ?plan-id crlf))
)

(defrule goal-expander-copy-action-duration
 (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
 ?pf <- (plan-action (state FORMULATED)
                   (action-name ?action-name)
                   (param-names $?param-names)
                   (param-values $?param-values)
                   (duration ?d1&:(<= ?d1 0)))
 (plan-action (action-name ?action-name)
              (param-names $?param-names )
              (param-values $?param-values)
              (duration ?d2&:(> ?d2 0)))
=>
 (modify ?pf  (duration ?d2))
)

(defrule goal-expander-deduce-plan-resources
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (goal (id ?g-id) (mode SELECTED) (sub-type SCHEDULE-SUBGOALS))
 ?pf <- (plan (id ?plan-id) (goal-id ?g-id)
              (required-resources $?req&:(eq ?req (create$))))
=>
 (do-for-all-facts ((?paf plan-action)) (eq ?paf:plan-id ?plan-id)
   (progn$ (?p ?paf:param-values)
     (if (not (member$ ?p ?req)) then
       (if (any-factp ((?dof domain-obj-is-of-type))
                      (member$ (create$ ?p resource) ?dof:implied))
           then
           (bind ?req (append$ ?req ?p))
           (assert (wm-fact (key meta plan-resource at-start args? p ?plan-id r ?p pred [ ])
                             (type SYMBOL) (value POSITIVE)))
           (assert (wm-fact (key meta plan-resource at-end args? p ?plan-id r ?p pred [ ])
                            (type SYMBOL) (value POSITIVE)))

       )
     )
   )
 )
 (if (> (length$ ?req) 0) then
     (modify ?pf  (required-resources ?req )))

)

(defrule expanded-plan-processing-deduce-plan-resources-at-start
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  (goal (id ?g) (mode SELECTED) (sub-type SCHEDULE-SUBGOALS))
  (plan (id ?p) (goal-id ?g) (required-resources $? ?rs $?))
  (domain-object (name ?rs) (type ?rs-type))
  (resource-info (type ?rs-type) (setup-preds $?setups) (state-preds $?states))
  (domain-atomic-precondition
    (grounded TRUE)
    (part-of ?part-of)
    (goal-id ?g)
    (plan-id ?p)
    (grounded-with ?action)
    (predicate ?predicate&:(or (member$ ?predicate ?setups) (member$ ?predicate ?states)))
    (param-values $?param-values&:(member$ ?rs ?param-values))
    )
  (not (domain-effect
        (grounded TRUE)
        (type POSITIVE)
        (goal-id ?g)
        (plan-id ?p)
        (grounded-with ?earlier-action&:(< ?earlier-action ?action))
        (predicate ?predicate)
        (param-values $?param-values)
  ))
  (test (not (domain-is-precond-negative ?part-of)))
=>
  (bind ?statement (create$ [ ?predicate (delete-member$ ?param-values ?rs)  ]))
  (printout t "Precon of plan (" ?p ") res (" ?rs ")  action (" ?action ") " ?statement  crlf)
  (assert (wm-fact (key meta plan-resource at-start args? p ?p r ?rs pred ?statement)
                   (type SYMBOL) (value POSITIVE)))
)

(defrule expanded-plan-processing-deduce-plan-resources-at-end
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  (goal (id ?g) (mode SELECTED) (sub-type SCHEDULE-SUBGOALS))
  (plan (id ?p) (goal-id ?g) (required-resources $? ?rs $?))
  (domain-object (name ?rs) (type ?rs-type))
  (resource-info (type ?rs-type) (setup-preds $?setups) (state-preds $?states))
  (domain-effect
    (grounded TRUE)
    (type ?effect-type)
    (goal-id ?g)
    (plan-id ?p)
    (grounded-with ?action)
    (predicate ?predicate&:(or (member$ ?predicate ?setups) (member$ ?predicate ?states)))
    (param-values $?param-values&:(member$ ?rs ?param-values))
    )
  (not (domain-effect
        (grounded TRUE)
        (type ?later-type&:(neq ?effect-type ?later-type))
        (goal-id ?g)
        (plan-id ?p)
        (grounded-with ?later-action&:(> ?later-action ?action))
        (predicate ?predicate)
        (param-values $?param-values)
  ))
=>
  (bind ?statement (create$ [ ?predicate (delete-member$ ?param-values ?rs) ]))
  (printout t "Effect of plan (" ?p ") res (" ?rs ")  action (" ?action ") " ?statement  crlf)
  (assert (wm-fact (key meta plan-resource at-end args? p ?p r ?rs pred ?statement)
                   (type SYMBOL) (value ?effect-type)))
)

