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


(deffunction plan-positive-atomic-precond-is-previous-effect (?pref)
  (if (not (domain-is-precond-negative (fact-slot-value ?pref part-of))) then
    (do-for-all-facts ((?actf plan-action) (?doef domain-effect))
                      (and
                        (eq ?doef:part-of ?actf:action-name)
                        (eq ?doef:type POSITIVE)
                        (eq (fact-slot-value ?pref grounded) TRUE)
                        (eq (fact-slot-value ?pref plan-id) ?actf:plan-id)
                        (eq (fact-slot-value ?pref goal-id) ?actf:goal-id)
                        (>  (fact-slot-value ?pref grounded-with) ?actf:id)
                        (eq (fact-slot-value ?pref predicate) ?doef:predicate))

    (if  (eq (fact-slot-value ?pref param-values)
         (domain-ground-effect ?doef:param-names ?doef:param-constants ?doef:param-names ?actf:param-values))
       then
       (printout warn " Action  [" ?actf:id " " ?actf:action-name  "]"
                      " effect[" ?doef:name "] ACHIVES "
                      "Action ["  (fact-slot-value ?pref grounded-with) "] PRE-COND "  ?doef:predicate " "
       (domain-ground-effect ?doef:param-names ?doef:param-constants ?actf:param-names ?actf:param-values)   crlf )
  (return TRUE)
   )
   (printout error " Action  [" ?actf:id " " ?actf:action-name  "]"
                   " effect[" ?doef:name "] doesnt achieve "
                   "Action ["  (fact-slot-value ?pref grounded-with) "] PRE-COND "
                   ?doef:predicate ": grounding of  " ?doef:param-names " " ?doef:param-constants " " ?doef:param-names " " ?actf:param-values " is "
                   (domain-ground-effect ?doef:param-names ?doef:param-constants ?actf:param-names ?actf:param-values) " !=! "
       (fact-slot-value ?pref param-values)   crlf )

;    (return TRUE)
)
)
  (return FALSE)
)

(deffunction domain-positive-effect-occurs-in-plan (?plan-id ?predicate ?param-values)
  (bind ?satisfied-by (create$))
  (do-for-all-facts ((?af plan-action) (?df domain-effect))
                    (and (eq ?af:plan-id ?plan-id)
                         (eq ?df:part-of ?af:action-name)
                         (eq ?df:predicate ?predicate)
                         (eq ?df:type POSITIVE)
                         (eq ?param-values (domain-ground-effect ?df:param-names
                                                                 ?df:param-constants
                                                                 ?af:param-names
                                                                 ?af:param-values)))
    (bind ?satisfied-by (append$ ?satisfied-by ?af:id))
  )
  (return ?satisfied-by)
)

(deffunction domain-negative-effect-occurs-in-plan (?plan-id ?predicate ?param-values)
  (bind ?satisfied-by (create$))
  (do-for-all-facts ((?af plan-action) (?df domain-effect))
                    (and (eq ?af:plan-id ?plan-id)
                         (eq ?df:part-of ?af:action-name)
                         (eq ?df:predicate ?predicate)
                         (eq ?df:type NEGATIVE)
                         (eq ?param-values (domain-ground-effect ?df:param-names
                                                                 ?df:param-constants
                                                                 ?af:param-names
                                                                 ?af:param-values)))
    (bind ?satisfied-by (append$ ?satisfied-by ?af:id))
  )
  (return ?satisfied-by)
)

(deffunction domain-positive-effect-earlier-in-plan (?plan-id ?action-id ?predicate ?param-values)
  (bind ?satisfied-by (create$))
  (do-for-all-facts ((?af plan-action) (?df domain-effect))
                    (and (eq ?af:plan-id ?plan-id)
                         (< ?af:id ?action-id)
                         (eq ?df:part-of ?af:action-name)
                         (eq ?df:predicate ?predicate)
                         (eq ?df:type POSITIVE)
                         (eq ?param-values (domain-ground-effect ?df:param-names
                                                                 ?df:param-constants
                                                                 ?af:param-names
                                                                 ?af:param-values)))
    (return TRUE)
  )
  (return FALSE)
)

(deffunction domain-negative-effect-later-in-plan (?plan-id ?action-id ?predicate ?param-values)
  (bind ?satisfied-by (create$))
  (do-for-all-facts ((?af plan-action) (?df domain-effect))
                    (and (eq ?af:plan-id ?plan-id)
                         (> ?af:id ?action-id)
                         (eq ?df:part-of ?af:action-name)
                         (eq ?df:predicate ?predicate)
                         (eq ?df:type NEGATIVE)
                         (eq ?param-values (domain-ground-effect ?df:param-names
                                                                 ?df:param-constants
                                                                 ?af:param-names
                                                                 ?af:param-values)))
    (return TRUE)
  )
  (return FALSE)
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
       )
     )
   )
 )
 (if (> (length$ ?req) 0) then
     (modify ?pf  (required-resources ?req )))
)

(defrule goal-expander-deduce-plan-resources-at-start
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  (disable)
  (goal (id ?goal) (mode SELECTED) (sub-type SCHEDULE-SUBGOALS))
  (plan (id ?plan-id) (goal-id ?goal-id) (required-resources $? ?rs $?))
  (not (wm-fact (key meta plan-resource at-start args? p ?plan-id r ?rs)))
=>
  (printout t "Positive preconds in plan " ?plan-id "  " crlf)
  (bind ?statements (create$))
  (do-for-all-facts ((?af plan-action) (?df domain-atomic-precondition))
                     (and (eq ?af:goal-id ?goal-id)
                          (eq ?df:goal-id ?goal-id)
                          (eq ?af:plan-id ?plan-id)
                          (eq ?df:plan-id ?plan-id)
                          (eq ?df:grounded-with ?af:id)
                          (eq ?df:grounded TRUE)
                          (member$ ?rs ?af:param-values)
                          (member$ ?rs ?df:param-values)
                          (not (domain-is-precond-negative ?df:part-of)))
    ;check if precodition is satisfied by an earlier plan action
    (bind ?previous-effect (domain-positive-effect-earlier-in-plan ?plan-id ?af:id ?df:predicate ?df:param-values))
    (if (not ?previous-effect) then
      (printout t "  action (" ?af:id  " " ?af:action-name ") precond [" ?df:predicate ?df:param-values "]"  crlf)
      (bind ?statements (append$ ?statements (create$ [ ?df:predicate (delete-member$ ?df:param-values ?rs) ])))
    )
  )
  (assert (wm-fact (key meta plan-resource at-start args? p ?plan-id r ?rs)
                   (is-list TRUE) (values ?statements)))
)

(defrule goal-expander-deduce-plan-resources-at-end
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (disable)
  (goal (id ?goal) (mode SELECTED) (sub-type SCHEDULE-SUBGOALS))
  (plan (id ?plan-id) (goal-id ?goal-id) (required-resources $? ?rs $?))
  (not (wm-fact (key meta plan-resource at-end args? p ?plan-id r ?rs)))
=>
  (printout t "Positive effects in plan " ?plan-id "  " crlf)
  (bind ?statements (create$))
  (do-for-all-facts ((?af plan-action) (?df domain-effect))
                     (and (eq ?af:goal-id ?goal-id)
                          (eq ?af:plan-id ?plan-id)
                          (eq ?df:part-of ?af:action-name)
                          (eq ?df:type POSITIVE)
                          (member$ ?rs ?af:param-values)
                          (member$ ?rs (domain-ground-effect ?df:param-names
                                                             ?df:param-constants
                                                             ?af:param-names
                                                             ?af:param-values)))
    (bind ?grounded-param-values (domain-ground-effect ?df:param-names
                                                       ?df:param-constants
                                                       ?af:param-names
                                                       ?af:param-values))
    ;check if positive effect is negated by a later plan action
    (bind ?later-effect (domain-negative-effect-later-in-plan ?plan-id ?af:id ?df:predicate ?grounded-param-values))
    (if (not ?later-effect) then
      (printout t  " action (" ?af:id  " " ?af:action-name ") effect [" ?df:predicate ?grounded-param-values  crlf)
      (bind ?statements (append$ ?statements
         (create$ [ ?df:predicate (delete-member$ ?grounded-param-values ?rs) ])))
    )
  )
  (assert (wm-fact (key meta plan-resource at-end args? p ?plan-id r ?rs)
                   (is-list TRUE) (values ?statements)))
)




(defrule goal-expander-process-plan-preconds-and-effects
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  (disable)
  (goal (id ?goal) (mode SELECTED) (sub-type SCHEDULE-SUBGOALS))
  (plan (id ?plan-id) (goal-id ?goal-id) (required-resources $?req-rs&:(neq ?req-rs (create$))))
  (not (wm-fact (key meta plan-positive-preconds args? p ?plan-id)))
  (not (wm-fact (key meta plan-positive-effects args? p ?plan-id)))
=>
  (printout t "Processing plan " ?plan-id "  " crlf)
  (bind ?positive-preconds (create$))
  (bind ?positive-effects (create$))
  (do-for-all-facts ((?af plan-action)) (and (eq ?af:goal-id ?goal-id)
                                             (eq ?af:plan-id ?plan-id))
    (bind ?action-resources (intersect ?req-rs ?af:param-values))
    (if (> (length$ ?action-resources) 0) then
      ;Plan's Positive preconds
      (do-for-all-facts ((?df domain-atomic-precondition))
                        (and (eq ?df:grounded TRUE)
                             (eq ?df:goal-id ?goal-id)
                             (eq ?df:plan-id ?plan-id)
                             (eq ?df:grounded-with ?af:id)
                             (not (domain-is-precond-negative ?df:part-of)))
          (progn$ (?rs ?action-resources)
               (if (member$ ?rs ?df:param-values) then
                   (bind ?actions (domain-positive-effect-occurs-in-plan ?plan-id
                                                                         ?df:predicate
                                                                         ?df:param-values))
                   (printout t "  For resource " ?rs
                               "  action (" ?af:id  " " ?af:action-name
                               ") precond [" ?df:predicate ?df:param-values
                               "] satisfied by actions " ?actions  crlf)
                   ;check if precodition is satisfied by an earlier plan action
                   (bind ?check FALSE)
                   (progn$ (?act ?actions) (if (< ?act ?af:id) then (bind ?check TRUE)))
                   (if (not ?check) then
                       (bind ?positive-preconds (append$ ?positive-preconds
                                (create$ [ ?df:predicate ?df:param-values  ]))))
              )
          )
      )
      ;Plan's effects
      (do-for-all-facts ((?df domain-effect))
                        (and (eq ?df:type POSITIVE)
                             (eq ?df:part-of ?af:action-name))
           (bind ?grounded-param-values (domain-ground-effect ?df:param-names
                                                              ?df:param-constants
                                                              ?af:param-names
                                                              ?af:param-values))
           (progn$ (?rs ?action-resources)
               (if (member$ ?rs ?grounded-param-values) then
                   (bind ?actions (domain-negative-effect-occurs-in-plan ?plan-id
                                                                         ?df:predicate
                                                                         ?grounded-param-values))
                   (printout t " For resource " ?rs
                               " action (" ?af:id  " " ?af:action-name
                                ") effect [" ?df:predicate ?grounded-param-values
                                 "] negated by actions " ?actions  crlf)
                   ;check if positive effect is negated by a later plan action
                   (bind ?check FALSE)
                   (progn$ (?act ?actions) (if (> ?act ?af:id) then (bind ?check TRUE)))
                   (if (not ?check) then
                       (bind ?postive-effects (append$ ?positive-effects
                              (create$ [ ?df:predicate ?grounded-param-values ]))))
               )
           )
      )
    )
  )
  (assert (wm-fact (key meta plan-positive-preconds args? p ?plan-id)
                   (is-list TRUE) (values ?positive-preconds)))
  (assert (wm-fact (key meta plan-positive-effects args? p ?plan-id)
                   (is-list TRUE) (values ?positive-effects)))
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

