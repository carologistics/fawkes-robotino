;---------------------------------------------------------------------------
;  remote-actions.clp - Pseudo actions that are not actually executed
;
;  Created: Sar 23 Jun 2020 16:40:59 CET
;  Copyright  2020  Motafa Gomaa <gomaa@kbsg.rwth-aachen.de>
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


(deffunction remote-action-create-args (?actionf)
   (printout t "Creating remote-action args" crlf)
   (bind ?args (create$))
   (bind ?arg-names (fact-slot-names ?actionf))
   (progn$ (?arg-name ?arg-names)
           (bind ?arg-value (fact-slot-value ?actionf ?arg-name))
           (if (multifieldp ?arg-value)
                then
                (bind ?arg-value (create$ [ ?arg-value ])))
           (if (numberp ?arg-value)
               then
               (printout t " Converting numerical value of " ?arg-name " " ?arg-value  " to symbol" crlf)
               (bind ?arg-value (sym-cat ?arg-value)))
           (printout t "arg? "  ?arg-name " " ?arg-value crlf)
           (bind ?args (create$ ?args ?arg-name ?arg-value))
   )
   (return (create$ args? ?args))
)

(deffunction remote-action-updated (?local-actionf ?remote-actionf)

   (bind ?old-key (fact-slot-value ?remote-actionf key))
   (bind ?old-args (wm-key-args ?old-key))

   (printout t "old args " ?old-args crlf)
   (printout t "new args " (remote-action-create-args ?local-actionf) crlf)
   (if (eq ?old-args (remote-action-create-args ?local-actionf)) then
   (printout t "matching :D " ?old-args crlf)
       (return TRUE)
)
   (return FALSE)
)


(defrule remote-action-create-remote-fact
  (wm-fact (key domain fact self args? r ?master))
  ?actionf <- (plan-action (state PENDING)
                          (id ?id)
                          (goal-id ?goal-id)
                          (plan-id ?plan-id)
                          (param-names $?p-names&:(member$ r ?p-names))
                          (param-values $?p-values&:(not (member$ ?master ?p-values))))
  (not (wm-fact (key exec remote-action args? id ?id-sym&:(eq ?id-sym (sym-cat ?id))
                                              goal-id ?goal-id
                                              plan-id ?plan-id
                                              $?)))
  =>
  (bind ?args (remote-action-create-args ?actionf))
  (assert (wm-fact (key exec remote-action ?args)))
)

(defrule remote-action-execution-end
  (wm-fact (key domain fact self args? r ?master))
  ?actionf <- (plan-action (state ?state&PENDING|WAITING|RUNNING)
                           (id ?id)
                           (goal-id ?goal-id)
                           (plan-id ?plan-id)
                           (param-names $?p-names&:(member$ r ?p-names))
                           (param-values $?p-values&:(not (member$ ?master ?p-values))))
;  ?remotef <- (wm-fact (key exec remote-action args? id ?id-sym&:(eq ?id-sym (sym-cat ?id))
;                                                     goal-id ?goal-id
;                                                     plan-id ?plan-id
;                                                     $?
;                                                     state ?new-state&:(neq ?state ?new-state)
;                                                     $?))
  ?remotef <- (wm-fact (key $?key))
  (test (wm-key-prefix ?key (create$ exec remote-action)))
  (test (eq (wm-key-arg ?key id) (sym-cat ?id)))
  (test (eq (wm-key-arg ?key goal-id) ?goal-id))
  (test (eq (wm-key-arg ?key plan-id) ?plan-id))
  (test (neq (wm-key-arg ?key state) ?state ))

 ;(test (not (remote-action-updated ?actionf ?remotef)))
  =>
  (printout t " Creating plan-action " crlf)
  (bind ?fact-string "(plan-action ")
  (bind ?key (fact-slot-value ?remotef key) )
  (bind ?l (+ 2 (length$ (wm-key-path ?key)))) ; + 2 offset from path to first-arg
  (bind ?L (length$ ?key))
  (while (<= ?l ?L) do
         (bind ?arg-name (nth ?l ?key))
         (bind ?arg-value (wm-key-arg ?key ?arg-name))
         (printout t " ( " ?arg-name " " ?arg-value ")" crlf)
         (if (multifieldp ?arg-value)
             then
             (bind ?arg-value (implode$ ?arg-value))
             (bind ?l (+ (wm-key-multifield-arg-end ?key ?l) 1))
             else
             (if (stringp ?arg-value) then (bind ?arg-value (str-cat "\"" ?arg-value "\"")))
             (bind ?l (+ ?l 2)))
         (bind ?slot-string (str-cat "(" ?arg-name " " ?arg-value ")"))
         (bind ?fact-string (str-cat ?fact-string " " ?slot-string))

         (if (and (eq ?arg-name state)
                  (or (eq ?arg-value EXECUTION-SUCCEEDED) (eq ?arg-value EXECUTION-FAILED)))
                  then
                  (retract ?remotef))
  )
  (bind ?fact-string (str-cat ?fact-string " )"))

  (retract ?actionf)
  (assert-string ?fact-string)
)




(defrule remote-action-execution-start
  (wm-fact (key domain fact self args? r ?robot))
  ?remotef <- (wm-fact (key exec remote-action args? id ?id-sym
                                                     goal-id ?goal-id
                                                     plan-id ?plan-id
                                                     $?
                                                     param-names [ r $? ]
                                                     param-values [ ?robot $? ]
                                                     $? ))
  (not (plan-action (id ?id&:(eq ?id (string-to-field (sym-cat ?id-sym))))
                    (goal-id ?goal-id)
                    (plan-id ?plan-id))
  )
  =>
  (printout t " Creating plan-action " crlf)
  (bind ?fact-string "(plan-action ")
  (bind ?key (fact-slot-value ?remotef key) )
  (bind ?l (+ 2 (length$ (wm-key-path ?key)))) ; + 2 offset from path to first-arg
  (bind ?L (length$ ?key))
  (while (<= ?l ?L) do
         (bind ?arg-name (nth ?l ?key))
         (bind ?arg-value (wm-key-arg ?key ?arg-name))
         (printout t " ( " ?arg-name " " ?arg-value ")" crlf)
         (if (multifieldp ?arg-value)
             then
             (bind ?arg-value (implode$ ?arg-value))
             (bind ?l (+ (wm-key-multifield-arg-end ?key ?l) 1))
             else
             (if (stringp ?arg-value) then (bind ?arg-value (str-cat "\"" ?arg-value "\"")))
             (bind ?l (+ ?l 2)))

         (bind ?slot-string (str-cat "(" ?arg-name " " ?arg-value ")"))
         (bind ?fact-string (str-cat ?fact-string " " ?slot-string)))

  (bind ?fact-string (str-cat ?fact-string " )"))
  (assert-string ?fact-string)
)


(defrule remote-action-execution-update
  (wm-fact (key domain fact self args? r ?robot))
  ?actionf <- (plan-action (state ?state)
                           (id ?id)
                           (goal-id ?goal-id)
                           (plan-id ?plan-id)
                           (param-names $?p-names&:(member$ r ?p-names))
                           (param-values $?p-values&:(member$ ?robot ?p-values)))
  ;?remotef <- (wm-fact (key $?key))
  ;(test (wm-key-prefix ?key (create$ exec remote-action)))
  ;(test (eq (wm-key-arg ?key id) (sym-cat ?id)))
  ;(test (eq (wm-key-arg ?key goal-id) ?goal-id))
  ;(test (eq (wm-key-arg ?key plan-id) ?plan-id))
  ;(test (neq (wm-key-arg ?key state) ?state ))
  =>
  (delayed-do-for-all-facts ((?wm wm-fact))
                            (and (wm-key-prefix ?wm:key (create$ exec remote-action))
                                 (eq (wm-key-arg ?wm:key id) (sym-cat ?id))
                                 (eq (wm-key-arg ?wm:key goal-id) ?goal-id)
                                 (eq (wm-key-arg ?wm:key plan-id) ?plan-id)
                                 (neq (wm-key-arg ?wm:key state) ?state))
     (printout t "Updating remote-action fact (state " (wm-key-arg ?wm:key state)
                 ") => (state " ?state ") " crlf)
     (retract ?wm)
     (bind ?args (remote-action-create-args ?actionf))
     (assert (wm-fact (key exec remote-action ?args)))
 )
)
