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

(defglobal
  ?*REMOTE-ACTION-ARGS-NAMES* = (create$ id goal-id plan-id action-name)
)


(deffunction remote-action-create-wm-args (?actionf)
   (printout t "Remote-action " (fact-slot-value ?actionf action-name)
               (fact-slot-value ?actionf param-values)
               " is " (fact-slot-value ?actionf state) crlf)
   (bind ?args (create$))
   ;(bind ?arg-keys (fact-slot-names ?actionf))
   (bind ?arg-keys ?*REMOTE-ACTION-ARGS-NAMES* )
   (progn$ (?arg-key ?arg-keys)
           (bind ?arg-value (fact-slot-value ?actionf ?arg-key))
           (if (multifieldp ?arg-value)
                then
                (bind ?arg-value (create$ [ ?arg-value ])))
           (if (numberp ?arg-value)
               then
               ;(printout t " Converting numerical value of " ?arg-key " " ?arg-value  " to symbol" crlf)
               (bind ?arg-value (sym-cat ?arg-value)))
           ;(printout t "arg? "  ?arg-key ": " ?arg-value crlf)
           (bind ?args (create$ ?args ?arg-key ?arg-value))
   )
   (return (create$ args? ?args))
)

(deffunction remote-action-create-wm-values (?actionf)
   ;(printout t "Creating remote-action key args?" crlf)
   (bind ?values-list (create$))
   (bind ?arg-keys (delete-member$ (fact-slot-names ?actionf) ?*REMOTE-ACTION-ARGS-NAMES* ))
   (progn$ (?arg-key ?arg-keys)
           (bind ?arg-value (fact-slot-value ?actionf ?arg-key))
           (if (multifieldp ?arg-value)
                then
                (bind ?arg-value (create$ [ ?arg-value ])))

           ;(printout t "       "  ?arg-key " " ?arg-value crlf)
           (bind ?values-list (create$ ?values-list ?arg-key ?arg-value))
   )
   (return ?values-list)
)

(deffunction remote-action-to-plan-action-str (?wm-idx)
  ;(printout t " Creating plan-action from remote-action " crlf)
  ;(printout t "  (plan-action " crlf)
  (bind ?fact-string " (plan-action ")
  ;Process remote-action key
  (bind ?wm-key (fact-slot-value ?wm-idx key) )
  (bind ?l (+ 2 (length$ (wm-key-path ?wm-key)))) ; + 2 offset from path to first-arg
  (bind ?L (length$ ?wm-key))
  (while (<= ?l ?L) do
         (bind ?slot-name (nth ?l ?wm-key))
         (bind ?slot-value (wm-key-arg ?wm-key ?slot-name))
         ;(printout t "     (" ?slot-name " " ?slot-value ")" crlf)
         (if (multifieldp ?slot-value)
             then
             (bind ?slot-value (implode$ ?slot-value))
             (bind ?l (+ (wm-key-multifield-arg-end ?wm-key ?l) 1))
             else
             (if (stringp ?slot-value) then (bind ?slot-value (str-cat "\"" ?slot-value "\"")))
             (bind ?l (+ ?l 2)))

         (bind ?slot-string (str-cat "(" ?slot-name " " ?slot-value ")"))
         (bind ?fact-string (str-cat ?fact-string " " ?slot-string))
  )
  ;Process remote-action values
  (bind ?wm-values (fact-slot-value ?wm-idx values) )
  (bind ?l 1)
  (bind ?L (length$ ?wm-values))
  (while (<= ?l ?L) do
         (bind ?slot-name (nth ?l ?wm-values))
         (bind ?l (+ 1 ?l))
         (bind ?slot-value (create$))
         (if (eq [ (nth$ ?l ?wm-values))
             then
             (bind ?l (+ 1 ?l))
             (while (and (<= ?l ?L) (neq ] (nth$ ?l ?wm-values)))
                    (bind ?slot-value (append$ ?slot-value (nth$ ?l ?wm-values)))
                    (bind ?l (+ 1 ?l)))
             else
             (bind ?slot-value (nth$ ?l ?wm-values))
             (if (stringp ?slot-value) then (bind ?slot-value (str-cat "\"" ?slot-value "\"")))
         )
         (if (multifieldp ?slot-value) then (bind ?slot-value (implode$ ?slot-value)))

         ;(printout t "    (" ?slot-name " " ?slot-value ")" crlf)
         (bind ?slot-string (str-cat "(" ?slot-name " " ?slot-value ")"))
         (bind ?fact-string (str-cat ?fact-string " " ?slot-string))
         (bind ?l (+ 1 ?l))
  )

  ;(printout t ")" crlf)
  (return (str-cat ?fact-string " )"))
)


(defrule remote-action-create-remote-fact
  (wm-fact (key domain fact self args? r ?self))
  (wm-fact (key cx identity) (value ?self-str))
  ?actionf <- (plan-action (state PENDING)
                           (executable TRUE)
                           (id ?id)
                           (goal-id ?goal-id)
                           (plan-id ?plan-id)
                           (param-names $?p-names&:(member$ r ?p-names))
                           (param-values $?p-values&:(and (not (member$ ?self ?p-values))
                                                          (not (member$ ?self-str ?p-values))))
              )
  (not (wm-fact (key exec remote-action args? id ?id-sym&:(eq ?id-sym (sym-cat ?id))
                                              goal-id ?goal-id
                                              plan-id ?plan-id
                                              $?)))
  =>
  (bind ?args (remote-action-create-wm-args ?actionf))
  (bind ?values (remote-action-create-wm-values ?actionf))
  (assert (wm-fact (key exec remote-action ?args) (is-list TRUE) (type SYMBOL) (values ?values)))
)

(defrule remote-action-execution-end
  (wm-fact (key domain fact self args? r ?master))
  ?actionf <- (plan-action (state ?state&PENDING|WAITING|RUNNING)
                           (id ?id)
                           (goal-id ?goal-id)
                           (plan-id ?plan-id)
                           (param-names $?p-names&:(member$ r ?p-names))
                           (param-values $?p-values&:(not (member$ ?master ?p-values))))
  ?remotef <- (wm-fact (key $?key) (values $? state ?new-state $?))
  (test (wm-key-prefix ?key (create$ exec remote-action)))
  (test (eq (wm-key-arg ?key id) (sym-cat ?id)))
  (test (eq (wm-key-arg ?key goal-id) ?goal-id))
  (test (eq (wm-key-arg ?key plan-id) ?plan-id))
  (test (neq ?new-state ?state))
  =>
  (retract ?actionf)
  (assert-string (remote-action-to-plan-action-str ?remotef ))

  (if (or (eq ?new-state EXECUTION-SUCCEEDED)
          (eq ?new-state EXECUTION-FAILED))
      then
      (retract ?remotef))
)



(defrule remote-action-execution-start
  (wm-fact (key domain fact self args? r ?robot))
  ?remotef <- (wm-fact (key exec remote-action args? id ?action-id-sym
                                                     goal-id ?goal-id
                                                     plan-id ?plan-id
                                                     $?)
                       (values $?values&:(and (member$  r ?values)
                                              (member$ ?robot ?values))))
  (not (plan-action (id ?action-id&:(eq ?action-id (string-to-field (sym-cat ?action-id-sym))))
                    (goal-id ?goal-id)
                    (plan-id ?plan-id)))
 =>
  (assert-string (remote-action-to-plan-action-str ?remotef ))
)


(defrule remote-action-execution-update
  (wm-fact (key domain fact self args? r ?robot))
  ?actionf <- (plan-action (state ?state)
                           (id ?id)
                           (goal-id ?goal-id)
                           (plan-id ?plan-id)
                           (param-names $?p-names&:(member$ r ?p-names))
                           (param-values $?p-values&:(member$ ?robot ?p-values)))
  ?remotef <- (wm-fact (key $?key) (values $? state ?old-state $?))
  (test (wm-key-prefix ?key (create$ exec remote-action)))
  (test (eq (wm-key-arg ?key id) (sym-cat ?id)))
  (test (eq (wm-key-arg ?key goal-id) ?goal-id))
  (test (eq (wm-key-arg ?key plan-id) ?plan-id))
  (test (neq ?old-state ?state))
  =>
  (printout t "Updating remote-action fact (state " ?old-state
                 ") => (state " ?state ") " crlf)
  (bind ?values (remote-action-create-wm-values ?actionf))
  (modify ?remotef (values ?values))
)
