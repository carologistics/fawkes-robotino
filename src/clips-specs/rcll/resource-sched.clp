;---------------------------------------------------------------------------
;  resources.clp - scheduling model resources
;
;  Created: Tue 08 Mar 2019 17:03:31 CET
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
;

(deftemplate resource-info
  (slot type (type SYMBOL))
  (slot consumable (type SYMBOL) (allowed-values TRUE FALSE) (default FALSE))
  (slot producible (type SYMBOL) (allowed-values TRUE FALSE) (default FALSE))
  (multislot state-preds (type SYMBOL))
  (multislot setup-preds (type SYMBOL))
)

(deftemplate setup-duration
  (slot resource-type (type SYMBOL))
  (multislot setup-from (type SYMBOL))
  (multislot setup-to (type SYMBOL))
  (slot duration (type INTEGER))
)


(defrule resources-initilize
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 ; config scheduling is enabled
 ; CX initialzied
 ; domain is rcll
 (not (resource-info))
=>
 (assert
  (resource-info (type robot)
                 (setup-preds (create$ at))
                 (state-preds (create$ can-hold is-holding)))

  (resource-info (type mps)
                 (setup-preds (create$))
                 (state-preds (create$ mps-type
                                       wp-at
                                       mps-state
                                       mps-side-free
                                       bs-prepared-color
                                       bs-prepare-side
                                       cs-can-perform
                                       cs-prepared-for
                                       cs-buffered
                                       cs-color
                                       cs-free
                                       rs-prepared-for
                                       rs-prepared-color
                                       ds-prepared-order))
  )

  (resource-info (type workpiece)
                 (consumable TRUE)
                 (setup-preds (create$))
                 (state-preds (create$ wp-at
                                       wp-base-color
                                       wp-ring1-color
                                       wp-ring2-color
                                       wp-ring3-color
                                       wp-cap-color))
  )

  (resource-info (type cap-carrier)
                 (consumable TRUE)
                 (producible TRUE)
                 (setup-preds (create$))
                 (state-preds (create$ wp-at
                                       wp-base-color
                                       wp-ring1-color
                                       wp-ring2-color
                                       wp-ring3-color
                                       wp-cap-color))
  )

 )
)


(deffunction statement-by-name (?statements ?pred-name)
   (bind ?pred-i (member$ ?pred-name ?statements ))
   (if (eq ?pred-i FALSE) then (return FALSE))
   (bind ?closing-i (wm-key-multifield-arg-end ?statements (- ?pred-i 2)))
   (printout t "statement by name " (subseq$ ?statements ?pred-i (- ?closing-i 1)) crlf) 
   (return (subseq$ ?statements ?pred-i (- ?closing-i 1)))
   
)

(deffunction statements-first$ (?statements)
   (bind ?opening-i (member$ [ ?statements ))
   (if (eq ?opening-i FALSE) then (return (create$)))
   (bind ?closing-i (wm-key-multifield-arg-end ?statements (- ?opening-i 1)))
   ;(printout t "statemnets-first$ " (subseq$ ?statements (+ ?opening-i 1) (- ?closing-i 1)) crlf)
   (return (subseq$ ?statements (+ ?opening-i 1) (- ?closing-i 1)))   
)

(deffunction statements-rest$ (?statements)
   (bind ?opening-i (member$ [ ?statements ))
   (bind ?L (length$ ?statements ))
   (if (eq ?opening-i FALSE) then (return (create$)))
   (bind ?closing-i (wm-key-multifield-arg-end ?statements (- ?opening-i 1)))
   ;(printout t "statemnet-rest$" (subseq$ ?statements (+ ?closing-i 1) ?L) crlf)
   (return (subseq$ ?statements (+ ?closing-i 1) ?L))   
)

(deffunction estimate-setup-duration (?r-type ?setup-from ?setup-to)
  (do-for-fact ((?rs setup-duration)) (and (eq ?rs:resource-type ?r-type)
                                           (eq ?rs:setup-from ?setup-from)
                                           (eq ?rs:setup-to ?setup-to))
                (return ?rs:duration))

  (bind ?duration 0)
  (if (member$ [ ?setup-to) then
     (progn$ (?opening-i (create$ (member$ [ ?setup-to)))
       (bind ?pred-name (nth$ (+ 1 ?opening-i) ?setup-to))
       (bind ?setup-state-to (statement-by-name ?setup-to ?pred-name ))
       (bind ?setup-state-from (statement-by-name ?setup-from ?pred-name ))

       (if (and (eq  ?r-type robot) (eq ?pred-name at) (neq ?setup-state-from FALSE)) then
           (bind ?duration (+ ?duration
                              (integer (round (estimate-action-duration "move"
                                              (create$ r from from-side to to-side)
                                              (create$ ANY
                                                      (rest$ ?setup-state-from)
                                                      (rest$ ?setup-state-to)))))))
       )
     )
  )

  (assert (setup-duration (resource-type ?r-type)
                          (setup-from ?setup-from)
                          (setup-to ?setup-to)
                          (duration ?duration)))

  (return ?duration)
)

