;---------------------------------------------------------------------------
;  refbox-worldmodel.clp - Update the worlmodel with RefBox info
;
;  Created: Thu 11 Jan 2018 14:48:30 CET
;  Copyright  2018  Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>
;                   Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

(defrule refbox-recv-BeaconSignal
  ?pf <- (protobuf-msg (type "llsf_msgs.BeaconSignal") (ptr ?p))
  (time $?now)
  =>
  (bind ?beacon-name (pb-field-value ?p "peer_name"))
  (printout t "Beacon Recieved from " ?beacon-name crlf)
  (retract ?pf) 
)


(defrule refbox-recv-GameState
  ?pf <- (protobuf-msg (type "llsf_msgs.GameState") (ptr ?p) (rcvd-from ?host ?port))
  ?gt <- (wm-fact (key refbox game-time $?args )  (value TRUE))
  ?rp <- (wm-fact (id "/refbox/phase")  (value ?phase) )
  ?rs <- (wm-fact (id "/refbox/state")  (value ?state) )
  ?pm <- (wm-fact (id "/refbox/points/magenta"))
  ?pc <- (wm-fact (id "/refbox/points/cyan")) 
  ?tc <- (wm-fact (id "/refbox/team-color")  (value ?team-color) )
  (wm-fact (id "/config/rcll/team-name")  (value ?team-name) )
  =>
  (retract ?pf ?gt ?pm ?pc)
  (bind ?new-state (pb-field-value ?p "state"))
  (bind ?new-phase (pb-field-value ?p "phase"))
  (bind ?new-team-color ?team-color)
  (if (and (pb-has-field ?p "team_cyan")
           (eq (pb-field-value ?p "team_cyan") ?team-name))
    then (bind ?new-team-color CYAN))
  (if (and (pb-has-field ?p "team_magenta")
           (eq (pb-field-value ?p "team_magenta") ?team-name))
    then (bind ?new-team-color MAGENTA))
  (if (neq ?new-team-color ?team-color) then
    (printout warn "Switching team color from " ?team-color " to " ?new-team-color crlf)
    (retract ?tc)
    (assert  (wm-fact (id "/refbox/team-color")  (value ?new-team-color) ))
  )
  (if (neq ?phase ?new-phase) then
    (retract ?rp)
    (assert  (wm-fact (id "/refbox/phase")  (value ?new-phase) ))
  )
  (if (neq ?state ?new-state) then
    (retract ?rs)
    (assert  (wm-fact (id "/refbox/state")  (value ?new-state) ))
  )
  (bind ?time (pb-field-value ?p "game_time"))
  (bind ?sec (pb-field-value ?time "sec"))
  (bind ?nsec (pb-field-value ?time "nsec"))
  (assert (wm-fact (key refbox game-time args? sec ?sec nsec ?nsec ) (type BOOL) (value TRUE)) )
  (assert (wm-fact (id "/refbox/points/magenta") (type UINT) (value (pb-field-value ?p "points_magenta")) ))
  (assert (wm-fact (id "/refbox/points/cyan") (type UINT) (value (pb-field-value ?p "points_cyan")) ))
)


(defrule refbox-recv-order
  "Assert orders sent by the refbox."
  ?pf <- (protobuf-msg (type "llsf_msgs.OrderInfo") (ptr ?ptr))
  (wm-fact (id "/refbox/team-color") (value ?team-color&:(neq ?team-color nil)))
  =>
  (foreach ?o (pb-field-list ?ptr "orders")
    (bind ?id (pb-field-value ?o "id"))
    (bind ?order-id (sym-cat O ?id))
    ;check if the order is new
    (if (not (any-factp ((?wm-fact wm-fact)) (and (wm-key-prefix ?wm-fact:key (create$ domain fact order-complexity) )
                                                  (eq ?order-id (wm-key-arg ?wm-fact:key ord)))))
                                                  ; (eq ?order-id (nth$ (+ (member$ order ?wm-fact:key) 1) ?wm-fact:key))
        then
          (bind ?complexity (pb-field-value ?o "complexity"))
          (bind ?delivery-gate (pb-field-value ?o "delivery_gate"))
          (bind ?quantity-requested (pb-field-value ?o "quantity_requested"))
          (bind ?begin (pb-field-value ?o "delivery_period_begin"))
          (bind ?end (pb-field-value ?o "delivery_period_end"))
          (if (pb-has-field ?o "base_color") then
            (bind ?base (pb-field-value ?o "base_color"))
          else
            (bind ?base UNKNOWN)
          )
          (bind ?cap (pb-field-value ?o "cap_color"))
          (progn$ (?p (pb-field-list ?o "ring_colors"))
            (assert (wm-fact (key domain fact (sym-cat order- ring ?p-index -color) args? ord ?order-id col ?p) (type BOOL) (value TRUE) ))
          )
          (assert 
            (wm-fact (key domain fact order-complexity args? ord ?order-id comp ?complexity) (type BOOL) (value TRUE) )
            (wm-fact (key domain fact order-base-color args? ord ?order-id col ?base) (type BOOL) (value TRUE) )
            (wm-fact (key domain fact order-cap-color  args? ord ?order-id col ?cap) (type BOOL) (value TRUE) )
            (wm-fact (key domain fact order-gate  args? ord ?order-id gate (sym-cat GATE- ?delivery-gate)) (type BOOL) (value TRUE) )
            (wm-fact (key refbox order ?order-id quantity-requested) (type UINT) (value ?quantity-requested) )
            (wm-fact (key refbox order ?order-id delivery-begin) (type UINT) (value ?begin) )
            (wm-fact (key refbox order ?order-id delivery-end) (type UINT) (value ?end) )
            )
          (printout t "Added order " ?id " with " (pb-field-value ?o "cap_color") crlf)
      else
          (if (eq ?team-color CYAN) then
            (bind ?quantity-delivered (pb-field-value ?o "quantity_delivered_cyan"))
          else
            (bind ?quantity-delivered (pb-field-value ?o "quantity_delivered_magenta"))
          )      
          (do-for-fact ((?old-deliverd-fact wm-fact)) (wm-key-prefix ?old-deliverd-fact:key (create$ refbox order ?order-id quantity-delivered ?team-color)) 
              (retract ?old-deliverd-fact)
          )
          (assert (wm-fact (key refbox order ?order-id quantity-delivered ?team-color) (type UINT) (value ?quantity-delivered) ))
    )
  )
  (retract ?pf)
)

(defrule refbox-recv-MachineInfo
  ?pb-msg <- (protobuf-msg (type "llsf_msgs.MachineInfo") (ptr ?p))
  (wm-fact (id "/refbox/team-color") (value ?team-color&:(neq ?team-color nil)))
  =>
  ; (printout t "***** Received MachineInfo *****" crlf)
  (retract ?pb-msg)
  (foreach ?m (pb-field-list ?p "machines")
    (bind ?m-name (sym-cat (pb-field-value ?m "name")))
    (bind ?m-type (sym-cat (pb-field-value ?m "type")))
    (bind ?m-team (sym-cat (pb-field-value ?m "team_color")))
    (bind ?m-state (sym-cat (pb-field-value ?m "state")))
    (if (not (any-factp ((?wm-fact wm-fact)) 
              (and  (wm-key-prefix ?wm-fact:key (create$ domain fact mps-type)) 
                    (eq ?m-name (wm-key-arg ?wm-fact:key m)))))
      then
      (if (eq ?team-color ?m-team) then
        (assert (wm-fact (key domain fact mps-type args? m ?m-name t ?m-type) (type BOOL) (value TRUE) ))
        (assert (wm-fact (key domain fact mps-state args? m ?m-name s ?m-state) (type BOOL) (value TRUE) )) 
      )
    ; set available rings for ring-stations
      (if (eq ?m-type RS) then
        (progn$ (?rc (pb-field-list ?m "ring_colors"))
          (assert (wm-fact (key domain fact rs-ring-spec args? m ?m-name r ?rc) (type BOOL) (value TRUE))) 
        )
      )
    )
   (do-for-fact ((?wm-fact wm-fact)) 
                  (and  (wm-key-prefix ?wm-fact:key (create$ domain fact mps-state)) 
                        (eq ?m-name (wm-key-arg ?wm-fact:key m))
                        (neq ?m-state (wm-key-arg ?wm-fact:key s)))
      (retract ?wm-fact)
      (assert (wm-fact (key domain fact mps-state args? m ?m-name s ?m-state))) 
    )
   )
)
