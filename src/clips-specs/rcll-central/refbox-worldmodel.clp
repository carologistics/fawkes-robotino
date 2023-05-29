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


; DANGERZONE: NEVER use a foreach on a pb-field-list. It will crash. Use progn$ instead.

(defrule refbox-recv-BeaconSignal
  ?pf <- (protobuf-msg (type "llsf_msgs.BeaconSignal") (ptr ?p))
  (time $?now)
  =>
  (bind ?beacon-name (pb-field-value ?p "peer_name"))
  (printout debug "Beacon Recieved from " ?beacon-name crlf)
  (retract ?pf)
)


(defrule refbox-recv-GameState
  ?pf <- (protobuf-msg (type "llsf_msgs.GameState") (ptr ?p) (rcvd-from ?host ?port))
  ?gt <- (wm-fact (key refbox game-time))
  ?rp <- (wm-fact (id "/refbox/phase")  (value ?phase) )
  ?rs <- (wm-fact (id "/refbox/state")  (value ?state) )
  ?pm <- (wm-fact (id "/refbox/points/magenta"))
  ?pc <- (wm-fact (id "/refbox/points/cyan"))
  ?tc <- (wm-fact (id "/refbox/team-color")  (value ?team-color) )
  (wm-fact (key config agent team)  (value ?team-name) )
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
  (assert (wm-fact (key refbox game-time) (is-list TRUE) (type UINT) (values ?sec (/ ?nsec 1000))))
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
          (bind ?competitive (pb-field-value ?o "competitive"))
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
          (bind ?rings-count 0)
          (progn$ (?p (pb-field-list ?o "ring_colors"))
            (assert (wm-fact (key domain fact (sym-cat order- ring ?p-index -color) args? ord ?order-id col ?p) (type BOOL) (value TRUE) ))
            (bind ?rings-count ?p-index)
          )
          (loop-for-count (?c (+ 1 ?rings-count) 3) do
            (assert (wm-fact (key domain fact (sym-cat order- ring ?c -color) args? ord ?order-id col RING_NONE) (type BOOL) (value TRUE) ))
          )
          (assert
            (wm-fact (key domain fact order-complexity args? ord ?order-id com ?complexity) (type BOOL) (value TRUE) )
            (wm-fact (key domain fact order-base-color args? ord ?order-id col ?base) (type BOOL) (value TRUE) )
            (wm-fact (key domain fact order-cap-color  args? ord ?order-id col ?cap) (type BOOL) (value TRUE) )
            (wm-fact (key domain fact order-gate  args? ord ?order-id gate (sym-cat GATE- ?delivery-gate)) (type BOOL) (value TRUE) )
            (wm-fact (key refbox order ?order-id quantity-requested) (type UINT) (value ?quantity-requested) )
            (wm-fact (key domain fact quantity-delivered args? ord ?order-id team CYAN)
                     (type UINT) (value 0))
            (wm-fact (key domain fact quantity-delivered args? ord ?order-id team MAGENTA)
                     (type UINT) (value 0))
            (wm-fact (key refbox order ?order-id delivery-begin) (type UINT) (value ?begin) )
            (wm-fact (key refbox order ?order-id delivery-end) (type UINT) (value ?end) )
            )
          (assert (wm-fact (key order meta competitive args? ord ?order-id)
                           (type BOOL) (is-list FALSE) (value ?competitive)))
          (printout t "Added order " ?id " with " (pb-field-value ?o "cap_color") crlf)
      else
          (if (eq ?team-color CYAN) then
            (bind ?qd-them (pb-field-value ?o "quantity_delivered_magenta"))
            (bind ?qd-us (pb-field-value ?o "quantity_delivered_cyan"))
          else
            (bind ?qd-them (pb-field-value ?o "quantity_delivered_cyan"))
            (bind ?qd-us (pb-field-value ?o "quantity_delivered_magenta"))
          )
          (do-for-fact ((?old-qd-them wm-fact))
            (and (wm-key-prefix ?old-qd-them:key
                   (create$ domain fact quantity-delivered args? ord ?order-id
                    team (mirror-team ?team-color)))
                 (neq ?old-qd-them:value ?qd-them))
              (modify ?old-qd-them (value ?qd-them))
          )
          (do-for-fact ((?old-qd-us wm-fact))
            (and (wm-key-prefix ?old-qd-us:key
                   (create$ domain fact quantity-delivered args? ord ?order-id
                    team ?team-color))
                 (neq ?old-qd-us:value ?qd-us))
              (modify ?old-qd-us (value ?qd-us))
          )
    )
  )
  (retract ?pf)
)


(defrule refbox-recv-MachineInfo
  ?pb-msg <- (protobuf-msg (type "llsf_msgs.MachineInfo") (ptr ?p))
  (wm-fact (id "/refbox/team-color") (value ?team-color&:(neq ?team-color nil)))
  =>
  ; (printout t "***** Received MachineInfo *****" crlf)
  (bind ?machines (create$)) ;keep track of the machines that actually exist

  (foreach ?m (pb-field-list ?p "machines")
    (bind ?m-name (sym-cat (pb-field-value ?m "name")))
    (bind ?machines (insert$ ?machines(+ (length$ ?machines) 1) ?m-name))
    (bind ?m-type (sym-cat (pb-field-value ?m "type")))
    (bind ?m-team (sym-cat (pb-field-value ?m "team_color")))
    (bind ?m-state (sym-cat (pb-field-value ?m "state")))
    (if (not (any-factp ((?wm-fact wm-fact))
              (and  (wm-key-prefix ?wm-fact:key (create$ domain fact mps-state))
                    (eq ?m-name (wm-key-arg ?wm-fact:key m)))))
      then
      (if (eq ?team-color ?m-team) then
        (assert (wm-fact (key domain fact mps-state args? m ?m-name s ?m-state) (type BOOL) (value TRUE) ))
      )
    ; set available rings for ring-stations
      (if (eq ?m-type RS) then
        (progn$ (?rc (pb-field-list ?m "ring_colors"))
          (assert (wm-fact (key domain fact rs-ring-spec args? m ?m-name r ?rc rn NA) (type BOOL) (value TRUE)))
        )
      )
    )
   (do-for-fact ((?wm-fact wm-fact))
                  (and  (wm-key-prefix ?wm-fact:key (create$ domain fact mps-state))
                        (eq ?m-name (wm-key-arg ?wm-fact:key m))
                        (neq ?m-state (wm-key-arg ?wm-fact:key s)))
      (retract ?wm-fact)
      (assert (wm-fact (key domain fact mps-state args? m ?m-name s ?m-state) (type BOOL) (value TRUE)))
    )
  )
  ;remove machines that do not exist
  (do-for-all-facts ((?wm-fact wm-fact))
      (or   (and  (wm-key-prefix ?wm-fact:key (create$ domain fact mps-team))
              (not (member$ (wm-key-arg ?wm-fact:key m) ?machines))
            )
            (and  (wm-key-prefix ?wm-fact:key (create$ domain fact mps-side-free))
              (not (member$ (wm-key-arg ?wm-fact:key m) ?machines))
            )
            (and  (wm-key-prefix ?wm-fact:key (create$ domain fact mps-state))
              (not (member$ (wm-key-arg ?wm-fact:key m) ?machines))
            )
            (and  (wm-key-prefix ?wm-fact:key (create$ domain fact mps-type))
              (not (member$ (wm-key-arg ?wm-fact:key m) ?machines))
            )
            (and  (wm-key-prefix ?wm-fact:key (create$ domain fact cs-color))
              (not (member$ (wm-key-arg ?wm-fact:key m) ?machines))
            )
            (and  (wm-key-prefix ?wm-fact:key (create$ domain fact rs-ring-spec))
              (not (member$ (wm-key-arg ?wm-fact:key m) ?machines))
              (neq (wm-key-arg ?wm-fact:key r) RING_NONE)
            )
      )
      (retract ?wm-fact)
  )
)



(defrule game-receive-field-layout-protobuf
"At the end of the exploration phase, the Refbox sends the true field layout.
 Assert a field-ground-truth fact for each machine the Refbox told us about."
  ; (declare (salience ?*PRIORITY-LOW*))
  (not (wm-fact (key refbox field-ground-truth complete) (value TRUE)))
  ?msg <- (protobuf-msg (type "llsf_msgs.MachineInfo") (ptr ?p))
=>
  (bind ?rcv-ground-truth FALSE)
  (bind ?incomplete-ground-truth FALSE)
  (foreach ?machine (pb-field-list ?p "machines")
    (bind ?name (sym-cat (pb-field-value ?machine "name")))
    (bind ?rot  FALSE)
    (bind ?zone FALSE)
    (bind ?type FALSE)
    (if (pb-has-field ?machine "rotation") then
      (bind ?rot  (pb-field-value ?machine "rotation"))
    )
    (if (pb-has-field ?machine "zone") then
      (bind ?zone (clips-name (pb-field-value ?machine "zone")))
    )
    (if (pb-has-field ?machine "type") then
      (bind ?type (sym-cat (pb-field-value ?machine "type")))
    )
    (if (and ?zone ?rot ?type (neq ?rot NOT-SET) (neq ?zone N-T-SET)) then
       (printout t "Received ground-truth for Machine: " ?name
        ", rot: " ?rot ", zone: " ?zone ", type: " ?type crlf)
      (bind ?yaw (deg-to-rad ?rot))
      (delayed-do-for-all-facts ((?old-gt wm-fact))
        (and (wm-key-prefix ?old-gt:key
                            (create$ refbox field-ground-truth))
             (eq (wm-key-arg ?old-gt:key m) ?name))
        (retract ?old-gt)
      )
      (assert
        (wm-fact (key refbox field-ground-truth name args? m ?name) (type BOOL) (value TRUE))
        (wm-fact (key refbox field-ground-truth mtype args? m ?name) (value ?type))
        (wm-fact (key refbox field-ground-truth zone args? m ?name) (value ?zone))
        (wm-fact (key refbox field-ground-truth yaw args? m ?name) (type FLOAT) (value ?yaw))
        (wm-fact (key refbox field-ground-truth orientation args? m ?name) (type FLOAT) (value ?rot))
      )
      (bind ?rcv-ground-truth TRUE)
    else
      (bind ?incomplete-ground-truth TRUE)
      (assert (wm-fact (key refbox field-ground-truth mtype args? m ?name) (value ?type)))
      (assert (wm-fact (key refbox field-ground-truth name args? m ?name) (type BOOL) (value FALSE)))
    )
  )
  (if (and ?rcv-ground-truth
            (not ?incomplete-ground-truth))
   then
   (printout t "Received ground-truth complete"crlf)
    (assert (wm-fact (key refbox field-ground-truth complete) (type BOOL) (value TRUE)))
  )
  ; (retract ?msg)
)


(defrule refbox-recv-RingInfo
  ?pf <- (protobuf-msg (type "llsf_msgs.RingInfo") (ptr ?p))
  =>
  (foreach ?r (pb-field-list ?p "rings")
    (bind ?color (pb-field-value ?r "ring_color"))
    (bind ?raw-material (pb-field-value ?r "raw_material"))
    (bind ?rn ZERO)
    (if (eq ?raw-material 1) then (bind ?rn ONE))
    (if (eq ?raw-material 2) then (bind ?rn TWO))
    (if (eq ?raw-material 3) then (bind ?rn THREE))
    (do-for-all-facts ((?wm-fact wm-fact))
                  (and  (wm-key-prefix ?wm-fact:key (create$ domain fact rs-ring-spec))
                        (eq ?color (wm-key-arg ?wm-fact:key r))
                        (neq ?rn (wm-key-arg ?wm-fact:key rn)))
      (bind ?mps (wm-key-arg ?wm-fact:key m))
      (retract ?wm-fact)
      (assert (wm-fact (key domain fact rs-ring-spec args? m ?mps r ?color rn ?rn) (type BOOL) (value TRUE)))
    )
  )
)

(defrule refbox-recv-RobotInfo
  "Receive robot state information to detect if a robot is placed into (or
	recovered from) maintenance."
  ?pf <- (protobuf-msg (type "llsf_msgs.RobotInfo") (ptr ?r))
  =>
  (foreach ?p (pb-field-list ?r "robots")
    (bind ?state (sym-cat (pb-field-value ?p "state")))
    (bind ?robot (sym-cat (pb-field-value ?p "name")))
    (bind ?old-state nil)
    (do-for-fact ((?wm wm-fact)) (wm-key-prefix ?wm:key (create$ monitoring state args? r ?robot))
      (bind ?old-state ?wm:value)
      (retract ?wm)
    )
    (assert (wm-fact (key monitoring state args? r ?robot) (is-list FALSE) (type SYMBOL) (value ?state)))
    (if (and (eq ?old-state MAINTENANCE)
             (eq ?state ACTIVE))
     then
      (assert (wm-fact (key central agent robot-waiting args? r ?robot)))
    )
    (if (and (eq ?old-state ACTIVE)
             (neq ?state ACTIVE))
     then
      (assert (reset-robot-in-wm ?robot))
    )
  )
)

(defrule refbox-recv-NavigationRoutes-initialize
  "When there are no waypoints,reached and remaining facts, initialize them based on
  the NavigationRoutes message from the refbox."
  ?pf <- (protobuf-msg (type "llsf_msgs.NavigationRoutes") (ptr ?p))
  (not (wm-fact (key domain fact waypoints $?)))
  (not (wm-fact (key domain fact reached $?)))
  (not (wm-fact (key domain fact remaining $?)))
  =>
  (bind ?waypoints (create$))
  (bind ?remaining (create$))
  (bind ?reached   (create$))

  (foreach ?r (pb-field-list ?p "routes")
    (progn$ (?waypoint (pb-field-list ?r "route"))
      (bind ?waypoints (append$ ?waypoints (clips-name ?waypoint)))
    )
    (progn$ (?waypoint (pb-field-list ?r "reached"))
      (bind ?reached (append$ ?reached (clips-name ?waypoint)))
    )
    (progn$ (?waypoint (pb-field-list ?r "remaining"))
      (bind ?remaining (append$ ?remaining (clips-name ?waypoint)))
    )
  )

  (assert (wm-fact (key domain fact waypoints args?) (is-list TRUE) (values $?waypoints)))
  (assert (wm-fact (key domain fact reached args?) (is-list TRUE) (values $?reached)))
  (assert (wm-fact (key domain fact remaining args?)  (is-list TRUE) (values $?remaining)))
)

(defrule refbox-recv-NavigationRoutes-update
  "When there are waypoints, reached and remaining facts, compare the incoming new lists
  from the refbox with the existing facts and update them if necessary. "
  ?pf <- (protobuf-msg (type "llsf_msgs.NavigationRoutes") (ptr ?p))
  ?waypoints-fact <- (wm-fact (key domain fact waypoints args?) (values $?waypoints-old))
  ?reached-fact <- (wm-fact (key domain fact reached args?) (values $?reached-old))
  ?remaining-fact <- (wm-fact (key domain fact remaining args?) (values $?remaining-old))
  =>
  (bind ?waypoints (create$))
  (bind ?remaining (create$))
  (bind ?reached   (create$))

  (foreach ?r (pb-field-list ?p "routes")
    (progn$ (?waypoint (pb-field-list ?r "route"))
      (bind ?waypoints (append$ ?waypoints ?waypoint))
    )
    (progn$ (?waypoint (pb-field-list ?r "reached"))
      (bind ?reached (append$ ?reached ?waypoint))
    )
    (progn$ (?waypoint (pb-field-list ?r "remaining"))
      (bind ?remaining (append$ ?remaining ?waypoint))
    )
  )
  (if (neq ?waypoints-old ?waypoints) then
    (retract ?waypoints-fact)
    (assert (wm-fact (key domain fact waypoints args?) (is-list TRUE) (values $?waypoints)))
  )
  (if (neq ?remaining-old ?remaining) then
    (retract ?remaining-fact)
    (assert (wm-fact (key domain fact remaining args?) (is-list TRUE) (values $?remaining)))
  )
  (if (neq ?reached-old ?reached) then
    (retract ?reached-fact)
    (assert (wm-fact (key domain fact reached args?) (is-list TRUE) (values $?reached)))
  )
)
