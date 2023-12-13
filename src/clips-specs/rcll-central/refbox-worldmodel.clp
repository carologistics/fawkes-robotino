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
(deffunction get-wm-fact-from-list-by-args (?wm-facts ?args ?values)
" Retrieve list of all wm-facts where the given arguments match the given
  values.
  @param ?wm-facts list of wm-facts
  @param ?args argument names to match
  @param ?values argument values to be matched
  @return The subset of ?wm-facts where all ?args match the corresponding
         ?values
"
    (if (neq (length$ ?args) (length$ ?values)) then (return FALSE))
    (foreach ?fact ?wm-facts
      (if (fact-relation ?fact)
       then ; the fact exists
        (bind ?res TRUE)
        (loop-for-count (?i 1 (length$ ?args))
          (if (neq (wm-key-arg (fact-slot-value ?fact key) (nth$ ?i ?args)) (nth$ ?i ?values))
           then (bind ?res FALSE) (break)
          )
        )
        (if ?res then (return ?fact))
      )
    )
    (return FALSE)
)


(deffunction get-fact-by-shelf-slot (?wm-facts ?shelf ?slot)
" filter those wm-facts which match the given storage position."
  (return (get-wm-fact-from-list-by-args ?wm-facts (create$ shelf slot) (create$ ?shelf ?slot)))
)

(deffunction refbox-get-storage-description (?description-string)
" Translate a storage description to a multifield of exactly six symbols:m
  (<WP-id> <base-color> <ring[1,2,3]-color> <cap-color>).
  If a description is empty or cannot be translated in that format an empty
  multifield is returned instead.
"
  (bind ?descriptions (explode$ ?description-string))
  (if (eq (length$ descriptions) 0) then (return ?descriptions))
  (bind ?description-length (length$ ?descriptions))
  (bind ?base-colors (create$ BASE_NONE))
  (bind ?ring-colors (create$ RING_NONE))
  (bind ?cap-colors (create$ CAP_NONE))
  (delayed-do-for-all-facts ((?col wm-fact))
    (wm-key-prefix ?col:key (create$ domain fact bs-color))
    (bind ?base-colors (insert$ ?base-colors 1 (wm-key-arg ?col:key col)))
  )
  (delayed-do-for-all-facts ((?col wm-fact))
    (wm-key-prefix ?col:key (create$ domain fact cs-color))
    (bind ?cap-colors (insert$ ?cap-colors 1 (wm-key-arg ?col:key col)))
  )
  (delayed-do-for-all-facts ((?col wm-fact))
    (wm-key-prefix ?col:key (create$ domain fact rs-ring-spec))
    (bind ?ring-colors (insert$ ?ring-colors 1 (wm-key-arg ?col:key r)))
  )
  (if (and (eq ?description-length 2)
           (member$ (nth$ 1 ?descriptions) ?base-colors)
           (member$ (nth$ 2 ?descriptions) ?cap-colors))
   then
    (return (create$ UNKNOWN-WP (nth$ 1 ?descriptions)
                     RING_NONE RING_NONE RING_NONE
                     (nth$ 2 ?descriptions)))
  )
  (if (and (eq ?description-length 6)
          ; (any-factp ((?obj domain-object))
          ;   (and (eq ?obj:name (nth$ 1 ?descriptions))
          ;        (eq ?obj:type workpiece))
          ; )
           (member$ (nth$ 2 ?descriptions) ?base-colors)
           (member$ (nth$ 3 ?descriptions) ?ring-colors)
           (member$ (nth$ 4 ?descriptions) ?ring-colors)
           (member$ (nth$ 5 ?descriptions) ?ring-colors)
           (member$ (nth$ 6 ?descriptions) ?cap-colors))
   then (return ?descriptions)
  )
  (printout error ?base-colors " " ?ring-colors " " ?cap-colors crlf)
  (printout error "Unexpected storage information received, expected format "
                  "() or (<base-col>, <cap-col>) or "
                  "(<wp>, <base-col>, <ring[1,2,3]-col>, <cap-col>)" crlf)
  (printout error "Got: " ?descriptions crlf)
  ; return empty descriptions to not introduce undefined behavior
  (return (create$))
)

(deffunction refbox-recv-ShelfSlotInfo  (?machine-message ?ss)
" Process the Storage Info and apply all observed changes to the worldmodel."
  ; gather the current knowledge of the storage station
  (bind ?stored-positions (find-all-facts ((?stored-wp wm-fact))
        (and (wm-key-prefix ?stored-wp:key (create$ domain fact ss-stored-wp))
             (eq (wm-key-arg ?stored-wp:key m) ?ss))))
  (bind ?free-positions (find-all-facts ((?pos-free wm-fact))
        (and (wm-key-prefix ?pos-free:key (create$ domain fact ss-shelf-slot-free))
             (eq (wm-key-arg ?pos-free:key m) ?ss))))
  (bind ?pending-updates (find-all-facts ((?needs-update wm-fact))
        (and (wm-key-prefix ?needs-update:key
                            (create$ domain fact ss-new-wp-at))
             (eq (wm-key-arg ?needs-update:key m) ?ss))))
  (bind ?unused-wps (find-all-facts ((?unused-wp wm-fact))
        (wm-key-prefix ?unused-wp:key
                            (create$ domain fact wp-unused))))
  ; process the update for each storage position
  (foreach ?o (pb-field-list ?machine-message "status_ss")
    (bind ?shelf (int-to-sym (pb-field-value ?o "shelf")))
    (bind ?slot (int-to-sym (pb-field-value ?o "slot")))
    (bind ?is-filled (pb-field-value ?o "is_filled"))
    (bind ?description (pb-field-value ?o "description"))
    ; lookup current knowledge about the storage position
    (bind ?our-is-filled (get-fact-by-shelf-slot ?stored-positions ?shelf ?slot))
    (bind ?our-is-not-filled (get-fact-by-shelf-slot ?free-positions ?shelf ?slot))
    (bind ?pending-update (get-fact-by-shelf-slot ?pending-updates ?shelf ?slot))
    ; update worldmodel if there is new info
    (if (and ?our-is-filled (not ?is-filled))
     then ; the storage position is now empty, update our storage info
      (retract ?our-is-filled)
      (bind ?stored-positions (remove-field ?our-is-filled ?stored-positions))
      (assert (wm-fact (key domain fact ss-shelf-slot-free
                        args? m ?ss shelf ?shelf slot ?slot)))
     else
      (if ?is-filled
       then ; the position is filled, it has a description to process
        (bind ?wp-info (refbox-get-storage-description ?description))
        (if (eq (length$ ?wp-info) 6)
         then ; the description is actually valid
          (bind ?wp (nth$ 1 ?wp-info))
          (bind ?base (nth$ 2 ?wp-info))
          (bind ?r1 (nth$ 3 ?wp-info))
          (bind ?r2 (nth$ 4 ?wp-info))
          (bind ?r3 (nth$ 5 ?wp-info))
          (bind ?cap (nth$ 6 ?wp-info))
          (bind ?wp-unused (get-wm-fact-from-list-by-args ?unused-wps
                                                          (create$ wp)
                                                          (create$ ?wp)))
          (if ?wp-unused
           then ; the workpiece is unusable but stored, treat it as a new
                ; workpiece so that it can be used again
            (assert (wm-fact (key domain fact ss-new-wp-at
                              args? m ?ss wp ?wp shelf ?shelf slot ?slot
                              base-col ?base ring1-col ?r1 ring2-col ?r2
                              ring3-col ?r3 cap-col ?cap)))
          )
          (if ?our-is-not-filled
           then ; this workpiece is not known, update it
            (bind ?free-positions (remove-field ?our-is-not-filled ?free-positions))
            (retract ?our-is-not-filled)
            (assert (wm-fact (key domain fact ss-stored-wp
                              args? m ?ss wp ?wp shelf ?shelf slot ?slot)))
           else
            ; the workpiece is known, process any pending update
            (if (and ?pending-update
                     (neq ?wp (wm-key-arg (fact-slot-value ?pending-update key)
                                           wp)))
             then
              (retract ?pending-update)
              (retract ?our-is-filled)
              (bind ?stored-positions (remove-field ?our-is-filled ?stored-positions))
              (assert (wm-fact (key domain fact ss-stored-wp
                                args? m ?ss wp ?wp shelf ?shelf slot ?slot)))
            )
          )
        )
      )
    )
  )
)




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
  ?pm <- (wm-fact (id "/refbox/points/MAGENTA"))
  ?pc <- (wm-fact (id "/refbox/points/CYAN"))
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
  (assert (wm-fact (id "/refbox/points/MAGENTA") (type UINT) (value (pb-field-value ?p "points_magenta")) ))
  (assert (wm-fact (id "/refbox/points/CYAN") (type UINT) (value (pb-field-value ?p "points_cyan")) ))

  ; assert the field height and width if it has not been asserted yet
  (if
    (not
      (any-factp ((?field-height wm-fact))
        (wm-key-prefix ?field-height:key (create$ refbox field height))
      )
    ) then
    (assert (wm-fact (id "/refbox/field/height") (type UINT) (value (pb-field-value ?p "field_height")) ))
    (assert (wm-fact (id "/refbox/field/width") (type UINT) (value (pb-field-value ?p "field_width")) ))
    (assert (wm-fact (id "/refbox/field/mirrored") (type UINT) (value (pb-field-value ?p "field_mirrored")) ))
  )
)


(defrule refbox-recv-product
  "Assert products sent by the refbox."
  ?pf <- (protobuf-msg (type "llsf_msgs.OrderInfo") (ptr ?ptr))
  (wm-fact (id "/refbox/team-color") (value ?team-color&:(neq ?team-color nil)))
  =>
  (foreach ?o (pb-field-list ?ptr "orders")
    (bind ?product-list (create$))
    (bind ?id (pb-field-value ?o "id"))
    (bind ?order-id (sym-cat O ?id))
    (foreach ?p (pb-field-list ?o "products")
      (bind ?id (pb-field-value ?p "pid"))
      (bind ?product-id (sym-cat ?order-id P ?id))
	  (bind ?product-list (append$ ?product-list ?product-id))
      ;check if the product is new
      (if (not (any-factp ((?wm-fact wm-fact)) (and (wm-key-prefix ?wm-fact:key (create$ domain fact product-complexity) )
                                                    (eq ?product-id (wm-key-arg ?wm-fact:key prod)))))
                                                    ; (eq ?product-id (nth$ (+ (member$ product ?wm-fact:key) 1) ?wm-fact:key))
          then
            (bind ?complexity (pb-field-value ?p "complexity"))
            (bind ?competitive (pb-field-value ?o "competitive"))
            (bind ?delivery-gate (pb-field-value ?o "delivery_gate"))
            (bind ?quantity-requested (pb-field-value ?o "quantity_requested"))
            (bind ?begin (pb-field-value ?o "delivery_period_begin"))
            (bind ?end (pb-field-value ?o "delivery_period_end"))
            (if (pb-has-field ?p "base_color") then
              (bind ?base (pb-field-value ?p "base_color"))
            else
              (bind ?base UNKNOWN)
            )
            (bind ?cap (pb-field-value ?p "cap_color"))
            (bind ?rings-count 1)
            (progn$ (?rc (pb-field-list ?p "ring_colors"))
			  (printout t "I have " ?rings-count " ring " ?rc crlf)
              (assert (wm-fact (key domain fact (sym-cat product- ring ?rings-count -color) args? prod ?product-id col ?rc) (type BOOL) (value TRUE) ))
              (bind ?rings-count (+ 1 ?rings-count))
            )
            (loop-for-count (?c ?rings-count 3) do
              (assert (wm-fact (key domain fact (sym-cat product- ring ?c -color) args? prod ?product-id col RING_NONE) (type BOOL) (value TRUE) ))
            )
            (assert
              (wm-fact (key domain fact product-complexity args? prod ?product-id comp ?complexity) (type BOOL) (value TRUE) )
              (wm-fact (key domain fact product-base-color args? prod ?product-id col ?base) (type BOOL) (value TRUE) )
              (wm-fact (key domain fact product-cap-color  args? prod ?product-id col ?cap) (type BOOL) (value TRUE) )
              (wm-fact (key domain fact product-gate  args? prod ?product-id gate (sym-cat GATE- ?delivery-gate)) (type BOOL) (value TRUE) )
              (wm-fact (key refbox product ?product-id quantity-requested) (type UINT) (value ?quantity-requested) )
              (wm-fact (key domain fact quantity-delivered args? prod ?product-id team CYAN)
                       (type UINT) (value 0))
              (wm-fact (key domain fact quantity-delivered args? prod ?product-id team MAGENTA)
                       (type UINT) (value 0))
              (wm-fact (key refbox product ?product-id delivery-begin) (type UINT) (value ?begin) )
              (wm-fact (key refbox product ?product-id delivery-end) (type UINT) (value ?end) )
              )
            (assert (wm-fact (key product meta competitive args? prod ?product-id)
                             (type BOOL) (is-list FALSE) (value ?competitive)))
            (printout t "Added product " ?id " with " (pb-field-value ?p "cap_color") crlf)
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
                     (create$ domain fact quantity-delivered args? prod ?product-id
                      team (mirror-team ?team-color)))
                   (neq ?old-qd-them:value ?qd-them))
                (modify ?old-qd-them (value ?qd-them))
            )
            (do-for-fact ((?old-qd-us wm-fact))
              (and (wm-key-prefix ?old-qd-us:key
                     (create$ domain fact quantity-delivered args? prod ?product-id
                      team ?team-color))
                   (neq ?old-qd-us:value ?qd-us))
                (modify ?old-qd-us (value ?qd-us))
            )
      )
	)
    (if (not (any-factp ((?wm-fact wm-fact)) (and (wm-key-prefix ?wm-fact:key (create$ order meta product-list) )
                                                    (eq ?order-id (wm-key-arg ?wm-fact:key ord)))))
	  then
	    (assert (wm-fact (key order meta product-list args? ord ?order-id) (is-list TRUE) (values ?product-list)))
	)
                                                    ; (eq ?product-id (nth$ (+ (member$ product ?wm-fact:key) 1) ?wm-fact:key))
  )
  (retract ?pf)
)

(defrule refbox-recv-MachineReportInfo
  ?pb-msg <- (protobuf-msg (type "llsf_msgs.MachineReportInfo") (ptr ?p))
  =>
  (bind ?machines (create$))

  (foreach ?m (pb-field-list ?p "reported_types")
    (bind ?m-name (sym-cat (pb-field-value ?m "name")))
    (if (and
          (any-factp ((?wm-fact wm-fact))
              (and (wm-key-prefix ?wm-fact:key (create$ domain fact mps-state))
                    (eq (wm-key-arg ?wm-fact:key m) ?m-name)
              )
          )
          (not
            (any-factp ((?wm-fact wm-fact))
                (and (wm-key-prefix ?wm-fact:key (create$ refbox explored-machine))
                      (eq (wm-key-arg ?wm-fact:key m) ?m-name)
                )
            )
          )
        )
      then
        (assert (wm-fact (key refbox explored-machine args? m ?m-name)))
    )
  )
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
    (if (eq ?m-type SS) then
      (refbox-recv-ShelfSlotInfo ?m ?m-name)
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
