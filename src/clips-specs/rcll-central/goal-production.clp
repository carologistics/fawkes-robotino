;---------------------------------------------------------------------------
;  goal-production.clp - Generate production goals of RCLL
;
;  Created: Tue 09 Jan 2018 17:03:31 CET
;  Copyright  2018  Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>
;             2021  Tarik Viehmann <viehmann@kbsg.rwth-aachen.de>
;             2021  Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
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

; ----------------------- Util -------------------------------

(defglobal
  ?*PRODUCTION-C0-PRIORITY* = 30
  ?*PRODUCTION-C1-PRIORITY* = 40
  ?*PRODUCTION-C2-PRIORITY* = 50
  ?*PRODUCTION-C3-PRIORITY* = 60
  ?*PRODUCTION-NOTHING-EXECUTABLE-TIMEOUT* = 30
  ?*PRODUCTION-TIME-INTERVAL* = 120
)
(deffunction generate-all-order-list ()
    "generate a list containing all order-ids"
    (bind ?order-list (create$))
    (do-for-all-facts 
        ((?order wm-fact))
        (wm-key-prefix ?order:key (create$ domain fact order-complexity))
        (bind ?order-list (insert$ ?order-list 1 (wm-key-arg ?order:key ord))))
    (return ?order-list)
)
(deffunction generate-candidate-order-list ()
    "generate a list containing all order-ids, whose order has not been worked on yet"
    (bind ?order-list (create$))
    (do-for-all-facts 
        ((?order wm-fact))
        (and (wm-key-prefix ?order:key (create$ domain fact order-complexity))
            (not (any-factp 
                    ((?or wm-fact))
                    (and (wm-key-prefix ?or:key (create$ order meta wp-for-order))
                        (eq (wm-key-arg ?order:key ord) (wm-key-arg ?or:key ord))
            )))) 
        (bind ?order-list (insert$ ?order-list 1 (wm-key-arg ?order:key ord))))
    (loop-for-count (round (* (length$ ?order-list) (/ 1 2)))
      (bind ?order-list (create$ ?order-list FALSE)))
    ; (return (create$ ?order-list FALSE))
    (return ?order-list)
)

(deffunction pick-random-order (?order-list)
    "generate a random integer, return the order-id on this integer position"
    (bind ?len (length$ ?order-list))
    (if (> ?len 0) 
    then (return (nth$ (random 1 ?len) ?order-list)))
    (return nil)
)

(deffunction goal-meta-assign-robot-to-goal (?goal ?robot)
"Changes an existing goal-meta fact and assign it to the given robot"
  (if (eq (fact-slot-value ?goal id) FALSE) then
    (printout t "Goal has no id! " ?goal crlf)
    (return)
  )
  (if (eq ?robot nil) then (return ))
  (if (not (do-for-fact ((?f goal-meta))
      (and (eq ?f:goal-id (fact-slot-value ?goal id))
           (or (eq ?f:restricted-to ?robot)
               (eq ?f:restricted-to nil)))
      (modify ?f (assigned-to ?robot))))
   then
    (printout t "FAILED assign robot " ?robot " to goal "
      (fact-slot-value ?goal id) crlf)
  )
)

(deffunction goal-meta-assert (?goal ?robot ?order-id ?ring-nr)
"Creates the goal-meta fact, assigns the goal to the robot and to its order"
  (assert (goal-meta (goal-id (fact-slot-value ?goal id))
                     (assigned-to ?robot)
                     (order-id ?order-id)
                     (ring-nr ?ring-nr)))
  (return ?goal)
)

(deffunction goal-meta-assert-restricted (?goal ?robot)
"Creates the goal-meta fact and restricts the goal to the robot"
  (if (neq ?robot nil) then
    (assert (goal-meta (goal-id (fact-slot-value ?goal id))
                       (restricted-to ?robot)))
  )
)

(deffunction goal-production-get-machine-for-color
  (?col-ring)

  (bind ?rs FALSE)
  (do-for-all-facts ((?mps-type wm-fact)) (and (wm-key-prefix ?mps-type:key (create$ domain fact mps-type))
                                               (eq (wm-key-arg ?mps-type:key t) RS))
    (bind ?machine (wm-key-arg ?mps-type:key m))
    (do-for-fact ((?rs-ring-spec wm-fact)) (and (wm-key-prefix ?rs-ring-spec:key (create$ domain fact rs-ring-spec))
                                                (eq (wm-key-arg ?rs-ring-spec:key m) ?machine)
                                                (eq (wm-key-arg ?rs-ring-spec:key r) ?col-ring))
      (bind ?rs ?machine)
    )
  )
  (return ?rs)
)

(deffunction trim-machine-name (?machine)
  ""
  (return (sym-cat (sub-string 3 5 (str-cat ?machine)))) 
)

(deffunction transfer-ring-requirements-into-number (?ring-re)
  ""
  (bind ?num 0)
  (if (eq ?ring-re ZERO) then (bind ?num 0))
  (if (eq ?ring-re ONE) then (bind ?num 1))
  (if (eq ?ring-re TWO) then (bind ?num 2))
  (return ?num)
)

(deffunction calculate-overall-workload-for-each-processing-order (?team-name)
  "for cs1, cs2, rs1, rs2"
  (bind ?cs1 0)
  (bind ?cs2 0)
  (bind ?rs1 0)
  (bind ?rs2 0)
  (do-for-fact ((?mps-workload wm-fact))
    (wm-key-prefix ?mps-workload:key (create$ mps workload overall args? m (sym-cat ?team-name CS1)))
    (bind ?cs1 ?mps-workload:value)
  )
  (do-for-fact ((?mps-workload wm-fact))
    (wm-key-prefix ?mps-workload:key (create$ mps workload overall args? m (sym-cat ?team-name CS2)))
    (bind ?cs2 ?mps-workload:value)
  )
  (do-for-fact ((?mps-workload wm-fact))
    (wm-key-prefix ?mps-workload:key (create$ mps workload overall args? m (sym-cat ?team-name RS1)))
    (bind ?rs1 ?mps-workload:value)
  )
  (do-for-fact ((?mps-workload wm-fact))
    (wm-key-prefix ?mps-workload:key (create$ mps workload overall args? m (sym-cat ?team-name RS2)))
    (bind ?rs2 ?mps-workload:value)
  )
  (return (create$ ?cs1 ?cs2 ?rs1 ?rs2))
)

(deffunction calculate-decreased-workload-for-each-processing-order (?next-step ?c ?cs ?r1 ?r2 ?r3)
  "if an C3 order's next step is ring2, the RS workload for ring1 should be taken away from workload overall"
  (bind ?res (create$ 0 0 0 0))
  (bind ?cs1 0)
  (bind ?cs2 0)
  (bind ?rs1 0)
  (bind ?rs2 0)
  (printout t ?next-step " " ?c " " ?cs " " ?r1 " " ?r2 " " ?r3 crlf)

  (if (eq ?next-step DELIVER)
  then
    (if (eq ?cs CS1)
    then
      (bind ?cs1 (+ ?cs1 1))
    else 
      (bind ?cs2 (+ ?cs2 1))
    )
  )
  
  (if (or (and (eq ?next-step CAP) (eq ?c C3)) 
          (and (eq ?next-step DELIVER) (eq ?c C3))) 
  then
    ; the first ring ?r1
    (if (eq (nth$ 1 ?r1) RS1)
    then
      (bind ?rs1 (+ ?rs1 (nth$ 2 ?r1)))
    else
      (bind ?rs2 (+ ?rs2 (nth$ 2 ?r1)))
    )  
    ; the second ring ?r2
    (if (eq (nth$ 1 ?r2) RS1)
    then
      (bind ?rs1 (+ ?rs1 (nth$ 2 ?r2)))
    else
      (bind ?rs2 (+ ?rs2 (nth$ 2 ?r2)))
    )
    ; the third ring ?r3
    (if (eq (nth$ 1 ?r3) RS1)
    then
      (bind ?rs1 (+ ?rs1 (nth$ 2 ?r3)))
    else
      (bind ?rs2 (+ ?rs2 (nth$ 2 ?r3)))
    )
  )

  (if (or (eq ?next-step RING3)
          (and (eq ?next-step CAP) (eq ?c C2))
          (and (eq ?next-step DELIVER) (eq ?c C2)))
  then
    ; the first ring ?r1
    (if (eq (nth$ 1 ?r1) RS1)
    then
      (bind ?rs1 (+ ?rs1 (nth$ 2 ?r1)))
    else
      (bind ?rs2 (+ ?rs2 (nth$ 2 ?r1)))
    )  
    ; the second ring ?r2
    (if (eq (nth$ 1 ?r2) RS1)
    then
      (bind ?rs1 (+ ?rs1 (nth$ 2 ?r2)))
    else
      (bind ?rs2 (+ ?rs2 (nth$ 2 ?r2)))
    )
  )

  (if (or (eq ?next-step RING2)
          (and (eq ?next-step CAP) (eq ?c C1))
          (and (eq ?next-step DELIVER) (eq ?c C1)))
  then
    ; the first ring ?r1
    (if (eq (nth$ 1 ?r1) RS1)
    then
      (bind ?rs1 (+ ?rs1 (nth$ 2 ?r1)))
    else
      (bind ?rs2 (+ ?rs2 (nth$ 2 ?r1)))
    )  
  )

  (if (or (eq ?next-step RING1)
          (and (eq ?next-step CAP) (eq ?c C0))
          (and (eq ?next-step DELIVER) (eq ?c C0)))
  then
    (printout t "Nothing done!!" crlf)
  )
  (bind ?res (create$ ?cs1 ?cs2 ?rs1 ?rs2))
  ; (printout t "the return value is " ?res crlf)
  (return ?res)
)

(deffunction print-all-useful-parameters-at-every-decision-moment ()
    ""
    (bind ?order-list (generate-all-order-list))
    (bind ?candidate-list (generate-candidate-order-list))

    ; game time
    (do-for-fact ((?game-time wm-fact))
      (wm-key-prefix ?game-time:key (create$ refbox game-time))
      (bind ?current-game-time (nth$ 1 ?game-time:values))
      (printout t "Current game time is " ?current-game-time crlf)
    )
    (printout t crlf)

    (bind ?arguments (create$ ?current-game-time))

    ; machine status
    (do-for-all-facts ((?wp-at-fact wm-fact)) 
        (wm-key-prefix ?wp-at-fact:key (create$ domain fact wp-at args? wp))
        (bind ?wp-for-order (wm-key-arg ?wp-at-fact:key wp))
        (bind ?machine-loc (wm-key-arg ?wp-at-fact:key m))
        ; (if (eq (sym-cat (sub-string 1 3 (str-cat ?wp-for-order))) wp-)
        ; then
        (printout t "wp " ?wp-for-order " is at machine " ?machine-loc " side " (wm-key-arg ?wp-at-fact:key side) crlf)
        ; )
    )
    (do-for-all-facts ((?robot-hold-wp wm-fact)) 
        (wm-key-prefix ?robot-hold-wp:key (create$ domain fact holding args? r))
        (bind ?holding-robot (wm-key-arg ?robot-hold-wp:key r))
        (bind ?holding-wp (wm-key-arg ?robot-hold-wp:key wp))
        (printout t ?holding-robot " is holding " ?holding-wp crlf)
    )
    ; (wm-fact (key domain fact holding args? r ?robot wp ?wp))
    (printout t crlf)
    ; (do-for-all-facts ((?mps-state wm-fact)) 
    ;     (wm-key-prefix ?mps-state:key (create$ domain fact mps-state args? m))
    ;     (bind ?machine (wm-key-arg ?mps-state:key m))
    ;     (bind ?machine-state (wm-key-arg ?mps-state:key s))
    ;     (bind ?machine-true ?mps-state:value)
    ;     (printout t "machine " ?machine " is at state " ?machine-state " with value " ?machine-true crlf)
    ; )
    (bind ?overall-workload (calculate-overall-workload-for-each-processing-order C-))
    (bind ?update-cs1 (nth$ 1 ?overall-workload))
    (bind ?update-cs2 (nth$ 2 ?overall-workload))
    (bind ?update-rs1 (nth$ 3 ?overall-workload))
    (bind ?update-rs2 (nth$ 4 ?overall-workload))

    (do-for-all-facts ((?started wm-fact))
      (and (wm-key-prefix ?started:key (create$ order meta started))
              (eq ?started:value TRUE))
      (bind ?or_started (wm-key-arg ?started:key ord))
      (printout t "order " ?or_started " is started" crlf)
    )
    (do-for-all-facts ((?fulfilled wm-fact))
      (wm-key-prefix ?fulfilled:key (create$ domain fact order-fulfilled))
      (bind ?or_fulfilled (wm-key-arg ?fulfilled:key ord))
      (printout t "order " ?or_fulfilled " is finished" crlf)
    )

    ; order status
    (foreach ?or ?order-list
      (printout t crlf)
      (bind ?tag 0)

      ; order ID 
      (printout t ?or-index ": " ?or " ")

      ; order complexity
      (do-for-fact ((?oc wm-fact)) 
          (wm-key-prefix ?oc:key (create$ domain fact order-complexity args? ord ?or com))
          (bind ?com (wm-key-arg ?oc:key com))
          (printout t " " ?com))

      ; If it is not started yet? being processed? delivered?
      (if (member$ ?or ?candidate-list) 
        then (printout t " candidate ")
        else
          (if (any-factp ((?delivered wm-fact)) 
                (and (eq ?delivered:key (create$ domain fact quantity-delivered args? ord ?or team CYAN))
                    (= ?delivered:value 1)))
          then
            (bind ?tag 2) (printout t " ->delivered ")
          else
            (bind ?tag 1) (printout t " being processed ")
          )
          ; (do-for-fact ((?delivered wm-fact)) (eq ?delivered:key (create$ domain fact quantity-delivered args? ord ?or team CYAN))
          ;   (printout t " ->delivered " ?delivered:value))
      ) 
      (printout t crlf)

      (printout t "the second method to determine the status of order" crlf)
      (if (any-factp ((?started wm-fact)) 
          (and (wm-key-prefix ?started:key (create$ order meta started args? ord ?or))
              (eq ?started:value TRUE)))
      then
        (if (any-factp ((?fulfilled wm-fact)) 
            (and (wm-key-prefix ?fulfilled:key (create$ domain fact order-fulfilled))
                (eq (wm-key-arg ?fulfilled:key ord) ?or)))
        then
          (bind ?order-status " ->fulfilled ")
        else
          (bind ?order-status " ->not finished yet ")
        )
      else
        (bind ?order-status " ->not started yet ")
      )
      (printout t ?order-status)
      (printout t crlf)

      ; delivery time window
      (do-for-all-facts ((?tw wm-fact)) 
          (and (wm-key-prefix ?tw:key (create$ refbox order)) (member$ ?or ?tw:key))
          (if (member$ delivery-begin ?tw:key) then (bind ?or-begin ?tw:value) (printout t "Delivery Begin " ?or-begin crlf))
          (if (member$ delivery-end ?tw:key) then (bind ?or-end ?tw:value) (printout t "Delivery End " ?or-end crlf)))    
      ; (wm-fact (key refbox order ?order delivery-end) (type UINT) (value ?end))

      ; base color
      (do-for-fact ((?bc wm-fact)) 
          (wm-key-prefix ?bc:key (create$ domain fact order-base-color args? ord ?or col))
          (printout t "Base Color " (wm-key-arg ?bc:key col) crlf))
      ; (if (any-factp ((?bc wm-fact)) (wm-key-prefix ?bc:key (create$ domain fact order-base-color args? ord ?or col)))
      ; then (printout t "Base Color" (wm-key-arg ?bc:key col) crlf))  

      ; cap color and station
      (do-for-fact ((?cc wm-fact)) 
          (wm-key-prefix ?cc:key (create$ domain fact order-cap-color args? ord ?or col))
          (bind ?cap-color (wm-key-arg ?cc:key col))
          (printout t "Cap Color " ?cap-color))
      (do-for-fact ((?cs wm-fact))
          (and (wm-key-prefix ?cs:key (create$ domain fact cs-color args? m))
              (eq ?cap-color (wm-key-arg ?cs:key col)))
          (bind ?cap-station (wm-key-arg ?cs:key m))
          (if (eq ?cap-station C-CS1) 
          then (bind ?CS1-num 1) (bind ?CS2-num 0)
          else (bind ?CS1-num 0) (bind ?CS2-num 1))
          (printout t " " ?cap-station))
      (printout t crlf)
      ; cap station
      (printout t "the second method to decide which cap station is used" crlf)
      (do-for-fact ((?cs wm-fact))
        (wm-key-prefix ?cs:key (create$ mps workload order args? m C-CS1 ord ?or))
        (printout t "the number of CS1 is " ?cs:value crlf))     
      (do-for-fact ((?cs wm-fact))
        (wm-key-prefix ?cs:key (create$ mps workload order args? m C-CS2 ord ?or))
        (printout t "the number of CS2 is " ?cs:value crlf))   
      (printout t crlf) 
      ; ring color
      (do-for-fact ((?r1c wm-fact)) 
          (wm-key-prefix ?r1c:key (create$ domain fact order-ring1-color args? ord ?or col))
          (bind ?ring1-color (wm-key-arg ?r1c:key col))
          (printout t "Ring1 Color " ?ring1-color))
      ; ring station
      (if (neq ?ring1-color RING_NONE)
      then 
        (bind ?rs1 (goal-production-get-machine-for-color ?ring1-color))
        (printout t " " ?rs1)
      else (bind ?rs1 None))
      ; ring cost
      (do-for-fact ((?r1r wm-fact)) 
          (wm-key-prefix ?r1r:key (create$ domain fact rs-ring-spec args? m ?rs1 r ?ring1-color rn))
          (bind ?ring1-requirement (wm-key-arg ?r1r:key rn))
          (printout t " " ?ring1-requirement))
      (printout t crlf)

      ; (wm-fact (key domain fact rs-ring-spec
      ;           args? m ?mps r ?ring1-color&~RING_NONE rn ?ring-num))

      (do-for-fact ((?r2c wm-fact)) 
          (wm-key-prefix ?r2c:key (create$ domain fact order-ring2-color args? ord ?or col))
          (bind ?ring2-color (wm-key-arg ?r2c:key col))
          (printout t "Ring2 Color " ?ring2-color))
      (if (neq ?ring2-color RING_NONE)
      then 
        (bind ?rs2 (goal-production-get-machine-for-color ?ring2-color))
        (printout t " " ?rs2)
      else (bind ?rs2 None))
      (do-for-fact ((?r2r wm-fact)) 
          (wm-key-prefix ?r2r:key (create$ domain fact rs-ring-spec args? m ?rs2 r ?ring2-color rn))
          (bind ?ring2-requirement (wm-key-arg ?r2r:key rn))
          (printout t " " ?ring2-requirement))
      (printout t crlf) 

      (do-for-fact ((?r3c wm-fact)) 
          (wm-key-prefix ?r3c:key (create$ domain fact order-ring3-color args? ord ?or col))
          (bind ?ring3-color (wm-key-arg ?r3c:key col))
          (printout t "Ring3 Color " ?ring3-color))
      (if (neq ?ring3-color RING_NONE)
      then 
        (bind ?rs3 (goal-production-get-machine-for-color ?ring3-color))
        (printout t " " ?rs3)
      else (bind ?rs3 None))
      (do-for-fact ((?r3r wm-fact)) 
          (wm-key-prefix ?r3r:key (create$ domain fact rs-ring-spec args? m ?rs3 r ?ring3-color rn))
          (bind ?ring3-requirement (wm-key-arg ?r3r:key rn))
          (printout t " " ?ring3-requirement))
      (printout t crlf)  
            
      ; ring station
      (printout t "the second method to decide which ring station is used" crlf)
      (do-for-fact ((?rs wm-fact))
        (wm-key-prefix ?rs:key (create$ mps workload order args? m C-RS1 ord ?or))
        (bind ?RS1-num ?rs:value)
        (printout t "the number of RS1 is " ?RS1-num crlf))     
      (do-for-fact ((?rs wm-fact))
        (wm-key-prefix ?rs:key (create$ mps workload order args? m C-RS2 ord ?or))
        (bind ?RS2-num ?rs:value)
        (printout t "the number of RS2 is " ?RS2-num crlf))   
      (printout t crlf)

      ; (do-for-all-facts ((?mps-workload wm-fact)) 
      ;   (and (wm-key-prefix ?mps-workload:key (create$ mps workload order args? m))
      ;       (eq (wm-key-arg ?mps-workload:key ord) ?or))
      ;   (bind ?machine (wm-key-arg ?mps-workload:key m))
      ;   (bind ?machine-workload ?mps-workload:value)
      ;   (printout t "machine " ?machine " has workload " ?machine-workload " for this order " crlf)
      ; )       

      ; the production step of this order
      (if (= ?tag 0) 
        then (printout t "No machines are occupied becauseof this order!" crlf)
        else 
          (bind ?wp-for-order (sym-cat wp- ?or))
          ; (any-factp ((?after-base wm-fact)) (and (wm-key-prefix ?after-base:key (create$ wp meta points-current args? wp))
          ;                                             (eq (wm-key-arg ?after-base:key wp) ?wp-for-order)))
          
          ; (printout t "Base of this order is already finished" crlf)
          (do-for-all-facts ((?next-step-fact wm-fact)) 
              (eq ?next-step-fact:key (create$ wp meta next-step args? wp ?wp-for-order))
              (bind ?next-step ?next-step-fact:value)
              (printout t "Next step of this order is " ?next-step crlf))
          ; (wm-fact (key order meta next-step args? ord ?order) (value ?next-step))
          (do-for-all-facts ((?prev-step-fact wm-fact)) 
              (eq ?prev-step-fact:key (create$ wp meta prev-step args? wp ?wp-for-order))
              (bind ?prev-step ?prev-step-fact:value)
              (printout t "Previous step of this order is " ?prev-step crlf))
          (do-for-all-facts ((?next-machine-fact wm-fact)) 
              (eq ?next-machine-fact:key (create$ wp meta next-machine args? wp ?wp-for-order))
              (bind ?next-machine ?next-machine-fact:value)
              (printout t "Next machine of this order is " ?next-machine crlf))
          ; (do-for-fact ((?delivered-fact plan-action)) 
          ;     (eq (sym-cat (sub-string 1 13 (str-cat ?delivered-fact:action-name))) fulfill-order)
          ;     ; (bind ?next-machine ?next-machine-fact:value)
          ;     (printout t "id " ?delivered-fact:id crlf)
          ;     (printout t "goal-id " ?delivered-fact:goal-id crlf)
          ;     (printout t "plan-id " ?delivered-fact:plan-id crlf))            
          ; (plan-action (id (string-to-field ?id-str)) (action-name ?name) (param-values $?param-values))
    
          ; (wm-fact (key wp meta estimated-points-total args? wp ?wp)
          ;          (type INT) (is-list FALSE) (value ?ep-total))
      )
      (if (= ?tag 1)
      then
        (if (neq ?rs1 None)
        then
          (bind ?r1 (create$ (trim-machine-name ?rs1) (+ 1 (transfer-ring-requirements-into-number ?ring1-requirement))))
        else 
          (bind ?r1 (create$ None 0))
        )
        (if (neq ?rs2 None)
        then
          (bind ?r2 (create$ (trim-machine-name ?rs2) (+ 1 (transfer-ring-requirements-into-number ?ring2-requirement))))
        else 
          (bind ?r2 (create$ None 0))
        )
        (if (neq ?rs3 None)
        then
          (bind ?r3 (create$ (trim-machine-name ?rs3) (+ 1 (transfer-ring-requirements-into-number ?ring3-requirement))))
        else 
          (bind ?r3 (create$ None 0))
        )
        (bind ?del-res (calculate-decreased-workload-for-each-processing-order ?next-step ?com (trim-machine-name ?cap-station) ?r1 ?r2 ?r3))
        (printout t "Subtracted workload is " ?del-res  crlf)
        (bind ?update-cs1 (- ?update-cs1 (nth$ 1 ?del-res)))
        (bind ?update-cs2 (- ?update-cs2 (nth$ 2 ?del-res)))
        (bind ?update-rs1 (- ?update-rs1 (nth$ 3 ?del-res)))
        (bind ?update-rs2 (- ?update-rs2 (nth$ 4 ?del-res)))
      )
      (printout t crlf)
      (bind ?one-order-info (create$ ?or-index ?or ?com ?order-status ?or-begin ?or-end ?CS1-num ?CS2-num ?RS1-num ?RS2-num))
      (bind ?arguments (create$ ?arguments ?one-order-info <>))
    )
    (bind ?res-workload (create$ ?update-cs1 ?update-cs2 ?update-rs1 ?update-rs2))
    (printout t "Overall workload is " ?overall-workload crlf)
    (printout t "Updated workload is " ?res-workload  crlf)
    (bind ?arguments (create$ ?arguments ?overall-workload ?res-workload))

    (return ?arguments)
)

(defrule goal-production-navgraph-compute-wait-positions-finished
  "Add the waiting points to the domain once their generation is finished."
  (NavGraphWithMPSGeneratorInterface (id "/navgraph-generator-mps") (final TRUE))
  (or (wm-fact (key config rcll use-static-navgraph) (type BOOL) (value TRUE))
      (forall
        (wm-fact (key central agent robot args? r ?robot))
        (NavGraphWithMPSGeneratorInterface (id ?id&:(eq ?id (remote-if-id ?robot "navgraph-generator-mps"))) (final TRUE))
      )
  )
=>
  (printout t "Navgraph generation of waiting-points finished. Getting waitpoints." crlf)
  (do-for-all-facts ((?waitzone navgraph-node)) (str-index "WAIT-" ?waitzone:name)
    (assert
      (domain-object (name (sym-cat ?waitzone:name)) (type waitpoint))
      (wm-fact (key navgraph waitzone args? name (sym-cat ?waitzone:name)) (is-list TRUE) (type INT) (values (nth$ 1 ?waitzone:pos) (nth$ 2 ?waitzone:pos)))
    )
  )
  (assert (wm-fact (key navgraph waitzone generated) (type BOOL) (value TRUE)))
  (delayed-do-for-all-facts ((?wm wm-fact)) (wm-key-prefix ?wm:key (create$ central agent robot))
    (assert (wm-fact (key central agent robot-waiting args? r (wm-key-arg ?wm:key r))))
  )
)
(deffunction goal-meta-get-goal-category (?goal-class)
  (bind ?production-goals (create$ MOUNT-CAP MOUNT-RING DELIVER-RC21 DELIVER))
  (bind ?maintenance-goals (create$ BUFFER-CAP PAY-FOR-RINGS-WITH-BASE PAY-FOR-RINGS-WITH-CAP-CARRIER PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF))
  (bind ?maintenance-instruct-goals (create$ INSTRUCT-RS-MOUNT-RING INSTRUCT-CS-MOUNT-CAP INSTRUCT-DS-DELIVER))
  (bind ?production-instruct-goals (create$ INSTRUCT-CS-BUFFER-CAP INSTRUCT-DS-DISCARD))
  (bind ?other-goals (create$ MOVE MOVE-OUT-OF-WAY ENTER-FIELD DISCARD WAIT-NOTHING-EXECUTABLE EXPLORATION-MOVE REFILL-SHELF))
  (bind ?other-instruct-goals (create$ INSTRUCT-BS-DISPENSE-BASE))

  (if (member$ ?goal-class ?production-goals) then (return PRODUCTION))
  (if (member$ ?goal-class ?maintenance-goals) then (return MAINTENANCE))
  (if (member$ ?goal-class ?production-instruct-goals) then (return PRODUCTION-INSTRUCT))
  (if (member$ ?goal-class ?maintenance-instruct-goals) then (return MAINTENANCE-INSTRUCT))
  (if (member$ ?goal-class ?other-instruct-goals) then (return OTHER-INSTRUCT))
  (if (member$ ?goal-class ?other-goals) then (return OTHER))

  (return UNKNOWN)
)

(defrule goal-meta-assign-category
  "Assign the category of a simple goal based on its class"
  (goal (id ?goal-id) (sub-type SIMPLE) (class ?class))
  ?gm <- (goal-meta (goal-id ?goal-id) (category nil))
  =>
  (modify ?gm (category (goal-meta-get-goal-category ?class)))
)

(defrule goal-meta-print-error-unknown-category
  (goal (id ?goal-id) (class ?class))
  (goal-meta (goal-id ?goal-id) (category UNKNOWN))
  =>
  (printout error "Simple goal " ?goal-id " of class " ?class " has UNKNOWN category!" crlf)
)

; ----------------------- Maintenance Goals -------------------------------

(defrule goal-production-create-refill-shelf-maintain
" The parent goal to refill a shelf. Allows formulation of goals to refill
  a shelf only if the game is in the production phase and the domain is loaded.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (domain-facts-loaded)
  (not (goal (class REFILL-SHELF-MAINTAIN)))
  (wm-fact (key refbox phase) (value PRODUCTION))
  =>
  (bind ?goal (goal-tree-assert-run-endless REFILL-SHELF-MAINTAIN 1))
  (modify ?goal (required-resources)
                (params frequency 1 retract-on-REJECTED)
                (verbosity QUIET))
)

(defrule goal-production-create-refill-shelf-achieve
  "Refill a shelf whenever it is empty."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?g <- (goal (id ?maintain-id) (class REFILL-SHELF-MAINTAIN) (mode SELECTED))
  (not (goal (class REFILL-SHELF) (mode ~RETRACTED)))
  (wm-fact (key refbox phase) (value PRODUCTION))
  (wm-fact (key game state) (value RUNNING))
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (not (wm-fact (key domain fact wp-on-shelf args? wp ?wp m ?mps spot ?spot)))
  =>
  (bind ?goal (assert (goal (id (sym-cat REFILL-SHELF- (gensym*)))
                (class REFILL-SHELF) (sub-type SIMPLE)
                (parent ?maintain-id) (verbosity QUIET)
                (params mps ?mps) (is-executable TRUE))))
  (goal-meta-assert ?goal central nil nil)
)

; ----------------------- Robot Assignment -------------------------------

(defrule goal-production-assign-robot-to-enter-field
  (wm-fact (key central agent robot args? r ?robot))
  (not (wm-fact (key domain fact entered-field args? r ?robot)))
  (goal (id ?oid) (class ENTER-FIELD)  (sub-type SIMPLE) (mode FORMULATED) (is-executable FALSE))
  ?gm <- (goal-meta (goal-id ?oid) (assigned-to nil))
  (not (goal-meta (assigned-to ?robot)))
  =>
  (modify ?gm (assigned-to ?robot))
)

(defrule goal-production-assign-robot-to-simple-goals
" Before checking SIMPLE goals for their executability, pick a waiting robot
  that should get a new goal assigned to it next. "
  (declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
;  "a simple unassigned goal"
  (goal (id ?oid) (sub-type SIMPLE) (mode FORMULATED) (is-executable FALSE))
  (goal-meta (goal-id ?oid) (assigned-to nil))
  (wm-fact (key central agent robot args? r ?robot))
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (goal-meta (assigned-to ?robot)))
  (wm-fact (key central agent robot-waiting args? r ?robot))
  =>
  (bind ?longest-waiting 0)
  (bind ?longest-waiting-robot ?robot)
  (delayed-do-for-all-facts ((?waiting wm-fact))
    (wm-key-prefix ?waiting:key (create$ central agent robot-waiting))
    (if (or (eq ?longest-waiting 0) (< (fact-index ?waiting) ?longest-waiting))
     then
      (bind ?longest-waiting-robot (wm-key-arg ?waiting:key r))
      (bind ?longest-waiting (fact-index ?waiting))
    )
  )
  (delayed-do-for-all-facts ((?g goal))
    (and (eq ?g:is-executable FALSE)
         (eq ?g:sub-type SIMPLE) (eq ?g:mode FORMULATED)
         (or (not (any-factp ((?gm  goal-meta))
                            (eq ?gm:goal-id ?g:id)))
             (any-factp ((?gm goal-meta))
                (and (eq ?gm:goal-id ?g:id)
                     (eq ?gm:assigned-to nil)))))
    (goal-meta-assign-robot-to-goal ?g ?robot)
  )
  (modify ?longest-waiting)
)

(defrule goal-production-unassign-robot-from-finished-goals
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?g <- (goal (id ?id) (sub-type SIMPLE) (mode RETRACTED)
        (parent ?parent))
  (not (goal (id ?parent) (type MAINTAIN)))
  (goal-meta (goal-id ?id) (assigned-to ?robot&~nil))
  =>
  (remove-robot-assignment-from-goal-meta ?g)
)

; ----------------------- Assert Goal Functions -------------------------------

(deffunction goal-production-assert-buffer-cap
  (?mps ?cap-color ?order-id)

  (bind ?goal (assert (goal (class BUFFER-CAP)
        (id (sym-cat BUFFER-CAP- (gensym*))) (sub-type SIMPLE)
        (verbosity NOISY) (is-executable FALSE)
        (params target-mps ?mps
                cap-color ?cap-color)
  )))
  (goal-meta-assert ?goal nil ?order-id nil)
  (return ?goal)
)

(deffunction goal-production-assert-pick-and-place
  (?mps ?robot)
  (bind ?goal (assert (goal (class PICK-AND-PLACE)
        (id (sym-cat PICK-AND-PLACE- (gensym*))) (sub-type SIMPLE)
        (verbosity NOISY) (is-executable FALSE)
        (params target-mps ?mps)
  )))
  (goal-meta-assert-restricted ?goal ?robot)
  (return ?goal)
)

(deffunction goal-production-assert-move-robot-to-output
  (?mps ?robot)
  (bind ?goal (assert (goal (class MOVE)
        (id (sym-cat MOVE- (gensym*))) (sub-type SIMPLE)
        (verbosity NOISY) (is-executable FALSE)
        (params target-mps ?mps)
  )))
  (goal-meta-assert-restricted ?goal ?robot)
  (return ?goal)
)

(deffunction goal-production-assert-mount-cap
  (?wp ?mps ?wp-loc ?wp-side ?order-id)

  (bind ?goal (assert (goal (class MOUNT-CAP)
        (id (sym-cat MOUNT-CAP- (gensym*))) (sub-type SIMPLE)
         (verbosity NOISY) (is-executable FALSE)
        (params wp ?wp
                target-mps ?mps
                target-side INPUT
                wp-loc ?wp-loc
                wp-side ?wp-side)
  )))
  (goal-meta-assert ?goal nil ?order-id nil)
  (return ?goal)
)

(deffunction goal-production-assert-mount-ring
  (?wp ?rs ?wp-loc ?wp-side ?ring-color ?order-id ?ring-nr)
  (bind ?goal (assert (goal (class MOUNT-RING)
        (id (sym-cat MOUNT-RING- (gensym*))) (sub-type SIMPLE)
        (verbosity NOISY) (is-executable FALSE)
        (params  wp ?wp
                 target-mps ?rs
                 target-side INPUT
                 wp-loc ?wp-loc
                 wp-side ?wp-side
                 ring-color ?ring-color
                 )
  )))
  (goal-meta-assert ?goal nil ?order-id ?ring-nr)
  (return ?goal)
)

(deffunction goal-production-assert-discard
  (?wp ?cs ?side ?order-id)

  (bind ?goal (assert (goal (class DISCARD)
        (id (sym-cat DISCARD- (gensym*))) (sub-type SIMPLE)
        (verbosity NOISY) (is-executable FALSE)
        (params wp ?wp wp-loc ?cs wp-side ?side)
  )))
  (goal-meta-assert ?goal nil ?order-id nil)
  (return ?goal)
)

(deffunction goal-production-assert-deliver-rc21
  (?wp ?order-id)

  (bind ?goal (assert (goal (class DELIVER-RC21)
        (id (sym-cat DELIVER-RC21- (gensym*))) (sub-type SIMPLE)
        (verbosity NOISY) (is-executable FALSE)
        (params wp ?wp)
  )))
  (goal-meta-assert ?goal nil ?order-id nil)
  (return ?goal)
)

(deffunction goal-production-assert-instruct-ds-deliver
  (?wp ?order-id ?ds)

  (bind ?goal (assert (goal (class INSTRUCT-DS-DELIVER)
    (id (sym-cat INSTRUCT-DS-DELIVER- (gensym*))) (sub-type SIMPLE)
    (verbosity NOISY) (is-executable FALSE)
    (params wp ?wp
            target-mps ?ds)
  )))
  (goal-meta-assert ?goal central ?order-id nil)
  (return ?goal)
)

(deffunction goal-production-assert-instruct-ds-discard
  (?wp ?ds)

  (bind ?goal (assert (goal (class INSTRUCT-DS-DISCARD)
    (id (sym-cat INSTRUCT-DS-DISCARD- (gensym*))) (sub-type SIMPLE)
    (verbosity NOISY) (is-executable FALSE)
    (params wp ?wp
            target-mps ?ds)
  )))
  (goal-meta-assert ?goal central nil nil)
  (return ?goal)
)

(deffunction goal-production-assert-deliver
  "If there is a DS, do a normal delivery, otherwise do a RoboCup 2021 delivery. "
  (?wp ?order-id ?instruct-parent ?ds)

  (bind ?goal nil)
  (if (any-factp ((?state domain-fact)) (and (eq ?state:name mps-state)
                                             (member$ ?ds ?state:param-values))
      )
  then

    (bind ?instruct-goal (goal-production-assert-instruct-ds-deliver ?wp ?order-id ?ds))
    (modify ?instruct-goal (parent ?instruct-parent))

    (bind ?goal
      (goal-meta-assert (assert (goal (class DELIVER)
        (id (sym-cat DELIVER- (gensym*))) (sub-type SIMPLE)
        (verbosity NOISY) (is-executable FALSE)
        (params wp ?wp
            target-mps ?ds
            target-side INPUT)
      )) nil ?order-id nil)
    )
  else
    (bind ?goal (goal-production-assert-deliver-rc21 ?wp ?order-id))
  )

  (return ?goal)
)

(deffunction goal-production-assert-pay-for-rings-with-base
  (?wp ?wp-loc ?wp-side ?target-mps ?target-side ?order-id)
  (bind ?goal (assert (goal (class PAY-FOR-RINGS-WITH-BASE)
        (id (sym-cat PAY-FOR-RINGS-WITH-BASE- (gensym*))) (sub-type SIMPLE)
        (verbosity NOISY) (is-executable FALSE)
        (params  wp ?wp
                 wp-loc ?wp-loc
                 wp-side ?wp-side
                 target-mps ?target-mps
                 target-side ?target-side
                 )
  )))
  (goal-meta-assert ?goal nil ?order-id nil)
  (return ?goal)
)

(deffunction goal-production-assert-pay-for-rings-with-cap-carrier
  (?wp ?wp-loc ?wp-side ?target-mps ?target-side ?order-id)

  (bind ?goal (assert (goal (class PAY-FOR-RINGS-WITH-CAP-CARRIER)
        (id (sym-cat PAY-FOR-RINGS-WITH-CAP-CARRIER- (gensym*))) (sub-type SIMPLE)
        (verbosity NOISY) (is-executable FALSE)
        (params  wp ?wp
                 wp-loc ?wp-loc
                 wp-side ?wp-side
                 target-mps ?target-mps
                 target-side ?target-side
                 )
  )))
  (goal-meta-assert ?goal nil ?order-id nil)
  (return ?goal)
)

(deffunction goal-production-assert-pay-for-rings-with-cap-carrier-from-shelf
  (?wp-loc ?target-mps ?target-side ?order-id)

  (bind ?goal (assert (goal (class PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF)
        (id (sym-cat PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF- (gensym*))) (sub-type SIMPLE)
        (verbosity NOISY) (is-executable FALSE)
        (params  wp-loc ?wp-loc
                 target-mps ?target-mps
                 target-side ?target-side
                 )
  )))
  (goal-meta-assert ?goal nil ?order-id nil)
  (return ?goal)
)

(deffunction goal-production-assert-instruct-cs-buffer-cap
  (?mps ?cap-color ?order-id)

  (bind ?goal (assert (goal (class INSTRUCT-CS-BUFFER-CAP)
        (id (sym-cat INSTRUCT-CS-BUFFER-CAP- (gensym*))) (sub-type SIMPLE)
        (verbosity NOISY) (is-executable FALSE)
        (params target-mps ?mps
                cap-color ?cap-color)
  )))
  (goal-meta-assert ?goal central ?order-id nil)
  (return ?goal)
)

(deffunction goal-production-assert-instruct-bs-dispense-base
  (?wp ?base-color ?side ?order-id ?bs)

  (bind ?goal (assert (goal (class INSTRUCT-BS-DISPENSE-BASE)
    (id (sym-cat INSTRUCT-BS-DISPENSE-BASE- (gensym*))) (sub-type SIMPLE)
    (verbosity NOISY) (is-executable FALSE)
        (params wp ?wp
                target-mps ?bs
                target-side ?side
                base-color ?base-color)
  )))
  (goal-meta-assert ?goal central ?order-id nil)
  (return ?goal)
)

(deffunction goal-production-assert-instruct-cs-mount-cap
  (?mps ?cap-color ?order-id)
  (bind ?goal (assert (goal (class INSTRUCT-CS-MOUNT-CAP)
        (id (sym-cat INSTRUCT-CS-MOUNT-CAP- (gensym*))) (sub-type SIMPLE)
        (verbosity NOISY) (is-executable FALSE)
        (params target-mps ?mps
                cap-color ?cap-color)
  )))
  (goal-meta-assert ?goal central ?order-id nil)
  (return ?goal)
)

(deffunction goal-production-assert-instruct-rs-mount-ring
  (?mps ?col-ring ?order-id ?ring-nr)
  (bind ?goal (assert (goal (class INSTRUCT-RS-MOUNT-RING)
        (id (sym-cat INSTRUCT-RS-MOUNT-RING- (gensym*))) (sub-type SIMPLE)
        (verbosity NOISY) (is-executable FALSE)
              (params target-mps ?mps
                      ring-color ?col-ring
               )
  )))
  (goal-meta-assert ?goal central ?order-id ?ring-nr)
  (return ?goal)
)

(deffunction goal-production-assert-enter-field
  (?robot ?zone)

  (bind ?goal (assert (goal (class ENTER-FIELD)
              (id (sym-cat ENTER-FIELD- (gensym*)))
              (sub-type SIMPLE)
              (verbosity NOISY) (is-executable FALSE)
              (params zone ?zone)
              (meta-template goal-meta)
  )))
  (goal-meta-assert-restricted ?goal ?robot)
  (return ?goal)
)


(deffunction goal-production-assert-move-out-of-way
	(?location)
	(bind ?goal (assert (goal (class MOVE-OUT-OF-WAY)
	            (id (sym-cat MOVE-OUT-OF-WAY- (gensym*)))
	            (sub-type SIMPLE)
	            (verbosity NOISY) (is-executable FALSE)
	            (meta-template goal-meta)
	            (params target-pos ?location)
	)))
	(return ?goal)
)

(deffunction goal-production-assign-order-and-prio-to-goal (?goal ?order-id ?prio)
  (bind ?goal-id (fact-slot-value ?goal id))
  (modify ?goal (priority ?prio))
  (do-for-fact ((?goal-meta goal-meta)) (eq ?goal-meta:goal-id ?goal-id)
    (modify ?goal-meta (root-for-order ?order-id))
  )
)

(deffunction goal-production-assert-requests
  ;assert requests for an order's support goals (i.e., payment, buffer, discard)
  (?rs ?cs ?col-rings ?col-cap ?order ?prio)

  (bind ?index 1)
  (bind ?seq 1)
  (loop-for-count (length$ ?rs)
    (bind ?price 0)
    (do-for-fact ((?rs-ring-spec wm-fact))
      (and (wm-key-prefix ?rs-ring-spec:key (create$ domain fact rs-ring-spec))
          (eq (wm-key-arg ?rs-ring-spec:key r ) (nth$ ?index ?col-rings))
      )
      (bind ?price (sym-to-int (wm-key-arg ?rs-ring-spec:key rn)))
    )
    (loop-for-count ?price
      (assert (wm-fact (key request pay args? ord ?order m (nth$ ?index ?rs) ring (sym-cat RING ?index) seq ?seq prio ?prio) (is-list TRUE) (type SYMBOL) (values status OPEN assigned-to)))
      (bind ?seq (+ ?seq 1))
    )
    (bind ?index (+ ?index 1))
  )
  (assert (wm-fact (key request buffer args? ord ?order col ?col-cap prio ?prio) (is-list TRUE) (type SYMBOL) (values status OPEN assigned-to)))
  (assert (wm-fact (key request discard args? ord ?order cs ?cs prio ?prio) (is-list TRUE) (type SYMBOL) (values status OPEN assigned-to)))
)


(deffunction goal-production-assert-c0
  (?root-id ?order-id ?wp-for-order ?cs ?ds ?bs ?col-cap ?col-base)

  ;assert the instruct goals
  (bind ?instruct-goals
    (goal-tree-assert-central-run-parallel-prio INSTRUCT-ORDER ?*PRODUCTION-C0-PRIORITY*
      (goal-production-assert-instruct-bs-dispense-base ?wp-for-order ?col-base INPUT ?order-id ?bs)
      (goal-production-assert-instruct-cs-mount-cap ?cs ?col-cap ?order-id)
    )
  )
  (bind ?instruct-parent (fact-slot-value ?instruct-goals id))
  (bind ?instruct-goals (modify ?instruct-goals (parent ?root-id) (priority ?*PRODUCTION-C0-PRIORITY*)))

  ;assert the main production tree
  (bind ?goal
    (goal-tree-assert-central-run-all-prio PRODUCE-ORDER ?*PRODUCTION-C0-PRIORITY*
      (goal-production-assert-deliver ?wp-for-order ?order-id ?instruct-parent ?ds)
      (goal-production-assert-mount-cap ?wp-for-order ?cs ?bs INPUT ?order-id)
    )
  )

  (goal-production-assign-order-and-prio-to-goal ?goal ?order-id ?*PRODUCTION-C0-PRIORITY*)

  (goal-production-assert-requests (create$ ) ?cs (create$ ) ?col-cap ?order-id ?*PRODUCTION-C0-PRIORITY*)
)

(deffunction goal-production-assert-c1
  (?root-id ?order-id ?wp-for-order ?cs ?ds ?bs ?rs1 ?col-cap ?col-base ?col-ring1)

  ;assert the instruct goals
  (bind ?instruct-goals
    (goal-tree-assert-central-run-parallel-prio INSTRUCT-ORDER ?*PRODUCTION-C1-PRIORITY*
      (goal-production-assert-instruct-bs-dispense-base ?wp-for-order ?col-base INPUT ?order-id ?bs)
      (goal-production-assert-instruct-cs-mount-cap ?cs ?col-cap ?order-id)
      (goal-production-assert-instruct-rs-mount-ring ?rs1 ?col-ring1 ?order-id ONE)
    )
  )
  (bind ?instruct-parent (fact-slot-value ?instruct-goals id))
  (bind ?instruct-goals (modify ?instruct-goals (parent ?root-id) (priority ?*PRODUCTION-C1-PRIORITY*)))

  ;assert the main production tree
  (bind ?goal
    (goal-tree-assert-central-run-all-prio PRODUCE-ORDER ?*PRODUCTION-C1-PRIORITY*
      (goal-production-assert-deliver ?wp-for-order ?order-id ?instruct-parent ?ds)
      (goal-production-assert-mount-cap ?wp-for-order ?cs ?rs1 OUTPUT ?order-id)
      (goal-production-assert-mount-ring ?wp-for-order ?rs1 ?bs INPUT ?col-ring1 ?order-id ONE)
    )
  )

  (goal-production-assign-order-and-prio-to-goal ?goal ?order-id ?*PRODUCTION-C1-PRIORITY*)

  (goal-production-assert-requests (create$ ?rs1) ?cs (create$ ?col-ring1) ?col-cap ?order-id ?*PRODUCTION-C1-PRIORITY*)
)

(deffunction goal-production-assert-c2
  (?root-id ?order-id ?wp-for-order ?cs ?ds ?bs ?rs1 ?rs2 ?col-cap ?col-base ?col-ring1 ?col-ring2)

  (bind ?instruct-goals
    (goal-tree-assert-central-run-parallel-prio INSTRUCT-ORDER ?*PRODUCTION-C2-PRIORITY*
      (goal-production-assert-instruct-bs-dispense-base ?wp-for-order ?col-base INPUT ?order-id ?bs)
      (goal-production-assert-instruct-cs-mount-cap ?cs ?col-cap ?order-id)
      (goal-production-assert-instruct-rs-mount-ring ?rs1 ?col-ring1 ?order-id ONE)
      (goal-production-assert-instruct-rs-mount-ring ?rs2 ?col-ring2 ?order-id TWO)
    )
  )
  (bind ?instruct-parent (fact-slot-value ?instruct-goals id))
  (bind ?instruct-goals (modify ?instruct-goals (parent ?root-id) (priority ?*PRODUCTION-C2-PRIORITY*)))

  (bind ?goal
    (goal-tree-assert-central-run-all-prio PRODUCE-ORDER ?*PRODUCTION-C2-PRIORITY*
      (goal-production-assert-deliver ?wp-for-order ?order-id ?instruct-parent ?ds)
      (goal-production-assert-mount-cap ?wp-for-order ?cs ?rs2 OUTPUT ?order-id)
      (goal-production-assert-mount-ring ?wp-for-order ?rs2 ?rs1 OUTPUT ?col-ring2 ?order-id TWO)
      (goal-production-assert-mount-ring ?wp-for-order ?rs1 ?bs INPUT ?col-ring1 ?order-id ONE)
    )
  )

  (goal-production-assign-order-and-prio-to-goal ?goal ?order-id ?*PRODUCTION-C2-PRIORITY*)

  (goal-production-assert-requests (create$ ?rs1 ?rs2) ?cs (create$ ?col-ring1 ?col-ring2) ?col-cap ?order-id ?*PRODUCTION-C2-PRIORITY*)
)

(deffunction goal-production-assert-c3
  (?root-id ?order-id ?wp-for-order ?cs ?ds ?bs ?rs1 ?rs2 ?rs3 ?col-cap ?col-base ?col-ring1 ?col-ring2 ?col-ring3)

  (bind ?instruct-goals
    (goal-tree-assert-central-run-parallel-prio INSTRUCT-ORDER ?*PRODUCTION-C3-PRIORITY*
      (goal-production-assert-instruct-bs-dispense-base ?wp-for-order ?col-base INPUT ?order-id ?bs)
      (goal-production-assert-instruct-cs-mount-cap ?cs ?col-cap ?order-id)
      (goal-production-assert-instruct-rs-mount-ring ?rs1 ?col-ring1 ?order-id ONE)
      (goal-production-assert-instruct-rs-mount-ring ?rs2 ?col-ring2 ?order-id TWO)
      (goal-production-assert-instruct-rs-mount-ring ?rs3 ?col-ring3 ?order-id THREE)
    )
  )
  (bind ?instruct-parent (fact-slot-value ?instruct-goals id))
  (bind ?instruct-goals (modify ?instruct-goals (parent ?root-id) (priority ?*PRODUCTION-C3-PRIORITY*)))

  (bind ?goal
    (goal-tree-assert-central-run-all-prio PRODUCE-ORDER ?*PRODUCTION-C3-PRIORITY*
      (goal-production-assert-deliver ?wp-for-order ?order-id ?instruct-parent ?ds)
      (goal-production-assert-mount-cap ?wp-for-order ?cs ?rs3 OUTPUT ?order-id)
      (goal-production-assert-mount-ring ?wp-for-order ?rs3 ?rs2 OUTPUT ?col-ring3 ?order-id THREE)
      (goal-production-assert-mount-ring ?wp-for-order ?rs2 ?rs1 OUTPUT ?col-ring2 ?order-id TWO)
      (goal-production-assert-mount-ring ?wp-for-order ?rs1 ?bs INPUT ?col-ring1 ?order-id ONE)
    )
  )

  (goal-production-assign-order-and-prio-to-goal ?goal ?order-id ?*PRODUCTION-C3-PRIORITY*)

  (goal-production-assert-requests (create$ ?rs1 ?rs2 ?rs3) ?cs (create$ ?col-ring1 ?col-ring2 ?col-ring3) ?col-cap ?order-id ?*PRODUCTION-C3-PRIORITY*)
)

(defrule goal-production-create-instruction-root
  "Create the production root under which all instruction goal trees for the orders
  are asserted"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (domain-facts-loaded)
  (not (goal (class INSTRUCTION-ROOT)))
  (wm-fact (key refbox phase) (value PRODUCTION))
  (wm-fact (key game state) (value RUNNING))
  (wm-fact (key refbox team-color) (value ?color))
  (not (wm-fact (key domain fact rs-ring-spec args? $? rn NA)))
  ; Ensure that a MachineInfo was received already.
  ; So if there are ring stations with specs, then those specs are registered.
  (wm-fact (key domain fact mps-state args? m ?any-mps s IDLE))
  =>
  (bind ?g (goal-tree-assert-central-run-parallel INSTRUCTION-ROOT))
  (modify ?g (meta do-not-finish) (priority 1.0))
)

(defrule goal-production-create-support-root
  "Create the SUPPPORT root, which is parent to all goals that are
   supporting the production, i.e., payment and buffer goals.
  "
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (domain-facts-loaded)
  (not (goal (class SUPPORT-ROOT)))
  (wm-fact (key refbox phase) (value PRODUCTION))
  (wm-fact (key game state) (value RUNNING))
  (wm-fact (key refbox team-color) (value ?color))
  =>
  (bind ?g (goal-tree-assert-central-run-parallel SUPPORT-ROOT))
  (modify ?g (meta do-not-finish) (priority ?*PRODUCTION-C3-PRIORITY*))
)

(defrule goal-production-create-wait-root
  "Create the WAIT root, which has low priority and dispatches WAIT goals if
   nothing else is executable.
  "
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (domain-facts-loaded)
  (not (goal (class WAIT-ROOT)))
  (wm-fact (key refbox phase) (value PRODUCTION))
  (wm-fact (key game state) (value RUNNING))
  (wm-fact (key refbox team-color) (value ?color))
  =>
  (bind ?g (goal-tree-assert-central-run-parallel WAIT-ROOT))
  (modify ?g (meta do-not-finish) (priority 0))
)

(defrule goal-production-assert-wait-nothing-executable
  "When the robot is stuck, assert a new goal that keeps it waiting"
  (goal (id ?p) (class WAIT-ROOT))
  (not (goal (parent ?p) (class WAIT-NOTHING-EXECUTABLE) (mode FORMULATED)))
  =>
  (bind ?goal (assert (goal (class WAIT-NOTHING-EXECUTABLE)
              (id (sym-cat WAIT-NOTHING-EXECUTABLE- (gensym*)))
              (sub-type SIMPLE) (parent ?p) (priority 0.0) (meta-template goal-meta)
              (verbosity NOISY)
  )))
)

(defrule goal-production-create-move-out-of-way
	"Creates a move out of way goal. As soon as it is completed it's reset"
	(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
	(goal (class INSTRUCTION-ROOT) (mode FORMULATED|DISPATCHED))
	(goal (id ?root-id) (class WAIT-ROOT))
	(not (goal (class MOVE-OUT-OF-WAY)))
	(not (wm-fact (key config rcll pick-and-place-challenge) (value TRUE)))
	(navgraph-node (name ?n&:(eq (str-index "-Z" ?n) 2)))
	=>
  (bind ?wait-zones (create$))
  (do-for-all-facts ((?nav navgraph-node))
                    (eq 2 (str-index "-Z" ?nav:name))
    (bind ?wait-zones (insert$ ?wait-zones 1 (goal-production-assert-move-out-of-way  (sym-cat ?nav:name))))
  )
	(bind ?g (goal-tree-assert-central-run-parallel MOVE-OUT-OF-WAY ?wait-zones))
	(modify ?g (parent ?root-id) (priority 1.0))
)

(defrule goal-production-change-priority-move-out-of-way
  ?g <- (goal (id ?goal-id) (class MOVE-OUT-OF-WAY)
              (type ACHIEVE) (sub-type SIMPLE)
              (mode FORMULATED) (parent ?pa-id&~nil)
              (priority ?p&:(or (eq ?p 2) (eq ?p 1)))
        )
  =>
  (printout t "modify priority of " ?goal-id crlf)
  (modify ?g (priority (- ?p 2)))
)

(defrule goal-production-create-cleanup-wp
	"Creates a cleanup-wp goal to get rid of WPs that do not belong to any order,
  or step in the production chain e.g. a workpiece left from stopping to pursue an
  order."
	(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
	(goal (class INSTRUCTION-ROOT) (mode FORMULATED|DISPATCHED))
	(goal (id ?root-id) (class WAIT-ROOT))
	(not (goal (class CLEANUP-WP)))
	=>
	(bind ?g (goal-tree-assert-central-run-parallel CLEANUP-WP))
	(modify ?g (parent ?root-id) (priority 0))
)

(defrule goal-production-debug-cap
  "If there is a mismatch between machines and orders, produce output"
  (wm-fact (key domain fact order-cap-color args? ord ?order-id col ?col))
  (not (wm-fact (key domain fact cs-color args? m ?cs col ?col)))
  (goal (id ?root-id) (class INSTRUCTION-ROOT))
  =>
  (printout error "Can not build order " ?order-id " with cap color " ?col " because there is no capstation for it" crlf)
)

(defrule goal-production-debug-ring1
  "If there is a mismatch between machines and orders, produce output"
  (wm-fact (key domain fact order-ring1-color args? ord ?order-id col ?col&~RING_NONE))
  (not (wm-fact (key domain fact rs-ring-spec args? $? r ?col $?)))
  (goal (id ?root-id) (class INSTRUCTION-ROOT))
  =>
  (printout error "Can not build order " ?order-id " with ring-1 color " ?col " because there is no ringstation for it" crlf)
)

(defrule goal-production-debug-ring2
  "If there is a mismatch between machines and orders, produce output"
  (wm-fact (key domain fact order-ring2-color args? ord ?order-id col ?col-ring&~RING_NONE))
  (not (wm-fact (key domain fact rs-ring-spec args? m ?rs r ?col-ring $?)))
  (goal (id ?root-id) (class INSTRUCTION-ROOT))
  =>
  (printout error "Can not build order " ?order-id " with ring-2 color " ?col-ring " because there is no ringstation for it" crlf)
)

(defrule goal-production-debug-ring3
  "If there is a mismatch between machines and orders, produce output"
  (wm-fact (key domain fact order-ring3-color args? ord ?order-id col ?col-ring&~RING_NONE))
  (not (wm-fact (key domain fact rs-ring-spec args? m ?rs r ?col-ring $?)))
  (goal (id ?root-id) (class INSTRUCTION-ROOT))
  =>
  (printout error "Can not build order " ?order-id " with ring-3 color " ?col-ring " because there is no ringstation for it" crlf)
)

(defrule write-overall-points-into-csv
  "Once the game state switches from PRODUCTION to POST-GAME, write the overal points into csv file"
  (wm-fact (key refbox phase) (value POST_GAME))
  (wm-fact (key refbox points CYAN) (value ?points))
  (wm-fact (key game ID) (values ?ID1 ?ID2))
  ?tf <- (wm-fact (key game overall points) (value 0))
  =>
  (modify ?tf (value 1))
  (simulation-result-into-csv (create$ ?ID1 ?ID2 "Overall" ?points))
)

(defrule ask-next-order-at-same-intervals
    "determine a random next order to be processed every minute or other time interval"
    (wm-fact (key refbox game-time) (values $?gt)) 
    ; (wm-fact (key refbox game-time) (values ?gt&:(= (mod ?gt ?*PRODUCTION-TIME-INTERVAL*) 0) ?)) 
    ; (wm-fact (key refbox game-time) (values $?gt&:(= (mod (+ (float (nth$ 1 ?gt)) (/ (float (nth$ 2 ?gt)) 1000000.)) ?*PRODUCTION-TIME-INTERVAL*) 0))) 
    ; (time $?now)
    ?tf <- (timer (name production-time-interval-timer) (time $?t&:(timeout ?gt ?t ?*PRODUCTION-TIME-INTERVAL*)))

    (wm-fact (key refbox points CYAN) (value ?points))
    (wm-fact (key game ID) (values ?ID1 ?ID2))
    =>
    (modify ?tf (time ?gt))
    ; (assert (wm-fact (key mps workload needs-update) (is-list FALSE) (type BOOL) (value TRUE)))
    ; (printout t "Current time" ?now crlf)
    (printout t crlf)
    (printout t "Result1:all orders" (generate-all-order-list) crlf)
    (bind ?result (generate-candidate-order-list))
    (printout t "Result2:all candidate orders" ?result crlf)
    (bind ?order-id (pick-random-order ?result))
    (printout t "Result3:")
    (if ?order-id then 
      (printout t "the chosen random order:" ?order-id crlf)
      (assert (wm-fact (key strategy meta selected-order args? cond random time (nth$ 1 ?gt)) (value ?order-id)))
    else  
      (printout t "nothing chosen!" crlf)
      (simulation-result-into-csv (create$ ?ID1 ?ID2 ?points "//" " " "//" (print-all-useful-parameters-at-every-decision-moment)))
    )
)
(defrule goal-production-create-produce-for-order
  "Create for each incoming order a grounded production tree with the"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?root-id) (class INSTRUCTION-ROOT) (mode FORMULATED|DISPATCHED))
  (wm-fact (key config rcll pick-and-place-challenge) (value FALSE))
  (wm-fact (key domain fact order-complexity args? ord ?order-id com ?comp))
  (wm-fact (key domain fact order-base-color args? ord ?order-id col ?col-base))
  (wm-fact (key domain fact order-cap-color  args? ord ?order-id col ?col-cap))
  (wm-fact (key domain fact order-ring1-color args? ord ?order-id col ?col-ring1))
  (wm-fact (key domain fact order-ring2-color args? ord ?order-id col ?col-ring2))
  (wm-fact (key domain fact order-ring3-color args? ord ?order-id col ?col-ring3))
  (wm-fact (key domain fact cs-color args? m ?cs col ?col-cap))
  (wm-fact (key domain fact mps-type args? m ?cs t CS))
  (wm-fact (key domain fact mps-type args? m ?bs t BS))
  (not (wm-fact (key order meta wp-for-order args? wp ?something ord ?order-id)))
  (or (wm-fact (key domain fact order-ring1-color args? ord ?order-id col RING_NONE))
      (wm-fact (key domain fact rs-ring-spec args? m ?rs1 r ?col-ring1 $?)))
  (or (wm-fact (key domain fact order-ring2-color args? ord ?order-id col RING_NONE))
      (wm-fact (key domain fact rs-ring-spec args? m ?rs2 r ?col-ring2 $?)))
  (or (wm-fact (key domain fact order-ring3-color args? ord ?order-id col RING_NONE))
      (wm-fact (key domain fact rs-ring-spec args? m ?rs3 r ?col-ring3 $?)))

  ; (or
  ;   (wm-fact (key strategy meta selected-order args? cond filter) (value ?order-id))
  ;   (and
  ;     (time $?now)
  ;     (timer (name production-strategy-nothing-executable-timer) (time $?t&:(timeout ?now ?t ?*PRODUCTION-NOTHING-EXECUTABLE-TIMEOUT*)))
  ;     (wm-fact (key strategy meta selected-order args? cond fallback) (value ?order-id))
  ;   )
  ; )
  (wm-fact (key strategy meta selected-order args? cond random time ?) (value ?order-id))

  ?os <- (wm-fact (key order meta started args? ord ?order-id) (value FALSE))
  (wm-fact (key mps workload needs-update) (value FALSE))

  (wm-fact (key refbox points CYAN) (value ?points))
  (wm-fact (key game ID) (values ?ID1 ?ID2))
  =>
  (printout t "Result4:selected order " ?order-id crlf)
  (printout t "production time interval " ?*PRODUCTION-TIME-INTERVAL* crlf)
  (simulation-result-into-csv (create$ ?ID1 ?ID2 ?points "//" ?order-id "//" (print-all-useful-parameters-at-every-decision-moment)))

  ;find the necessary ringstations
  (bind ?rs1 (goal-production-get-machine-for-color ?col-ring1))
  (bind ?rs2 (goal-production-get-machine-for-color ?col-ring2))
  (bind ?rs3 (goal-production-get-machine-for-color ?col-ring3))

  ;bind the ds to NONE - if there is none, or the actual DS - if there is one
  (bind ?ds NONE)
  (do-for-fact ((?do domain-object) (?df domain-fact))
    (and (eq ?do:type mps) (member$ ?do:name ?df:param-values) (member$ DS ?df:param-values))
    (bind ?ds ?do:name)
  )

  ;create facts for workpiece
  (bind ?wp-for-order (sym-cat wp- ?order-id))
  (assert (domain-object (name ?wp-for-order) (type workpiece))
      (domain-fact (name wp-unused) (param-values ?wp-for-order))
      (wm-fact (key domain fact wp-base-color args? wp ?wp-for-order col BASE_NONE) (type BOOL) (value TRUE))
      (wm-fact (key domain fact wp-cap-color args? wp ?wp-for-order col CAP_NONE) (type BOOL) (value TRUE))
      (wm-fact (key domain fact wp-ring1-color args? wp ?wp-for-order col RING_NONE) (type BOOL) (value TRUE))
      (wm-fact (key domain fact wp-ring2-color args? wp ?wp-for-order col RING_NONE) (type BOOL) (value TRUE))
      (wm-fact (key domain fact wp-ring3-color args? wp ?wp-for-order col RING_NONE) (type BOOL) (value TRUE))
      (wm-fact (key order meta wp-for-order args? wp ?wp-for-order ord ?order-id))
  )
  (if (eq ?comp C0)
    then
    (goal-production-assert-c0 ?root-id ?order-id ?wp-for-order ?cs ?ds ?bs ?col-cap ?col-base)
  )
  (if (and (eq ?comp C1) ?rs1)
    then
    (goal-production-assert-c1 ?root-id ?order-id ?wp-for-order ?cs ?ds ?bs ?rs1 ?col-cap ?col-base ?col-ring1)
  )
  (if (and (eq ?comp C2) ?rs1 ?rs2)
    then
    (goal-production-assert-c2 ?root-id ?order-id ?wp-for-order ?cs ?ds ?bs
                ?rs1 ?rs2 ?col-cap ?col-base ?col-ring1 ?col-ring2)
  )
  (if (and (eq ?comp C3) ?rs1 ?rs2 ?rs3)
    then
    (goal-production-assert-c3 ?root-id ?order-id ?wp-for-order ?cs ?ds ?bs
                ?rs1 ?rs2 ?rs3 ?col-cap ?col-base ?col-ring1 ?col-ring2 ?col-ring3)
  )

  ;clean-up needs update facts
  (delayed-do-for-all-facts
    ((?update-fact wm-fact)) (wm-key-prefix ?update-fact:key (create$ mps workload needs-update))
    (retract ?update-fact)
  )
  (assert (wm-fact (key mps workload needs-update) (is-list FALSE) (type BOOL) (value TRUE)))

  (modify ?os (value TRUE))

  ;if a timer exists, retract it to avoid goal creation spamming
  (delayed-do-for-all-facts
    ((?timer timer)) (eq ?timer:name production-strategy-nothing-executable-timer)
    (retract ?timer)
  )
)

(defrule goal-production-fill-in-unknown-wp-discard-from-cs
  "Fill in missing workpiece information into the discard goals from CS"
  ; there is a discard goal for a CS with formulated assigned goals
  (wm-fact (key request discard args? ord ?order-id cs ?cs prio ?prio) (values status ACTIVE assigned-to ?goal-id ?i-goal-id))

  ?g <- (goal (id ?goal-id) (class DISCARD) (mode FORMULATED) (parent ?parent)
              (params wp UNKNOWN wp-loc ?mps wp-side ?mps-side))
  ?i <- (goal (id ?i-goal-id) (class INSTRUCT-DS-DISCARD) (mode FORMULATED)
	            (params wp UNKNOWN target-mps ?ds))

  ; there is a wp at the machine
  (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side ?mps-side))
  (not (wm-fact (key order meta wp-for-order args? wp ?wp $?)))

  ; there is not another discard goal bound to this wp
  (not (goal (id ?other-goal-id) (class DISCARD) (outcome ~FAILED) (params wp ?wp wp-loc ?mps wp-side ?mps-side)))

  ; there is not a payment goal bound to this wp
  (not (goal (class PAY-FOR-RINGS-WITH-CAP-CARRIER) (params wp ?wp wp-loc ?mps wp-side ?mps-side $?)))
  =>
  (modify ?g (params wp ?wp wp-loc ?mps wp-side ?mps-side))
  (modify ?i (params wp ?wp target-mps ?ds))
)

(defrule goal-production-remove-grounding-from-discard-wp-moved
  "If a DISCARD goal is grounded on a certain WP, but the WP moved, unground it."
  (wm-fact (key request discard args? ord ?order-id cs ?cs prio ?prio) (values status ACTIVE assigned-to ?goal-id ?i-goal-id))

  ?g <- (goal (id ?goal-id) (class DISCARD) (mode FORMULATED) (parent ?parent)
              (params wp ?wp wp-loc ?mps wp-side ?mps-side))
  ?i <- (goal (id ?i-goal-id) (class INSTRUCT-DS-DISCARD) (mode FORMULATED)
	            (params wp ?wp target-mps ?ds))
  (not (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side ?mps-side)))
  (test (neq ?wp UNKNOWN))
  =>
  (modify ?g (params wp UNKNOWN wp-loc ?mps wp-side ?mps-side))
  (modify ?i (params wp UNKNOWN target-mps ?ds))
)

(defrule goal-production-remove-grounding-from-discard-wp-assinged-to-pay
  "If a DISCARD goal is grounded on a certain WP, but the WP is assigned to a payment goal, unground it."
  (wm-fact (key request discard args? ord ?order-id cs ?cs prio ?prio) (values status ACTIVE assigned-to ?goal-id ?i-goal-id))

  ?g <- (goal (id ?goal-id) (class DISCARD) (mode FORMULATED) (parent ?parent)
              (params wp ?wp wp-loc ?mps wp-side ?mps-side))
  ?i <- (goal (id ?i-goal-id) (class INSTRUCT-DS-DISCARD) (mode FORMULATED)
	            (params wp ?wp target-mps ?ds))
  (goal (class PAY-FOR-RINGS-WITH-CAP-CARRIER) (params wp ?wp wp-loc ?mps wp-side ?mps-side $?))
  (test (neq ?wp UNKNOWN))
  =>
  (modify ?g (params wp UNKNOWN wp-loc ?mps wp-side ?mps-side))
  (modify ?i (params wp UNKNOWN target-mps ?ds))
)

(defrule goal-production-assert-enter-field-zones
  (not (wm-fact (key enter-field targets) (values $?zones)))
  (wm-fact (key refbox team-color) (value ?team-color))
  =>
	(if (eq ?team-color CYAN) then
    (assert (wm-fact (key enter-field targets) (is-list TRUE) (type SYMBOL) (values C-Z33 C-Z23 C-Z43)))
  else
    (assert (wm-fact (key enter-field targets) (is-list TRUE) (type SYMBOL) (values M-Z33 M-Z23 M-Z43)))
  )
)


(defrule goal-production-create-enter-field
  "Enter the field (drive outside of the starting box)."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (wm-fact (key central agent robot args? r ?robot))
  (not (wm-fact (key domain fact entered-field args? r ?robot)))
  (not
    (and
      (goal (id ?enter-field) (class ENTER-FIELD))
      (goal-meta (goal-id ?enter-field) (restricted-to ?robot))
    )
  )
  (domain-facts-loaded)
  (wm-fact (key refbox phase) (value PRODUCTION))
  ?targets <- (wm-fact (key enter-field targets) (values ?zone $?zones))
  =>
  (printout t "Goal " ENTER-FIELD " formulated" crlf)
  (goal-production-assert-enter-field ?robot ?zone)
)

(defrule goal-production-remove-enter-field
  "Enter the field (drive outside of the starting box)."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?gf <- (goal (id ?some-goal-id) (class ENTER-FIELD) (mode RETRACTED))
  ?gm <- (goal-meta (goal-id ?some-goal-id))
  =>
  (printout t "Goal " ENTER-FIELD " removed after entering" crlf)
  (retract ?gf ?gm)
)
