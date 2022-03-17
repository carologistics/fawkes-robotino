(define (domain rcll-goal-planning)

(:requirements :strips :typing)
(:types
  robot - object
	team-color - object
	location - object
  location_ - location
	obstacle - object
	waitpoint - location
	mps - location
	mps - obstacle
	zone - location
	mps-typename - object
	mps-statename - object
	mps-side - object
	base-color - object
	cap-color - object
	ring-color - object
  step - object
	ds-gate - object
	ss-operation - object
	cs-operation - object
	cs-statename - object
	order - object
	order-complexity-value - object
	workpiece - object
	cap-carrier - workpiece
	shelf-spot - object
	;popf has a problem with the type name number and would induce an error
	number_ - object
	ring-num - number_
	ss-shelf - number_
	ss-slot - number_
	token - object
	master-token - token
)

(:constants
  START - location_
  UNKNOWN NONE - obstacle
  UNKNOWN_ROBOT - robot
  BS CS DS RS SS - mps-typename
  IDLE BROKEN PREPARED PROCESSING PROCESSED WAIT-IDLE READY-AT-OUTPUT DOWN - mps-statename
  INPUT OUTPUT WAIT - mps-side
  BASE_NONE BASE_RED BASE_BLACK BASE_SILVER - base-color
  CAP_NONE CAP_BLACK CAP_GREY - cap-color
  GATE-1 GATE-2 GATE-3 - ds-gate
  RING_NONE RING_BLUE RING_GREEN RING_ORANGE RING_YELLOW - ring-color
  RETRIEVE_CAP MOUNT_CAP - cs-operation
  RETRIEVE STORE - ss-operation
  C0 C1 C2 C3 - order-complexity-value
  LEFT MIDDLE RIGHT - shelf-spot
  NA ZERO ONE TWO THREE - ring-num
  ;ZERO ONE TWO THREE FOUR FIVE - ss-shelf
  ;ZERO ONE TWO THREE FOUR FIVE SIX SEVEN - ss-slot
  C-Z18 C-Z28 C-Z38 C-Z48 C-Z58 C-Z68 C-Z78 - zone
  C-Z17 C-Z27 C-Z37 C-Z47 C-Z57 C-Z67 C-Z77 - zone
  C-Z16 C-Z26 C-Z36 C-Z46 C-Z56 C-Z66 C-Z76 - zone
  C-Z15 C-Z25 C-Z35 C-Z45 C-Z55 C-Z65 C-Z75 - zone
  C-Z14 C-Z24 C-Z34 C-Z44 C-Z54 C-Z64 C-Z74 - zone
  C-Z13 C-Z23 C-Z33 C-Z43 C-Z53 C-Z63 C-Z73 - zone
  C-Z12 C-Z22 C-Z32 C-Z42 C-Z52 C-Z62 C-Z72 - zone
  C-Z11 C-Z21 C-Z31 C-Z41 - zone
  M-Z78 M-Z68 M-Z58 M-Z48 M-Z38 M-Z28 M-Z18 - zone
  M-Z77 M-Z67 M-Z57 M-Z47 M-Z37 M-Z27 M-Z17 - zone
  M-Z76 M-Z66 M-Z56 M-Z46 M-Z36 M-Z26 M-Z16 - zone
  M-Z75 M-Z65 M-Z55 M-Z45 M-Z35 M-Z25 M-Z15 - zone
  M-Z74 M-Z64 M-Z54 M-Z44 M-Z34 M-Z24 M-Z14 - zone
  M-Z73 M-Z63 M-Z53 M-Z43 M-Z33 M-Z23 M-Z13 - zone
  M-Z72 M-Z62 M-Z52 M-Z42 M-Z32 M-Z22 M-Z12 - zone
                    M-Z41 M-Z31 M-Z21 M-Z11 - zone

  G-1-1 G-2-1 G-3-1 G-4-1 G-5-1 - waitpoint
  G-1-2 G-2-2 G-3-2 G-4-2 G-5-2 - waitpoint
  G-1-3 G-2-3 G-3-3 G-4-3 G-5-3 - waitpoint
  G-1-4 G-2-4 G-3-4 G-4-4 G-5-4 - waitpoint
  G-1-5 G-2-5 G-3-5 G-4-5 G-5-5 - waitpoint

  ;GET_BASE - step
)
(:predicates
  (wp-moveable-to ?wp - workpiece ?mps - mps ?side - mps-side)
  (at ?r - robot ?m - location ?side - mps-side)
  (holding ?r - robot ?wp - workpiece)
  (can-hold ?r - robot)
  (entered-field ?r - robot)
  (robot-waiting ?r - robot)
  (maps ?m - mps ?r - robot)
  (zone-content ?z - zone ?m - obstacle)
  (mps-type ?m - mps ?t - mps-typename)
  (mps-state ?m - mps ?s - mps-statename)
  (mps-team ?m - mps ?col - team-color)
  (mps-side-free ?m - mps ?side - mps-side)
  (mps-side-approachable ?m - location ?side - mps-side)
  (bs-prepared-color ?m - mps ?col - base-color)
  (bs-prepared-side ?m - mps ?side - mps-side)
  (bs-color ?m - mps ?col - base-color)
  (cs-can-perform ?m - mps ?op - cs-operation)
  (cs-prepared-for ?m - mps ?op - cs-operation)
  (cs-buffered ?m - mps ?col - cap-color)
  (cs-color ?m - mps ?col - cap-color)
  (ss-prepared-for ?m - mps ?op - ss-operation ?wp - workpiece ?shelf - ss-shelf ?slot - ss-slot)
  (rs-prepared-color ?m - mps ?col - ring-color)
  (rs-next-ring-mountable ?wp - workpiece ?mps - mps ?order - order ?ring-color - ring-color)
  (rs-payed-for ?mps - mps ?ring-color - ring-color)
  (rs-ring-spec ?m - mps ?r - ring-color ?rn - ring-num)
  (rs-filled-with ?m - mps ?n - ring-num)
  ;rs-sub and rs-inc are static predicates stating the legal ring-num operations
  (rs-sub ?minuend - ring-num ?subtrahend - ring-num ?difference - ring-num)
  (rs-inc ?summand - ring-num ?sum - ring-num)
  (ds-prepared-order ?m - mps ?ord - order)
  (order-complexity ?ord - order ?com - order-complexity-value)
  (order-base-color ?ord - order ?col - base-color)
  (order-ring-color ?ord - order ?col - ring-color ?ring-num - ring-num)
  (order-ring1-color ?ord - order ?col - ring-color)
  (order-ring2-color ?ord - order ?col - ring-color)
  (order-ring3-color ?ord - order ?col - ring-color)
  (order-cap-color ?ord - order ?col - cap-color)
  (order-fulfilled ?ord - order)
  (order-delivery-begin ?ord - order)
  (order-delivery-end ?ord - order)
  (order-gate ?ord - order ?gate - ds-gate)
  (wp-unused ?wp - workpiece)
  (wp-usable ?wp - workpiece)
  (wp-at ?wp - workpiece ?m - mps ?side - mps-side)
  (wp-base-color ?wp - workpiece ?col - base-color)
  (wp-ring-color ?wp - workpiece ?col - ring-color ?ring-num - ring-num)
  (wp-ring1-color ?wp - workpiece ?col - ring-color)
  (wp-ring2-color ?wp - workpiece ?col - ring-color)
  (wp-ring3-color ?wp - workpiece ?col - ring-color)
  (wp-cap-color ?wp - workpiece ?col - cap-color)
  (wp-on-shelf ?wp - workpiece ?m - mps ?spot - shelf-spot)
  (wp-spawned-for ?wp - workpiece ?r - robot)
  (spot-free ?m - mps ?spot - shelf-spot)
  (ss-stored-wp ?m  - mps ?wp - workpiece ?shelf - ss-shelf ?slot - ss-slot)
  (ss-shelf-slot-free ?m  - mps ?shelf - ss-shelf ?slot - ss-slot)
  (ss-new-wp-at ?m  - mps ?wp - workpiece ?shelf - ss-shelf
                ?slot - ss-slot ?base-col - base-color
                ?ring1-col - ring-color ?ring2-col - ring-color
                ?ring3-col - ring-color ?cap-col - cap-color)
  (wp-get-pending ?wp - workpiece ?m - mps ?side - mps-side)
  ;(order-matching-wp ?wp - workpiece)
  (deliverd ?ord - order)
  (wp-for-order ?wp - workpiece ?ord - order)
  ;(ring-num-mountable ?wp - workpiece ?num - ring-num)
  ;(order-matching-wp-rings ?wp - workpiece)
  (wp-complexity ?wp - workpiece ?n - ring-num)
  (rings-mounted ?wp - workpiece ?n - ring-num)
  (wp-reachable ?wp - workpiece  ?r - robot)
)


  (:action goal-buffer-cap
    :parameters (?target-mps - mps ?wp - workpiece ?robot - robot ?wp-loc - mps ?ss - shelf-spot)
    :precondition (and (mps-type ?target-mps CS)
                       ;(not (mps-state ?target-mps BROKEN))
                       ;(cs-can-perform ?target-mps RETRIEVE_CAP)
                       ;(not (or (cs-buffered ?target-mps CAP_BLACK)
                       ;		 (cs-buffered ?target-mps CAP_GREY)
                       ;	 )
                       ;)
                       (cs-can-perform ?target-mps RETRIEVE_CAP)
                       ;(wp-cap-color ?wp ?cap-color)
                       (wp-reachable ?wp ?robot)
                       (mps-side-free ?target-mps INPUT)
                       (wp-on-shelf ?wp ?wp-loc ?ss)
                  )
    :effect (and
              (not (mps-side-free ?target-mps INPUT))
              (wp-at ?wp ?target-mps INPUT)
              (not (wp-on-shelf ?wp ?wp-loc ?ss))
            )
  )

  (:action goal-instruct-cs-buffer-cap
    :parameters (?target-mps - mps ?cap-color - cap-color ?cc - workpiece)
    :precondition (and (mps-type ?target-mps CS)
                       ;(not (mps-state ?target-mps BR(wp-reachable ?wp ?robot)OKEN))
                       ;(cs-can-perform ?target-mps RETRIEVE_CAP)
                       ;(not (or (cs-buffered ?target-mps CAP_BLACK)
                       ;		 (cs-buffered ?target-mps CAP_GREY)
                       ;	 )
                       ;)
                       (cs-can-perform ?target-mps RETRIEVE_CAP)
                       (wp-cap-color ?cc ?cap-color)
                       (wp-at ?cc ?target-mps INPUT)
                       (mps-side-free ?target-mps OUTPUT)
                  )
    :effect (and
              (not (wp-at ?cc ?target-mps INPUT))
              (mps-side-free ?target-mps INPUT)
              (wp-at ?cc ?target-mps OUTPUT)
              (not (mps-side-free ?target-mps OUTPUT))
              (not (wp-cap-color ?cc ?cap-color))
              (wp-cap-color ?cc CAP_NONE)
              (cs-buffered ?target-mps ?cap-color)
              (not (cs-buffered ?target-mps CAP_NONE))
              (cs-can-perform ?target-mps MOUNT_CAP)
              (not (cs-can-perform ?target-mps RETRIEVE_CAP))
            )
  )

  (:action goal-instruct-bs-dispense-base
    :parameters (?wp - workpiece ?target-mps - mps ?base-color - base-color ?robot - robot)
    :precondition (and (mps-type ?target-mps BS)
                       ;(mps-state ?target-mps IDLE)
                       ;(mps-side-free ?target-mps INPUT)
                       (mps-side-free ?target-mps OUTPUT)
                       ;(mps-side-free ?target-mps WAIT)
                       (wp-unused ?wp)
                       ;(next-step ?wp GET_BASE)
                       (wp-base-color ?wp BASE_NONE)
                       ;(wp-get-pending ?wp ?target-mps OUTPUT)
                       ;(mps-free ?target-mps)
                  )
    :effect (and
                 (not (mps-side-free ?target-mps OUTPUT))
                 (wp-at ?wp ?target-mps OUTPUT)
                 (not (wp-base-color ?wp BASE_NONE))
                 (wp-base-color ?wp ?base-color)
                 (not (wp-unused ?wp))
                 (wp-reachable ?wp ?robot)
                 ;(ring-num-mountable ?wp ONE)
            )
  )


	;(:action NOOP-assert-wp-get-pending
	;	:parameters (?wp - workpiece ?target-mps - mps ?target-side - mps-side)
	;	:precondition ()
	;	:effect (wp-get-pending ?wp ?target-mps ?target-side)
	;)

  (:action goal-mount-cap
    :parameters (?wp - workpiece ?target-mps - mps  ?robot - robot ?cap-color - cap-color ?wp-loc - mps ?ring-num - ring-num ?wp-loc-side - mps-side)
    :precondition (and (mps-type ?target-mps CS)
                       ;(not (mps-state ?target-mps BROKEN))
                       ;(not (wp-at ?any-wp  ?target-mps INPUT))
                       ;(mps-side-free ?target-mps INPUT)
                       ;(mps-team ?target-mps ?team-color)
                       ;(or (cs-buffered ?target-mps CAP_BLACK)
                       ;		 (cs-buffered ?target-mps CAP_GREY)
                       ;)
                       (cs-buffered ?target-mps ?cap-color)
                       (cs-can-perform ?target-mps MOUNT_CAP)
                       ;(mps-team ?wp-loc ?team-color)
                       ;(holding ?robot ?wp)
                       (wp-reachable ?wp ?robot)
                       (mps-side-free ?target-mps INPUT)
                       (wp-at ?wp ?wp-loc ?wp-loc-side)
                       (rings-mounted ?wp ?ring-num)
                       (wp-complexity ?wp ?ring-num)
                       ;(order-matching-wp-rings ?wp)
               )
    :effect (and
                (not (mps-side-free ?target-mps INPUT))
                (wp-at ?wp ?target-mps INPUT)
                (not (wp-at ?wp ?wp-loc ?wp-loc-side))
                (mps-side-free ?wp-loc ?wp-loc-side)
            )
  )

  (:action goal-instruct-cs-mount-cap
    :parameters (?mps - mps ?cap-color - cap-color ?wp - workpiece)
    :precondition (and
        ;(not (mps-state ?mps BROKEN))
        (mps-type ?mps CS)
        (cs-can-perform ?mps MOUNT_CAP)
        ;(or (cs-buffered ?mps CAP_BLACK)
        ;		(cs-buffered ?mps CAP_GREY)
        ;)
        (cs-buffered ?mps ?cap-color)
        (mps-side-free ?mps OUTPUT)
        (wp-at ?wp ?mps INPUT)
        (wp-cap-color ?wp CAP_NONE)
        ;(mps-occupied ?mps ?wp)
      )
    :effect (and
        ;(not (mps-state ?mps IDLE))
        ;(mps-state ?mps READY-AT-OUTPUT)
        (not (wp-at ?wp ?mps INPUT))
        (mps-side-free ?mps INPUT)
        (wp-at ?wp ?mps OUTPUT)
        (not (mps-side-free ?mps OUTPUT))
        (not (wp-cap-color ?wp CAP_NONE))
        (wp-cap-color ?wp ?cap-color)
        (cs-can-perform ?mps RETRIEVE_CAP)
        (not (cs-can-perform ?mps MOUNT_CAP))
        (not (cs-buffered ?mps ?cap-color))
        (cs-buffered ?mps CAP_NONE)
      )
  )


  ;(:action NOOP-assert-order-match-wp
  ;  :parameters (?wp - workpiece ?ord - order ?ring1 - ring-color ?ring2 - ring-color ?ring3 - ring-color ?base-color - base-color ?cap-color - cap-color)
  ;  :precondition (and ;(wm-fact (key domain fact mps-team args? m ?wp-loc col ?team-color))
  ;                   (order-ring-color ?ord ?ring1 ONE)
  ;                   (order-ring-color ?ord ?ring2 TWO)
  ;                   (order-ring-color ?ord ?ring3 THREE)
  ;                   (order-base-color ?ord ?base-color)
  ;                   (order-cap-color ?ord ?cap-color)
  ;                   (wp-ring-color ?wp ?ring1 ONE)
  ;                   (wp-ring-color ?wp ?ring2 TWO)
  ;                   (wp-ring-color ?wp ?ring3 THREE)
  ;                   (wp-base-color ?wp ?base-color)
  ;                   (wp-cap-color ?wp ?cap-color)
  ;                   (wp-for-order ?wp ?ord)
  ;                )
  ;  :effect (order-matching-wp ?wp)
  ;)

  ;(:action NOOP-assert-order-match-wp-rings
  ;  :parameters (?wp - workpiece ?ord - order ?ring1 - ring-color ?ring2 - ring-color ?ring3 - ring-color ?base-color - base-color)
  ;  :precondition (and ;(wm-fact (key domain fact mps-team args? m ?wp-loc col ?team-color))
  ;                   (order-ring-color ?ord ?ring1 ONE)
  ;                   (order-ring-color ?ord ?ring2 TWO)
  ;                   (order-ring-color ?ord 1?ring3 THREE)
  ;                   (order-base-color ?ord ?base-color)
  ;                   (wp-ring-color ?wp ?ring1 ONE)
  ;                   (wp-ring-color ?wp ?ring2 TWO)
  ;                   (wp-ring-color ?wp ?ring3 THREE)
  ;                   (wp-base-color ?wp ?base-color)
  ;                   (wp-for-order ?wp ?ord)
  ;                )
  ;  :effect (order-matching-wp-rings ?wp)
  ;)

  (:action goal-deliver-rc21
    :parameters (?wp - workpiece ?target-mps - mps ?ord - order ?robot - robot ?wp-side - mps-side  ?ring1 - ring-color ?ring2 - ring-color ?ring3 - ring-color ?base-color - base-color ?cap-color - cap-color)
    :precondition (and ;(mps-type ?target-mps DS)
                       ;(mps-state ?target-mps IDLE)
                       ;(wp-at ?wp ?target-mps INPUT)
                       ;(wm-fact (key domain fact mps-type args? m ?wp-loc t ?))
                       ;(wm-fact (key domain fact mps-team args? m ?wp-loc col ?team-color))
                       (wp-reachable ?wp ?robot)
                       ;(order-matching-wp ?wp)
                       (wp-at ?wp ?target-mps ?wp-side)

                       (order-ring-color ?ord ?ring1 ONE)
                       (order-ring-color ?ord ?ring2 TWO)
                       (order-ring-color ?ord ?ring3 THREE)
                       (order-base-color ?ord ?base-color)
                       (order-cap-color ?ord ?cap-color)
                       (wp-ring-color ?wp ?ring1 ONE)
                       (wp-ring-color ?wp ?ring2 TWO)
                       (wp-ring-color ?wp ?ring3 THREE)
                       (wp-base-color ?wp ?base-color)
                       (wp-cap-color ?wp ?cap-color)
                       (wp-for-order ?wp ?ord)
                  )
    :effect (and
              (deliverd ?ord)
              (not (wp-at ?wp ?target-mps ?wp-side))
              (mps-side-free ?target-mps ?wp-side)
            )
  )

  ;(:action debug-deliver-a-to-b
  ;  :parameters (?wp - workpiece ?wp-loc - mps ?wp-loc-side - mps-side ?wp-to - mps ?wp-to-side - mps-side)
  ;  :precondition (and
  ;        (wp-at ?wp ?wp-loc ?wp-loc-side)
  ;        (mps-side-free ?wp-to ?wp-to-side)
  ;    )
  ;  :effect (and
  ;      (not (wp-at ?wp ?wp-loc ?wp-loc-side))
  ;      (not (mps-side-free ?wp-to ?wp-to-side))
  ;      (wp-at ?wp ?wp-to ?wp-to-side)
  ;      (mps-side-free ?wp-loc ?wp-loc-side)
  ;    )
  ;)

  (:action goal-mount-next-ring
    :parameters (?robot - robot ?rs - mps ?wp - workpiece ?ring-color - ring-color ?ring-payment-cost - ring-num ?ring-num - ring-num ?wp-loc - mps ?ord - order ?wp-loc-side - mps-side ?ring-num-next - ring-num)
    :precondition (and
            ;(not (mps-state ?rs BROKEN))
            ;(rs-paid-for ?rs ?bases-needed)
            ;(mps-side-free ?rs INPUT)
            (mps-type ?rs RS)
            (rs-ring-spec ?rs ?ring-color ?ring-payment-cost)
            ;(rs-filled-with ?rs ?rings-filled)
            ;(rs-sub ?rings-filled ?ring-payment-cost ?bases-left)
            ;(ring-num-mountable ?wp ?ring-num)
            (rings-mounted ?wp ?ring-num)
            (rs-inc ?ring-num ?ring-num-next)
            (wp-cap-color ?wp CAP_NONE)
            (wp-for-order ?wp ?ord)
            (order-ring-color ?ord ?ring-color ?ring-num-next)
            ;(and)
            ;    (wp-ring1-color ?wp RING_NONE)
            ;    ;(wp-ring2-color ?wp RING_NONE)
            ;    ;(wp-ring3-color ?wp RING_NONE)
            ;    (wp-base-color ?wp ?base-color)
            ;    (wp-cap-color ?wp CAP_NONE)
            ;    (order-base-color ?ord ?base-color)
            ;)

            ;(wp-for-order ?wp ?ord)
            ;(order-ring1-color ?ord ?ring-color)
            ;(mps-has-side ?bs ?side)
            ;missing deadlock prevention()
            ;(order-producible ?order)
            ;(and)
            ;  (can-hold ?robot)
            ;  (wp-at ?wp ?wp-loc ?wp-side)
            ;)
            (wp-reachable ?wp ?robot)
            (wp-at ?wp ?wp-loc ?wp-loc-side)
            (mps-side-free ?rs INPUT)
        )
    :effect (and
        ;(mps-occupied ?rs ?wp)
        ;(not (mps-free ?rs))
        ;(not (mps-occupied ?wp-loc ?wp))
        ;(mps-free ?wp-loc)
        (not (wp-at ?wp ?wp-loc ?wp-loc-side))
        ;(not (rs-filled-with ?rs ?rings-filled))
        ;(rs-filled-with ?rs ?bases-left)
        (not (mps-side-free ?rs INPUT))
        (wp-at ?wp ?rs INPUT)
        (mps-side-free ?wp-loc ?wp-loc-side)
        ;(not (holding ?robot ?wp))
        ;(can-hold ?robot)
    )
  )

  ;(:action assert-facts
  ;    :parameters (?wp - workpiece ?ord - order ?mps - mps)
  ;    :precondition (mps-type ?mps SS
  ;    )
  ;    :effect (and
  ;      (rs-ring-spec ?mps RING_YELLOW ONE)
  ;      (wp-for-order ?wp ?ord)
  ;      (mps-type ?mps RS)
  ;      (order-ring-color ?ord RING_YELLOW TWO)
  ;    )
  ; )


  (:action goal-instruct-rs-mount-ring
    :parameters (?rs - mps ?wp - workpiece ?ring-color - ring-color ?ring-payment-cost - ring-num ?rings-filled - ring-num ?bases-left - ring-num ?ring-num - ring-num ?ring-num-next - ring-num)
    :precondition (and
          ;(not (mps-state ?rs BROKEN))
          ;(rs-paid-for ?rs ?bases-needed)
          (wp-at ?wp ?rs INPUT)
          (mps-side-free ?rs OUTPUT)
          ;ring-mountable()
          (mps-type ?rs RS)
          (rs-ring-spec ?rs ?ring-color ?ring-payment-cost)
          ;(sufficient-payment ?rs ?ring-color)
          (rs-filled-with ?rs ?rings-filled)
          (rs-sub ?rings-filled ?ring-payment-cost ?bases-left)
          ;(ring-num-mountable ?wp ?ring-num)
          (wp-cap-color ?wp CAP_NONE)
          (rings-mounted ?wp ?ring-num)
          (rs-inc ?ring-num ?ring-num-next)
          ;(and)
          ;    (wp-ring1-color ?wp RING_NONE)
          ;    ;(wp-ring2-color ?wp RING_NONE)
          ;    ;(wp-ring3-color ?wp RING_NONE)
          ;    (wp-base-color ?wp ?base-color)
          ;    (wp-cap-color ?wp CAP_NONE)
          ;    (order-base-color ?ord ?base-color)
          ;)

          ;(wp-for-order ?wp ?ord)
          ;(order-ring1-color ?ord ?ring-color)
          ;(mps-has-side ?bs ?side)
          ;missing deadlock prevention()
          ;(order-producible ?order)
          ;(mps-occupied ?rs ?wp)
      )
    :effect (and
          ;(not (rs-filled-with ?rs ?rings-filled))
          ;(rs-filled-with ?rs ?bases-left)
          (mps-side-free ?rs INPUT)
          (not (wp-at ?wp ?rs INPUT))
          (not (mps-side-free ?rs OUTPUT))
          (wp-at ?wp ?rs OUTPUT)
          (wp-ring-color ?wp ?ring-color ?ring-num-next)
          (not (wp-ring-color ?wp RING_NONE ?ring-num-next))
          ;(not (ring-num-mountable ?wp ?ring-num-next))
          (not (rs-filled-with ?rs ?rings-filled))
          (rs-filled-with ?rs ?bases-left)
          (rings-mounted ?wp ?ring-num-next)
          (not (rings-mounted ?wp ?ring-num))
      )
  )

  ;(:action NOOP-ring-num-two-mountable
  ;  :parameters (?wp - workpiece ?ord - order ?ring1 - ring-color ?ring2 - ring-color)
  ;  :precondition (and
  ;                   (wp-for-order ?wp ?ord)
  ;                   (wp-ring-color ?wp ?ring1 ONE)
  ;                   (wp-ring-color ?wp RING_NONE TWO)
  ;                   (order-ring-color ?ord ?ring1 ONE)
  ;                   (order-ring-color ?ord ?ring2 TWO)
  ;                )
  ;  :effect (ring-num-mountable ?wp TWO)
  ;)

  ;(:action NOOP-ring-num-three-mountable
  ;  :parameters (?wp - workpiece ?ord - order ?ring1 - ring-color ?ring2 - ring-color ?ring3 - ring-color)
  ;  :precondition (and
  ;                   (wp-for-order ?wp ?ord)
  ;                   (wp-ring-color ?wp ?ring1 ONE)
  ;                   (wp-ring-color ?wp ?ring2 TWO)
  ;                   (wp-ring-color ?wp RING_NONE THREE)
  ;                   (order-ring-color ?ord ?ring1 ONE)
  ;                   (order-ring-color ?ord ?ring2 TWO)
  ;                   (order-ring-color ?ord ?ring3 THREE)
  ;                )
  ;  :effect (ring-num-mountable ?wp THREE)
  ;)

  ;(:action NOOP-fill-payment
  ;    :parameters (?rs - mps ?rings-filled - ring-num ?rings-inc - ring-num)
  ;    :precondition (and
  ;          (mps-type ?rs RS)
  ;          (rs-filled-with ?rs ?rings-filled)
  ;          (rs-inc ?rings-filled ?rings-inc)
  ;    )
  ;    :effect (and
  ;          (not (rs-filled-with ?rs ?rings-filled))
  ;          (rs-filled-with ?rs ?rings-inc)
  ;    )
  ;)


  (:action goal-get-base-to-fill-rs-wp-at-mps
    :parameters (?wp - workpiece ?wp-loc - mps ?wp-side - mps-side ?target-mps - mps ?robot - robot ?bases-filled - ring-num ?bases-inc - ring-num)
    :precondition (and
                      (mps-type ?target-mps RS)

                      (wp-reachable ?wp ?robot)
                      (wp-at ?wp ?wp-loc ?wp-side)
                      (rs-filled-with ?target-mps ?bases-filled)
                      (rs-inc ?bases-filled ?bases-inc)
                      ;(wp-cap-color ?wp CAP_NONE)
                  )
    :effect (and
          (not (wp-at ?wp ?wp-loc ?wp-side))
          (mps-side-free ?wp-loc ?wp-side)
          (not (wp-reachable ?wp ?robot))
          (not (rs-filled-with ?target-mps ?bases-filled))
          (rs-filled-with ?target-mps ?bases-inc)
      )
  )

  (:action goal-get-base-to-fill-rs-wp-on-shelf
  :parameters (?wp - workpiece ?wp-loc - mps ?spot - shelf-spot ?target-mps - mps ?robot - robot ?bases-filled - ring-num ?bases-inc - ring-num)
  :precondition (and
                    (mps-type ?target-mps RS)

                    (wp-reachable ?wp ?robot)
                    (wp-on-shelf ?wp ?wp-loc ?spot)
                    (rs-filled-with ?target-mps ?bases-filled)
                    (rs-inc ?bases-filled ?bases-inc)
                    ;(wp-cap-color ?wp CAP_NONE)
                )
  :effect (and
        (not (wp-on-shelf ?wp ?wp-loc ?spot))
        (not (wp-reachable ?wp ?robot))
        (not (rs-filled-with ?target-mps ?bases-filled))
        (rs-filled-with ?target-mps ?bases-inc)
    )
)

  (:action goal-discard-from-mps
    :parameters (?wp - workpiece ?mps - mps ?robot - robot ?wp-side - mps-side)
    :precondition (and ;(wm-fact (key domain fact mps-team args? m ?wp-loc col ?team-color))
                       (wp-reachable ?wp ?robot)
                       (wp-at ?wp ?mps ?wp-side)
                  )
    :effect (and (not (wp-at ?wp ?mps ?wp-side))
                 (mps-side-free ?mps ?wp-side)
                 (not (wp-reachable ?wp ?robot))
            )
  )

)
