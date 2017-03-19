;TODO replenish shelfs at CS
(define (domain RCLL2016)
  (:requirements :typing :strips)
  (:types
    robot - object

    machine - object
      base-station - machine
      cap-station - machine
      ring-station - machine
      delivery-station - machine
    pos - object
      input-pos - pos
      output-pos - pos
      wait-pos - pos
    shelf-slot - object
    cs-slide - object
    rs-slide - object

    workpiece - object
      cap-carrier - workpiece
      base - workpiece
    mountable - object
      ring - mountable
      cap - mountable
    ring-color - object
    ring-enum - object
    cap-color - object
    base-color - object

    order - object
    complexity - object
  )
  (:constants
    C0 C1 C2 C3 - complexity
    BASE_BLACK BASE_SILVER BASE_RED BASE_CLEAR - base-color
    CAP_BLACK CAP_GRAY - cap-color
    RING_GREEN RING_ORANGE RING_BLUE RING_YELLOW RING_NONE - ring-color
    ZERO ONE TWO THREE - ring-enum
  )
  (:predicates
    (has-color-one ?rs - ring-station ?r - ring-color)
    (has-color-two ?rs - ring-station ?r - ring-color)
    (wp-in-slide ?m - ring-station ?n - ring-enum)

    (cap-in-slide ?cs - cap-station ?cap - cap-color)
    (no-cap-in-slide ?cs - cap-station)
    (slide-clear ?cs - cap-station)
    (has-shelf-slot ?cs - cap-station ?ss - shelf-slot)
    (cc-in-slot ?cc - cap-carrier ?s - shelf-slot)

    (has-color ?wp - workpiece ?c - base-color)
    (has-cap ?wp - workpiece ?c - cap-color)
    (has-ring-one ?wp - workpiece ?c - ring-color)
    (has-ring-two ?wp - workpiece ?c - ring-color)
    (has-ring-three ?wp - workpiece ?c - ring-color)

    (needs-bases ?ring-col - ring-color ?count - ring-enum)

    (base-in-bs ?b - base ?bs - base-station)

    (has-pos ?m - machine ?pos - pos)

    (at-pos ?r - robot ?pos - pos)
    (pos-free ?pos)
    
    (wp-in-production ?wp - workpiece ?m - machine)
    (idle ?m - machine)

    (holding ?r - robot ?wp - workpiece)
    (not-holding ?r - robot)

    (wp ?wp - base)
    (wp-at-bs ?wp - base)

    (order-fulfilled ?o)

    (order-complexity ?o - order ?c - complexity)
    (order-base-color ?o - order ?b - base-color)
    (order-cap-color ?o - order ?c - cap-color)
    (order-ring-one ?o - order ?r - ring-color)
    (order-ring-two ?o - order ?r - ring-color)
    (order-ring-three ?o - order ?r - ring-color)
  )
  ;add bases to RS slide
  (:action add-base-to-slide
    :parameters (?r - robot ?m - ring-station ?pos - input-pos ?wp - base)
    :precondition (and
      (has-pos ?m ?pos)
      (at-pos ?r ?pos)
      (holding ?r ?wp)
      (not
        (and
          (wp-in-slide ?m ONE)
          (wp-in-slide ?m TWO)
        )
      )
    )
    :effect (and
      (when (wp-in-slide ?m ZERO)
        (and 
          (wp-in-slide ?m ONE)
        )
      )
      (when (wp-in-slide ?m ONE)
        (and 
          (wp-in-slide ?m TWO)
          (not (wp-in-slide ?m ONE))
        )
      )
      (when (wp-in-slide ?m TWO)
        (and 
          (wp-in-slide ?m ONE)
        )
      )
      (not (holding ?r ?wp))
      (not-holding ?r)
    )
  )
  (:action move-to-position
    :parameters (?r - robot ?orig - pos ?dest - pos)
    :precondition (and
        (at-pos ?r ?orig)
        (pos-free ?dest)
    )
    :effect (and
        (at-pos ?r ?dest)
        (not (pos-free ?dest))
        (not (at-pos ?r ?orig))
        (pos-free ?orig)
    )
  )
  (:action pick-cc-from-shelf
    :parameters (?r - robot ?m - cap-station ?pos - input-pos ?ss - shelf-slot ?wp - cap-carrier)
    :precondition (and
      (at-pos ?r ?pos)
      (has-pos ?m ?pos)
      (has-shelf-slot ?m ?ss)
      (cc-in-slot ?wp ?ss)
      (not-holding ?r)
    )
    :effect (and
      (holding ?r ?wp)
      (not (not-holding ?r))
      (not (cc-in-slot ?wp ?ss))
    )
  )
  (:action load-cs
    :parameters (?r - robot ?m - cap-station ?pos - input-pos ?col - cap-color ?any-col - cap-color ?cc - cap-carrier)
    :precondition (and
      (has-pos ?m ?pos)
      (at-pos ?r ?pos)
      (no-cap-in-slide ?m)
      (has-cap ?cc ?col)
      (holding ?r ?cc)
      (idle ?m)
    )
    :effect (and
      (not (holding ?r ?cc))
      (not-holding ?r)
      (cap-in-slide ?m ?col)
      (not (no-cap-in-slide ?m))
      (not (has-cap ?cc ?col))
      (wp-in-production ?cc ?m)
      (not (idle ?m))
    )
  )
  (:action add-ring-one
    :parameters (?r - robot ?m - ring-station ?pos - input-pos ?wp - base ?any-ring - ring-color ?ring-col - ring-color ?any-cap - cap-color ?base-col - base-color ?o - order ?num-bases - ring-enum)
    :precondition (and
      (idle ?m)
      (has-pos ?m ?pos)
      (at-pos ?r ?pos)
      (holding ?r ?wp)
      (not (has-ring-one ?wp ?ring-col))
      (not (has-ring-two ?wp ?any-ring))
      (not (has-cap ?wp ?any-cap))
      (has-color ?wp ?base-col)
      (wp-in-slide ?m ?num-bases)
      (order-base-color ?o ?base-col)
      (order-ring-one ?o ?ring-col)
      (needs-bases ?ring-col ?num-bases)
    )
    :effect (and
      (not (holding ?r ?wp))
      (not-holding ?r)
      (wp-in-production ?wp ?m)
      (not (idle ?m))
      (not (wp-in-slide ?m ?num-bases))
      (has-ring-one ?wp ?ring-col)
    )
  )
  (:action add-ring-two
    :parameters (?r - robot ?m - ring-station ?pos - input-pos ?wp - base ?any-ring - ring-color ?ring-col1 - ring-color ?ring-col2 - ring-color ?any-cap - cap-color ?base-col - base-color ?o - order ?num-bases - ring-enum)
    :precondition (and
      (idle ?m)
      (has-pos ?m ?pos)
      (at-pos ?r ?pos)
      (holding ?r ?wp)
      (has-ring-one ?wp ?ring-col1)
      (not (has-ring-two ?wp ?ring-col2))
      (not (has-ring-three ?wp ?any-ring))
      (not (has-cap ?wp ?any-cap))
      (has-color ?wp ?base-col)
      (wp-in-slide ?m ?num-bases)
      (order-base-color ?o ?base-col)
      (order-ring-one ?o ?ring-col1)
      (order-ring-two ?o ?ring-col2)
      (needs-bases ?ring-col2 ?num-bases)
    )
    :effect (and
      (not (holding ?r ?wp))
      (not-holding ?r)
      (wp-in-production ?wp ?m)
      (not (idle ?m))
      (not (wp-in-slide ?m ?num-bases))
      (has-ring-two ?wp ?ring-col2)
    )
  )
  (:action add-ring-three
    :parameters (?r - robot ?m - ring-station ?pos - input-pos ?wp - base ?any - workpiece ?ring-col1 - ring-color ?ring-col2 - ring-color ?ring-col3 - ring-color ?any-cap - cap-color ?base-col - base-color ?o - order ?num-bases - ring-enum)
    :precondition (and
      (idle ?m)
      (has-pos ?m ?pos)
      (at-pos ?r ?pos)
      (holding ?r ?wp)
      (has-ring-one ?wp ?ring-col1)
      (has-ring-two ?wp ?ring-col2)
      (not (has-ring-three ?wp ?ring-col3))
      (not (has-cap ?wp ?any-cap))
      (has-color ?wp ?base-col)
      (wp-in-slide ?m ?num-bases)
      (order-base-color ?o ?base-col)
      (order-ring-one ?o ?ring-col1)
      (order-ring-two ?o ?ring-col2)
      (order-ring-three ?o ?ring-col3)
      (needs-bases ?ring-col3 ?num-bases)
    )
    :effect (and
      (not (holding ?r ?wp))
      (not-holding ?r)
      (wp-in-production ?wp ?m)
      (not (idle ?m))
      (not (wp-in-slide ?m ?num-bases))
      (has-ring-three ?wp ?ring-col3)
    )
  )

  (:action reset-rs-slide
    ; Dummy action to reset RS to zero bases
    :parameters (?m - ring-station)
    :precondition (not (wp-in-slide ?m ZERO))
    :effect (wp-in-slide ?m ZERO)
  )
  (:action prod-at-cs
    :parameters (?r - robot ?m - cap-station ?pos - input-pos ?wp - workpiece ?any - workpiece ?any-cap - cap-color ?base-col - base-color ?cap-col - cap-color ?o - order)
    :precondition (and
      (idle ?m)
      (has-pos ?m ?pos)
      (at-pos ?r ?pos)
      (holding ?r ?wp)
      (not (has-cap ?wp ?any-cap))
      (has-color ?wp ?base-col)
      (cap-in-slide ?m ?cap-col)
      (order-base-color ?o ?base-col)
      (order-cap-color ?o ?cap-col)
    )
    :effect (and
      (wp-in-production ?wp ?m)
      (not (idle ?m))
      (not (holding ?r ?wp))
      (not-holding ?r)
      (not (cap-in-slide ?m ?cap-col))
      (no-cap-in-slide ?m)
      (has-cap ?wp ?cap-col)
    )
  )
  (:action pick-wp-from-cs
    :parameters (?r - robot ?m - cap-station ?pos - output-pos ?wp - workpiece ?any - workpiece)
    :precondition (and
      (has-pos ?m ?pos)
      (at-pos ?r ?pos)
      (wp-in-production ?wp ?m)
      (not-holding ?r)
    )
    :effect (and
      (not (wp-in-production ?wp ?m))
      (idle ?m)
      (holding ?r ?wp)
      (not (not-holding ?r))
    )
  )
  (:action pick-wp-from-rs
    :parameters (?r - robot ?m - ring-station ?pos - output-pos ?wp - base ?any - workpiece)
    :precondition (and
      (has-pos ?m ?pos)
      (at-pos ?r ?pos)
      (wp-in-production ?wp ?m)
      (not-holding ?r)
    )
    :effect (and
      (not (wp-in-production ?wp ?m))
      (idle ?m)
      (holding ?r ?wp)
      (not (not-holding ?r))
    )
  )
  (:action discard-wp
    :parameters (?r - robot ?wp - workpiece)
    :precondition (holding ?r ?wp)
    :effect (and
      (not (holding ?r ?wp))
      (not-holding ?r)
    )
  )
  (:action pick-wp-from-bs
    :parameters (?r - robot ?m - base-station ?pos - pos ?wp - base)
    :precondition (and
      (has-pos ?m ?pos)
      (at-pos ?r ?pos)
      (wp-at-bs ?wp)
      (not-holding ?r)
    )
    :effect (and
      (holding ?r ?wp)
      (not (not-holding ?r))
      (not (wp-at-bs ?wp))
    )
  )
  (:action deliver-c0
    :parameters (?r - robot ?m - delivery-station ?pos - input-pos ?wp - base ?base-col - base-color ?cap-col - cap-color ?o - order)
    :precondition (and
      (has-pos ?m ?pos)
      (at-pos ?r ?pos)
      (has-color ?wp ?base-col)
      (has-cap ?wp ?cap-col)
      (holding ?r ?wp)
      (order-complexity ?o C0)
      (order-base-color ?o ?base-col)
      (order-cap-color ?o ?cap-col)
    )
    :effect (and
      (order-fulfilled ?o)
      (not (holding ?r ?wp))
      (not-holding ?r)
      (not (has-cap ?wp ?cap-col))
      (not (has-color ?wp ?base-col))
    )
  )
  (:action deliver-c1
    :parameters (?r - robot ?m - delivery-station ?pos - input-pos ?wp - base ?base-col - base-color ?cap-col - cap-color ?ring-col1 - ring-color ?o - order)
    :precondition (and
      (has-pos ?m ?pos)
      (at-pos ?r ?pos)
      (has-color ?wp ?base-col)
      (has-cap ?wp ?cap-col)
      (has-ring-one ?wp ?ring-col1)
      (holding ?r ?wp)
      (order-complexity ?o C1)
      (order-base-color ?o ?base-col)
      (order-cap-color ?o ?cap-col)
      (order-ring-one ?o ?ring-col1)
    )
    :effect (and
      (order-fulfilled ?o)
      (not (holding ?r ?wp))
      (not-holding ?r)
      (not (has-cap ?wp ?cap-col))
      (not (has-color ?wp ?base-col))
      (not (has-ring-one ?wp ?ring-col1))
    )
  )
  (:action deliver-c2
    :parameters (?r - robot ?m - delivery-station ?pos - input-pos ?wp - base ?base-col - base-color ?cap-col - cap-color ?ring-col1 - ring-color ?ring-col2 - ring-color ?o - order)
    :precondition (and
      (has-pos ?m ?pos)
      (at-pos ?r ?pos)
      (has-color ?wp ?base-col)
      (has-cap ?wp ?cap-col)
      (has-ring-one ?wp ?ring-col1)
      (has-ring-two ?wp ?ring-col2)
      (holding ?r ?wp)
      (order-complexity ?o C2)
      (order-base-color ?o ?base-col)
      (order-cap-color ?o ?cap-col)
      (order-ring-one ?o ?ring-col1)
      (order-ring-two ?o ?ring-col2)
    )
    :effect (and
      (order-fulfilled ?o)
      (not (holding ?r ?wp))
      (not-holding ?r)
      (not (has-cap ?wp ?cap-col))
      (not (has-color ?wp ?base-col))
      (not (has-ring-one ?wp ?ring-col1))
      (not (has-ring-two ?wp ?ring-col2))
    )
  )
  (:action deliver-c3
    :parameters (?r - robot ?m - delivery-station ?pos - input-pos ?wp - base ?base-col - base-color ?cap-col - cap-color ?ring-col1 - ring-color ?ring-col2 - ring-color ?ring-col3 - ring-color ?o - order)
    :precondition (and
      (has-pos ?m ?pos)
      (at-pos ?r ?pos)
      (has-color ?wp ?base-col)
      (has-cap ?wp ?cap-col)
      (has-ring-one ?wp ?ring-col1)
      (has-ring-two ?wp ?ring-col2)
      (has-ring-three ?wp ?ring-col3)
      (holding ?r ?wp)
      (order-complexity ?o C3)
      (order-base-color ?o ?base-col)
      (order-cap-color ?o ?cap-col)
      (order-ring-one ?o ?ring-col1)
      (order-ring-two ?o ?ring-col2)
      (order-ring-three ?o ?ring-col3)
    )
    :effect (and
      (order-fulfilled ?o)
      (not (holding ?r ?wp))
      (not-holding ?r)
      (not (has-cap ?wp ?cap-col))
      (not (has-color ?wp ?base-col))
      (not (has-ring-one ?wp ?ring-col1))
      (not (has-ring-two ?wp ?ring-col2))
      (not (has-ring-three ?wp ?ring-col3))
    )
  )

)
