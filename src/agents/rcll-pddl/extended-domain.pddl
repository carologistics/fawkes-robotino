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
    action - object
  )
  (:constants
    C0 C1 C2 C3 - complexity
    BASE_BLACK BASE_SILVER BASE_RED BASE_CLEAR - base-color
    CAP_BLACK CAP_GREY - cap-color
    RING_GREEN RING_ORANGE RING_BLUE RING_YELLOW RING_NONE - ring-color
    ZERO ONE TWO THREE - ring-enum
    ADD-BASE-SLIDE-ONE ADD-BASE-SLIDE-TWO ADD-BASE-SLIDE-THREE ; MOVE-TO-EMPTY MOVE-TO-HOLDING
    PICK-CC LOAD-CS ADD-RING-ONE ADD-RING-TWO ADD-RING-THREE RESET-SLIDE DISCARD-WP
    PICK-FROM-CS PICK-FROM-RS PICK-FROM-BS PROD-AT-CS
    DELIVER-C0 DELIVER-C1 DELIVER-C2 DELIVER-C3 - action
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
    (pos-free ?pos - pos)
    
    (locked ?r - robot ?action - action ?orig - pos)
    (lock ?r - robot)
    (not-locked ?dest - pos)
    (no-lock ?r - robot)
    (action-done ?action - action)

    (wp-in-production ?wp - workpiece ?m - machine)
    (idle ?m - machine)

    (holding ?r - robot ?wp - workpiece)
    (not-holding ?r - robot)

    (wp ?wp - base)
    (wp-at-bs ?wp - base)

    (order-fulfilled ?o - order)

    (order-complexity ?o - order ?c - complexity)
    (order-base-color ?o - order ?b - base-color)
    (order-cap-color ?o - order ?c - cap-color)
    (order-ring-one ?o - order ?r - ring-color)
    (order-ring-two ?o - order ?r - ring-color)
    (order-ring-three ?o - order ?r - ring-color)
  )
  (:durative-action add-base-to-slide-one
    :parameters (?r - robot ?m - ring-station ?pos - input-pos ?wp - base)
    :duration (= ?duration 5)
    :precondition (and
      (locked ?r ADD-BASE-SLIDE-ONE ?pos)
      (has-pos ?m ?pos)
      (at-pos ?r ?pos)
      (holding ?r ?wp)
      (not
        (and
          (wp-in-slide ?m ONE)
          (wp-in-slide ?m TWO)
        )
      )
      (wp-in-slide ?m ZERO)
    )
    :effect (and
      (wp-in-slide ?m ONE)
      (not (holding ?r ?wp))
      (not-holding ?r)
      (action-done ADD-BASE-SLIDE-ONE)
    )
  )
  (:durative-action add-base-to-slide-two
    :parameters (?r - robot ?m - ring-station ?pos - input-pos ?wp - base)
    :precondition (and
      (locked ?r ADD-BASE-SLIDE-TWO ?pos)
      (has-pos ?m ?pos)
      (at-pos ?r ?pos)
      (holding ?r ?wp)
      (not
        (and
          (wp-in-slide ?m ONE)
          (wp-in-slide ?m TWO)
        )
      )
      (wp-in-slide ?m ONE)
    )
    :effect (and
      (wp-in-slide ?m TWO)
      (not (holding ?r ?wp))
      (not-holding ?r)
      (action-done ADD-BASE-SLIDE-TWO)
    )
  )
  (:durative-action add-base-to-slide-three
    :parameters (?r - robot ?m - ring-station ?pos - input-pos ?wp - base)
    :precondition (and
      (locked ?r ADD-BASE-SLIDE-THREE ?pos)
      (has-pos ?m ?pos)
      (at-pos ?r ?pos)
      (holding ?r ?wp)
      (not
        (and
          (wp-in-slide ?m ONE)
          (wp-in-slide ?m TWO)
        )
      )
      (wp-in-slide ?m TWO)
    )
    :effect (and
      (wp-in-slide ?m ONE)
      (not (holding ?r ?wp))
      (not-holding ?r)
      (action-done ADD-BASE-SLIDE-THREE)
    )
  )
  (:durative-action lock-position
    :parameters (?r - robot ?a - action ?dest - pos)
    :precondition (and
      (not-locked ?dest)
      (no-lock ?r)
    )
    :effect (and
      (not (not-locked ?dest))
      (not (no-lock ?r))
      (locked ?r ?a ?dest)
      (lock ?r)
    )
    :conditional-breakup (and (no-lock) (not-locked))
  )
  (:durative-action unlock-position
    :parameters (?r - robot ?a - action ?orig - pos)
    :precondition (and
      (locked ?r ?a ?orig)
      (lock ?r)
      (action-done ?a)
    )
    :effect (and
      (not (locked ?r ?a ?orig))
      (not (lock ?r))
      (not (action-done ?a))
      (not-locked ?orig)
      (no-lock ?r)
    )
  )
  (:durative-action move-to-position-empty
    :parameters (?r - robot ?a - action ?orig - pos ?dest - pos)
    :precondition (and
        (locked ?r ?a ?dest)
        (at-pos ?r ?orig)
        (pos-free ?dest)
        (not-holding ?r)
    )
    :effect (and
        (at-pos ?r ?dest)
        (not (pos-free ?dest))
        (not (at-pos ?r ?orig))
        (pos-free ?orig)
    )
    :conditional-breakup (and (at-pos) (not-holding))
    :temporal-breakup (at-pos)
  )
  (:durative-action move-to-position-holding
    :parameters (?r - robot ?a - action ?orig - pos ?dest - pos)
    :precondition (and
        (locked ?r ?a ?dest)
        (at-pos ?r ?orig)
        (pos-free ?dest)
        (not (not-holding ?r))
    )
    :effect (and
        (at-pos ?r ?dest)
        (not (pos-free ?dest))
        (not (at-pos ?r ?orig))
        (pos-free ?orig)
    )
    :conditional-breakup (at-pos)
    :temporal-breakup (at-pos)
  )

  (:durative-action pick-cc-from-shelf
    :parameters (?r - robot ?m - cap-station ?pos - input-pos ?ss - shelf-slot ?wp - cap-carrier)
    :precondition (and
      (locked ?r PICK-CC ?pos)
      (at-pos ?r ?pos)
      (has-pos ?m ?pos)
      (has-shelf-slot ?m ?ss)
      (cc-in-slot ?wp ?ss)
      (not-holding ?r)
    )
    :effect (and
      (action-done PICK-CC)
      (holding ?r ?wp)
      (not (not-holding ?r))
      (not (cc-in-slot ?wp ?ss))
    )
  )
  (:durative-action load-cs
    :parameters (?r - robot ?m - cap-station ?pos - input-pos ?col - cap-color ?any-col - cap-color ?cc - cap-carrier)
    :precondition (and
      (locked ?r LOAD-CS ?pos)
      (has-pos ?m ?pos)
      (at-pos ?r ?pos)
      (no-cap-in-slide ?m)
      (has-cap ?cc ?col)
      (holding ?r ?cc)
      (idle ?m)
    )
    :effect (and
      (action-done LOAD-CS)
      (not (holding ?r ?cc))
      (not-holding ?r)
      (cap-in-slide ?m ?col)
      (not (no-cap-in-slide ?m))
      (not (has-cap ?cc ?col))
      (wp-in-production ?cc ?m)
      (not (idle ?m))
    )
  )
  (:durative-action add-ring-one
    :parameters (?r - robot ?m - ring-station ?pos - input-pos ?wp - base ?any-ring - ring-color ?ring-col - ring-color ?any-cap - cap-color ?base-col - base-color ?o - order ?num-bases - ring-enum)
    :precondition (and
      (locked ?r ADD-RING-ONE ?pos)
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
      (action-done ADD-RING-ONE)
      (not (holding ?r ?wp))
      (not-holding ?r)
      (wp-in-production ?wp ?m)
      (not (idle ?m))
      (not (wp-in-slide ?m ?num-bases))
      (has-ring-one ?wp ?ring-col)
    )
  )
  (:durative-action add-ring-two
    :parameters (?r - robot ?m - ring-station ?pos - input-pos ?wp - base ?any-ring - ring-color ?ring-col1 - ring-color ?ring-col2 - ring-color ?any-cap - cap-color ?base-col - base-color ?o - order ?num-bases - ring-enum)
    :precondition (and
      (locked ?r ADD-RING-TWO ?pos)
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
      (action-done ADD-RING-TWO)
      (not (holding ?r ?wp))
      (not-holding ?r)
      (wp-in-production ?wp ?m)
      (not (idle ?m))
      (not (wp-in-slide ?m ?num-bases))
      (has-ring-two ?wp ?ring-col2)
    )
  )
  (:durative-action add-ring-three
    :parameters (?r - robot ?m - ring-station ?pos - input-pos ?wp - base ?any - workpiece ?ring-col1 - ring-color ?ring-col2 - ring-color ?ring-col3 - ring-color ?any-cap - cap-color ?base-col - base-color ?o - order ?num-bases - ring-enum)
    :precondition (and
      (locked ?r ADD-RING-THREE ?pos)
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
      (action-done ADD-RING-THREE)
      (not (holding ?r ?wp))
      (not-holding ?r)
      (wp-in-production ?wp ?m)
      (not (idle ?m))
      (not (wp-in-slide ?m ?num-bases))
      (has-ring-three ?wp ?ring-col3)
    )
  )
  (:durative-action reset-rs-slide
    ; Dummy action to reset RS to zero bases
    :parameters (?r - robot ?m - ring-station)
    :precondition (not (wp-in-slide ?m ZERO))
    :effect (wp-in-slide ?m ZERO)
  )
  (:durative-action prod-at-cs
    :parameters (?r - robot ?m - cap-station ?pos - input-pos ?wp - workpiece ?any - workpiece ?any-cap - cap-color ?base-col - base-color ?cap-col - cap-color ?o - order)
    :precondition (and
      (locked ?r PROD-AT-CS ?pos)
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
      (action-done PROD-AT-CS)
      (wp-in-production ?wp ?m)
      (not (idle ?m))
      (not (holding ?r ?wp))
      (not-holding ?r)
      (not (cap-in-slide ?m ?cap-col))
      (no-cap-in-slide ?m)
      (has-cap ?wp ?cap-col)
    )
  )
  (:durative-action pick-wp-from-cs
    :parameters (?r - robot ?m - cap-station ?pos - output-pos ?wp - workpiece ?any - workpiece)
    :precondition (and
      (locked ?r PICK-FROM-CS ?pos)
      (has-pos ?m ?pos)
      (at-pos ?r ?pos)
      (wp-in-production ?wp ?m)
      (not-holding ?r)
    )
    :effect (and
      (action-done PICK-FROM-CS)
      (not (wp-in-production ?wp ?m))
      (idle ?m)
      (holding ?r ?wp)
      (not (not-holding ?r))
    )
  )
  (:durative-action pick-wp-from-rs
    :parameters (?r - robot ?m - ring-station ?pos - output-pos ?wp - base ?any - workpiece)
    :precondition (and
      (locked ?r PICK-FROM-RS ?pos)
      (has-pos ?m ?pos)
      (at-pos ?r ?pos)
      (wp-in-production ?wp ?m)
      (not-holding ?r)
    )
    :effect (and
      (action-done PICK-FROM-RS)
      (not (wp-in-production ?wp ?m))
      (idle ?m)
      (holding ?r ?wp)
      (not (not-holding ?r))
    )
  )
  (:durative-action discard-wp
    :parameters (?r - robot ?wp - cap-carrier)
    :precondition (holding ?r ?wp)
    :effect (and
      (not (holding ?r ?wp))
      (not-holding ?r)
    )
  )
  (:durative-action pick-wp-from-bs
    :parameters (?r - robot ?m - base-station ?pos - pos ?wp - base)
    :precondition (and
      (locked ?r PICK-FROM-BS ?pos)
      (has-pos ?m ?pos)
      (at-pos ?r ?pos)
      (wp-at-bs ?wp)
      (not-holding ?r)
    )
    :effect (and
      (action-done PICK-FROM-BS)
      (holding ?r ?wp)
      (not (not-holding ?r))
      (not (wp-at-bs ?wp))
    )
  )
  (:durative-action deliver-c0
    :parameters (?r - robot ?m - delivery-station ?pos - input-pos ?wp - base ?base-col - base-color ?cap-col - cap-color ?o - order)
    :precondition (and
      (locked ?r DELIVER-C0 ?pos)
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
      (action-done DELIVER-C0)
      (order-fulfilled ?o)
      (not (holding ?r ?wp))
      (not-holding ?r)
      (not (has-cap ?wp ?cap-col))
      (not (has-color ?wp ?base-col))
    )
  )
  (:durative-action deliver-c1
    :parameters (?r - robot ?m - delivery-station ?pos - input-pos ?wp - base ?base-col - base-color ?cap-col - cap-color ?ring-col1 - ring-color ?o - order)
    :precondition (and
      (locked ?r DELIVER-C1 ?pos)
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
      (action-done DELIVER-C1)
      (order-fulfilled ?o)
      (not (holding ?r ?wp))
      (not-holding ?r)
      (not (has-cap ?wp ?cap-col))
      (not (has-color ?wp ?base-col))
      (not (has-ring-one ?wp ?ring-col1))
    )
  )
  (:durative-action deliver-c2
    :parameters (?r - robot ?m - delivery-station ?pos - input-pos ?wp - base ?base-col - base-color ?cap-col - cap-color ?ring-col1 - ring-color ?ring-col2 - ring-color ?o - order)
    :precondition (and
      (locked ?r DELIVER-C2 ?pos)
      (has-pos ?m ?pos)
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
      (action-done DELIVER-C2)
      (order-fulfilled ?o)
      (not (holding ?r ?wp))
      (not-holding ?r)
      (not (has-cap ?wp ?cap-col))
      (not (has-color ?wp ?base-col))
      (not (has-ring-one ?wp ?ring-col1))
      (not (has-ring-two ?wp ?ring-col2))
    )
  )
  (:durative-action deliver-c3
    :parameters (?r - robot ?m - delivery-station ?pos - input-pos ?wp - base ?base-col - base-color ?cap-col - cap-color ?ring-col1 - ring-color ?ring-col2 - ring-color ?ring-col3 - ring-color ?o - order)
    :precondition (and
      (locked ?r DELIVER-C3 ?pos)
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
      (action-done DELIVER-C3)
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
