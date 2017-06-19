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
		ADD-BASE-SLIDE-ONE ADD-BASE-SLIDE-TWO ADD-BASE-SLIDE-THREE PICK-CC LOAD-CS ADD-RING-ONE ADD-RING-TWO ADD-RING-THREE RESET-SLIDE DISCARD-WP PICK-FROM-CS PICK-FROM-RS PICK-FROM-BS PROD-AT-CS DELIVER-C0 DELIVER-C1 DELIVER-C2 DELIVER-C3 - action
	)
	(:predicates
		(has-ring-color ?rs - ring-station ?r - ring-color)
		(wp-in-slide ?m - ring-station ?n - ring-enum)
		(cap-in-slide ?cs - cap-station ?cap - cap-color)
		(no-cap-in-slide ?cs - cap-station)
		(slide-clear ?cs - cap-station)
		(has-shelf-slot ?cs - cap-station ?ss - shelf-slot)
		(cc-in-slot ?cc - cap-carrier ?s - shelf-slot)
		(has-color ?wp - workpiece ?c - base-color)
		(has-cap ?wp - workpiece ?c - cap-color)
		(has-cap-mounted ?wp - workpiece)
		(has-ring-one ?wp - workpiece ?c - ring-color)
		(has-ring-one-mounted ?wp - workpiece)
		(has-ring-two ?wp - workpiece ?c - ring-color)
		(has-ring-two-mounted ?wp - workpiece)
		(has-ring-three ?wp - workpiece ?c - ring-color)
		(has-ring-three-mounted ?wp - workpiece)
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
	(:action add-base-to-slide-one
		:parameters ( ?r - robot ?m - ring-station ?pos - input-pos ?wp - base)
		:precondition
			(and (locked ?r ADD-BASE-SLIDE-ONE ?pos)(has-pos ?m ?pos)(at-pos ?r ?pos)(holding ?r ?wp)(not (has-ring-one-mounted ?wp)) (not (has-ring-two-mounted ?wp)) (not (has-ring-three-mounted ?wp)) (not (has-cap-mounted ?wp)) (not (and (wp-in-slide ?m ONE)(wp-in-slide ?m TWO)) ) (wp-in-slide ?m ZERO)) 
		:effect
			(and (wp-in-slide ?m ONE)(not (holding ?r ?wp)) (not-holding ?r)(action-done ADD-BASE-SLIDE-ONE)) 
	)
	(:action add-base-to-slide-two
		:parameters ( ?r - robot ?m - ring-station ?pos - input-pos ?wp - base)
		:precondition
			(and (locked ?r ADD-BASE-SLIDE-TWO ?pos)(has-pos ?m ?pos)(at-pos ?r ?pos)(holding ?r ?wp)(not (has-ring-one-mounted ?wp)) (not (has-ring-two-mounted ?wp)) (not (has-ring-three-mounted ?wp)) (not (has-cap-mounted ?wp)) (not (and (wp-in-slide ?m ONE)(wp-in-slide ?m TWO)) ) (wp-in-slide ?m ONE)) 
		:effect
			(and (wp-in-slide ?m TWO)(not (holding ?r ?wp)) (not-holding ?r)(action-done ADD-BASE-SLIDE-TWO)) 
	)
	(:action add-base-to-slide-three
		:parameters ( ?r - robot ?m - ring-station ?pos - input-pos ?wp - base)
		:precondition
			(and (locked ?r ADD-BASE-SLIDE-THREE ?pos)(has-pos ?m ?pos)(at-pos ?r ?pos)(holding ?r ?wp)(not (has-ring-one-mounted ?wp)) (not (has-ring-two-mounted ?wp)) (not (has-ring-three-mounted ?wp)) (not (has-cap-mounted ?wp)) (not (and (wp-in-slide ?m ONE)(wp-in-slide ?m TWO)) ) (wp-in-slide ?m TWO)) 
		:effect
			(and (wp-in-slide ?m ONE)(not (holding ?r ?wp)) (not-holding ?r)(action-done ADD-BASE-SLIDE-THREE)) 
	)
	(:action lock-position
		:parameters ( ?r - robot ?a - action ?dest - pos)
		:precondition
			(and (not-locked ?dest)(no-lock ?r)) 
		:effect
			(and (not (not-locked ?dest)) (not (no-lock ?r)) (locked ?r ?a ?dest)(lock ?r)) 
	)
	(:action unlock-position
		:parameters ( ?r - robot ?a - action ?orig - pos)
		:precondition
			(and (locked ?r ?a ?orig)(lock ?r)(action-done ?a)) 
		:effect
			(and (not (locked ?r ?a ?orig)) (not (lock ?r)) (not (action-done ?a)) (not-locked ?orig)(no-lock ?r)) 
	)
	(:action move-to-position-empty
		:parameters ( ?r - robot ?a - action ?orig - pos ?dest - pos)
		:precondition
			(and (locked ?r ?a ?dest)(at-pos ?r ?orig)(pos-free ?dest)(not-holding ?r)) 
		:effect
			(and (at-pos ?r ?dest)(not (pos-free ?dest)) (not (at-pos ?r ?orig)) (pos-free ?orig)) 
	)
	(:action move-to-position-holding
		:parameters ( ?r - robot ?a - action ?orig - pos ?dest - pos)
		:precondition
			(and (locked ?r ?a ?dest)(at-pos ?r ?orig)(pos-free ?dest)(not (not-holding ?r)) ) 
		:effect
			(and (at-pos ?r ?dest)(not (pos-free ?dest)) (not (at-pos ?r ?orig)) (pos-free ?orig)) 
	)
	(:action pick-cc-from-shelf
		:parameters ( ?r - robot ?m - cap-station ?pos - input-pos ?ss - shelf-slot ?wp - cap-carrier)
		:precondition
			(and (locked ?r PICK-CC ?pos)(at-pos ?r ?pos)(has-pos ?m ?pos)(has-shelf-slot ?m ?ss)(cc-in-slot ?wp ?ss)(not-holding ?r)) 
		:effect
			(and (action-done PICK-CC)(holding ?r ?wp)(not (not-holding ?r)) (not (cc-in-slot ?wp ?ss)) ) 
	)
	(:action load-cs
		:parameters ( ?r - robot ?m - cap-station ?pos - input-pos ?col - cap-color ?cc - cap-carrier)
		:precondition
			(and (locked ?r LOAD-CS ?pos)(has-pos ?m ?pos)(at-pos ?r ?pos)(no-cap-in-slide ?m)(has-cap ?cc ?col)(holding ?r ?cc)(idle ?m)) 
		:effect
			(and (action-done LOAD-CS)(not (holding ?r ?cc)) (not-holding ?r)(cap-in-slide ?m ?col)(not (no-cap-in-slide ?m)) (not (has-cap ?cc ?col)) (wp-in-production ?cc ?m)(not (idle ?m)) ) 
	)
	(:action add-ring-one
		:parameters ( ?r - robot ?m - ring-station ?pos - input-pos ?wp - base ?ring-col - ring-color ?base-col - base-color ?o - order ?num-bases - ring-enum)
		:precondition
			(and (locked ?r ADD-RING-ONE ?pos)(idle ?m)(has-pos ?m ?pos)(at-pos ?r ?pos)(holding ?r ?wp)(has-ring-color ?m ?ring-col)(not (has-ring-one-mounted ?wp)) (not (has-ring-two-mounted ?wp)) (not (has-cap-mounted ?wp)) (has-color ?wp ?base-col)(wp-in-slide ?m ?num-bases)(order-base-color ?o ?base-col)(order-ring-one ?o ?ring-col)(needs-bases ?ring-col ?num-bases)) 
		:effect
			(and (action-done ADD-RING-ONE)(not (holding ?r ?wp)) (not-holding ?r)(wp-in-production ?wp ?m)(not (idle ?m)) (not (wp-in-slide ?m ?num-bases)) (has-ring-one ?wp ?ring-col)(has-ring-one-mounted ?wp)) 
	)
	(:action add-ring-two
		:parameters ( ?r - robot ?m - ring-station ?pos - input-pos ?wp - base ?ring-col1 - ring-color ?ring-col2 - ring-color ?base-col - base-color ?o - order ?num-bases - ring-enum)
		:precondition
			(and (locked ?r ADD-RING-TWO ?pos)(idle ?m)(has-pos ?m ?pos)(at-pos ?r ?pos)(holding ?r ?wp)(has-ring-color ?m ?ring-col2)(has-ring-one ?wp ?ring-col1)(has-ring-one-mounted ?wp)(not (has-ring-two-mounted ?wp)) (not (has-ring-three-mounted ?wp)) (not (has-cap-mounted ?wp)) (has-color ?wp ?base-col)(wp-in-slide ?m ?num-bases)(order-base-color ?o ?base-col)(order-ring-one ?o ?ring-col1)(order-ring-two ?o ?ring-col2)(needs-bases ?ring-col2 ?num-bases)) 
		:effect
			(and (action-done ADD-RING-TWO)(not (holding ?r ?wp)) (not-holding ?r)(wp-in-production ?wp ?m)(not (idle ?m)) (not (wp-in-slide ?m ?num-bases)) (has-ring-two ?wp ?ring-col2)(has-ring-two-mounted ?wp)) 
	)
	(:action add-ring-three
		:parameters ( ?r - robot ?m - ring-station ?pos - input-pos ?wp - base ?ring-col1 - ring-color ?ring-col2 - ring-color ?ring-col3 - ring-color ?base-col - base-color ?o - order ?num-bases - ring-enum)
		:precondition
			(and (locked ?r ADD-RING-THREE ?pos)(idle ?m)(has-pos ?m ?pos)(at-pos ?r ?pos)(holding ?r ?wp)(has-ring-color ?m ?ring-col3)(has-ring-one ?wp ?ring-col1)(has-ring-one-mounted ?wp)(has-ring-two ?wp ?ring-col2)(has-ring-two-mounted ?wp)(not (has-ring-three-mounted ?wp)) (not (has-cap-mounted ?wp)) (has-color ?wp ?base-col)(wp-in-slide ?m ?num-bases)(order-base-color ?o ?base-col)(order-ring-one ?o ?ring-col1)(order-ring-two ?o ?ring-col2)(order-ring-three ?o ?ring-col3)(needs-bases ?ring-col3 ?num-bases)) 
		:effect
			(and (action-done ADD-RING-THREE)(not (holding ?r ?wp)) (not-holding ?r)(wp-in-production ?wp ?m)(not (idle ?m)) (not (wp-in-slide ?m ?num-bases)) (has-ring-three ?wp ?ring-col3)(has-ring-three-mounted ?wp)) 
	)
	(:action reset-rs-slide
		:parameters ( ?r - robot ?m - ring-station)
		:precondition
			(not (wp-in-slide ?m ZERO)) 
		:effect
			(wp-in-slide ?m ZERO)
	)
	(:action prod-at-cs
		:parameters ( ?r - robot ?m - cap-station ?pos - input-pos ?wp - workpiece ?base-col - base-color ?cap-col - cap-color ?o - order)
		:precondition
			(and (locked ?r PROD-AT-CS ?pos)(idle ?m)(has-pos ?m ?pos)(at-pos ?r ?pos)(holding ?r ?wp)(not (has-cap-mounted ?wp)) (has-color ?wp ?base-col)(cap-in-slide ?m ?cap-col)(order-base-color ?o ?base-col)(order-cap-color ?o ?cap-col)) 
		:effect
			(and (action-done PROD-AT-CS)(wp-in-production ?wp ?m)(not (idle ?m)) (not (holding ?r ?wp)) (not-holding ?r)(not (cap-in-slide ?m ?cap-col)) (no-cap-in-slide ?m)(has-cap ?wp ?cap-col)(has-cap-mounted ?wp)) 
	)
	(:action pick-wp-from-cs
		:parameters ( ?r - robot ?m - cap-station ?pos - output-pos ?wp - workpiece)
		:precondition
			(and (locked ?r PICK-FROM-CS ?pos)(has-pos ?m ?pos)(at-pos ?r ?pos)(wp-in-production ?wp ?m)(not-holding ?r)) 
		:effect
			(and (action-done PICK-FROM-CS)(not (wp-in-production ?wp ?m)) (idle ?m)(holding ?r ?wp)(not (not-holding ?r)) ) 
	)
	(:action pick-wp-from-rs
		:parameters ( ?r - robot ?m - ring-station ?pos - output-pos ?wp - base)
		:precondition
			(and (locked ?r PICK-FROM-RS ?pos)(has-pos ?m ?pos)(at-pos ?r ?pos)(wp-in-production ?wp ?m)(not-holding ?r)) 
		:effect
			(and (action-done PICK-FROM-RS)(not (wp-in-production ?wp ?m)) (idle ?m)(holding ?r ?wp)(not (not-holding ?r)) ) 
	)
	(:action discard-wp
		:parameters ( ?r - robot ?wp - cap-carrier)
		:precondition
			(holding ?r ?wp)
		:effect
			(and (not (holding ?r ?wp)) (not-holding ?r)) 
	)
	(:action pick-wp-from-bs
		:parameters ( ?r - robot ?m - base-station ?pos - pos ?wp - base)
		:precondition
			(and (locked ?r PICK-FROM-BS ?pos)(has-pos ?m ?pos)(at-pos ?r ?pos)(wp-at-bs ?wp)(not-holding ?r)) 
		:effect
			(and (action-done PICK-FROM-BS)(holding ?r ?wp)(not (not-holding ?r)) (not (wp-at-bs ?wp)) ) 
	)
	(:action deliver-c0
		:parameters ( ?r - robot ?m - delivery-station ?pos - input-pos ?wp - base ?base-col - base-color ?cap-col - cap-color ?o - order)
		:precondition
			(and (locked ?r DELIVER-C0 ?pos)(has-pos ?m ?pos)(at-pos ?r ?pos)(has-color ?wp ?base-col)(has-cap ?wp ?cap-col)(has-cap-mounted ?wp)(holding ?r ?wp)(order-complexity ?o C0)(order-base-color ?o ?base-col)(order-cap-color ?o ?cap-col)) 
		:effect
			(and (action-done DELIVER-C0)(order-fulfilled ?o)(not (holding ?r ?wp)) (not-holding ?r)(not (has-cap ?wp ?cap-col)) (not (has-color ?wp ?base-col)) ) 
	)
	(:action deliver-c1
		:parameters ( ?r - robot ?m - delivery-station ?pos - input-pos ?wp - base ?base-col - base-color ?cap-col - cap-color ?ring-col1 - ring-color ?o - order)
		:precondition
			(and (locked ?r DELIVER-C1 ?pos)(has-pos ?m ?pos)(at-pos ?r ?pos)(has-color ?wp ?base-col)(has-cap ?wp ?cap-col)(has-cap-mounted ?wp)(has-ring-one ?wp ?ring-col1)(has-ring-one-mounted ?wp)(holding ?r ?wp)(order-complexity ?o C1)(order-base-color ?o ?base-col)(order-cap-color ?o ?cap-col)(order-ring-one ?o ?ring-col1)) 
		:effect
			(and (action-done DELIVER-C1)(order-fulfilled ?o)(not (holding ?r ?wp)) (not-holding ?r)(not (has-cap ?wp ?cap-col)) (not (has-color ?wp ?base-col)) (not (has-ring-one ?wp ?ring-col1)) ) 
	)
	(:action deliver-c2
		:parameters ( ?r - robot ?m - delivery-station ?pos - input-pos ?wp - base ?base-col - base-color ?cap-col - cap-color ?ring-col1 - ring-color ?ring-col2 - ring-color ?o - order)
		:precondition
			(and (locked ?r DELIVER-C2 ?pos)(has-pos ?m ?pos)(has-pos ?m ?pos)(at-pos ?r ?pos)(has-color ?wp ?base-col)(has-cap ?wp ?cap-col)(has-cap-mounted ?wp)(has-ring-one ?wp ?ring-col1)(has-ring-one-mounted ?wp)(has-ring-two ?wp ?ring-col2)(has-ring-two-mounted ?wp)(holding ?r ?wp)(order-complexity ?o C2)(order-base-color ?o ?base-col)(order-cap-color ?o ?cap-col)(order-ring-one ?o ?ring-col1)(order-ring-two ?o ?ring-col2)) 
		:effect
			(and (action-done DELIVER-C2)(order-fulfilled ?o)(not (holding ?r ?wp)) (not-holding ?r)(not (has-cap ?wp ?cap-col)) (not (has-color ?wp ?base-col)) (not (has-ring-one ?wp ?ring-col1)) (not (has-ring-two ?wp ?ring-col2)) ) 
	)
	(:action deliver-c3
		:parameters ( ?r - robot ?m - delivery-station ?pos - input-pos ?wp - base ?base-col - base-color ?cap-col - cap-color ?ring-col1 - ring-color ?ring-col2 - ring-color ?ring-col3 - ring-color ?o - order)
		:precondition
			(and (locked ?r DELIVER-C3 ?pos)(has-pos ?m ?pos)(at-pos ?r ?pos)(has-color ?wp ?base-col)(has-cap ?wp ?cap-col)(has-cap-mounted ?wp)(has-ring-one ?wp ?ring-col1)(has-ring-one-mounted ?wp)(has-ring-two ?wp ?ring-col2)(has-ring-two-mounted ?wp)(has-ring-three ?wp ?ring-col3)(has-ring-three-mounted ?wp)(holding ?r ?wp)(order-complexity ?o C3)(order-base-color ?o ?base-col)(order-cap-color ?o ?cap-col)(order-ring-one ?o ?ring-col1)(order-ring-two ?o ?ring-col2)(order-ring-three ?o ?ring-col3)) 
		:effect
			(and (action-done DELIVER-C3)(order-fulfilled ?o)(not (holding ?r ?wp)) (not-holding ?r)(not (has-cap ?wp ?cap-col)) (not (has-color ?wp ?base-col)) (not (has-ring-one ?wp ?ring-col1)) (not (has-ring-two ?wp ?ring-col2)) (not (has-ring-three ?wp ?ring-col3)) ) 
	)
; MACRO lock-position-move-to-position-empty ACTIONS [lock-position,move-to-position-empty] PARAMETERS [[1,2,3],[1,2,4,3]]
  (:action lock-position-move-to-position-empty :parameters ( ?p1 - robot ?p2 - action ?p3 ?p4 - pos ) :precondition ( and ( not-locked ?p3 ) ( no-lock ?p1 ) ( at-pos ?p1 ?p4 ) ( pos-free ?p3 ) ( not-holding ?p1 ) ) :effect ( and ( at-pos ?p1 ?p3 ) ( not ( pos-free ?p3 ) ) ( not ( at-pos ?p1 ?p4 ) ) ( pos-free ?p4 ) ( not ( not-locked ?p3 ) ) ( not ( no-lock ?p1 ) ) ( locked ?p1 ?p2 ?p3 ) ( lock ?p1 ) ) )
)
