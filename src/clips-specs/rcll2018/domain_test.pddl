(define (domain rcll-production)
	(:requirements :strips :typing)

	(:types
		robot - object
		team-color - object
		location - object
    	waitpoint - location
		mps - location
		mps-typename - object
		mps-statename - object
		mps-side - object
		base-color - object
		cap-color - object
		ring-color - object
		ds-gate - object
		cs-operation - object
		cs-statename - object
		order - object
    	order-complexity-value - object
		workpiece - object
		cap-carrier - workpiece
		shelf-spot - object
		ring-num - object
		zone - object
		token - object
		master-token - token
		component - object
		state - object
	)

	(:constants
		START - location
		BS CS DS RS SS - mps-typename
		IDLE BROKEN PREPARED PROCESSING PROCESSED WAIT-IDLE READY-AT-OUTPUT DOWN - mps-statename
		;INPUT OUTPUT - mps-side
		BASE_NONE BASE_RED BASE_BLACK BASE_SILVER - base-color
		CAP_NONE CAP_BLACK CAP_GREY - cap-color
		GATE-1 GATE-2 GATE-3 - ds-gate
		RING_NONE RING_BLUE RING_GREEN RING_ORANGE RING_YELLOW - ring-color
		RETRIEVE_CAP MOUNT_CAP - cs-operation
		C0 C1 C2 C3 - order-complexity-value
		LEFT MIDDLE RIGHT - shelf-spot
		NA ZERO ONE TWO THREE - ring-num
	)

	(:predicates
		(self ?r - robot)
		(at ?r - robot ?m - location ?side - mps-side)
		(holding ?r - robot ?wp - workpiece)
		(can-hold ?r - robot)
		(entered-field ?r - robot)
		(location-free ?l - location ?side - mps-side)
		(robot-waiting ?r - robot)
		(mps-type ?m - mps ?t - mps-typename)
		(mps-state ?m - mps ?s - mps-statename)
		(mps-team ?m - mps ?col - team-color)
		(bs-prepared-color ?m - mps ?col - base-color)
		(bs-prepared-side ?m - mps ?side - mps-side)
		(cs-can-perform ?m - mps ?op - cs-operation)
		(cs-prepared-for ?m - mps ?op - cs-operation)
		(cs-buffered ?m - mps ?col - cap-color)
		(cs-color ?m - mps ?col - cap-color)
		(cs-free ?m - mps)
		(rs-prepared-color ?m - mps ?col - ring-color)
		(rs-ring-spec ?m - mps ?r - ring-color ?rn - ring-num)
		(rs-filled-with ?m - mps ?n - ring-num)
		;rs-sub and rs-inc are static predicates stating the legal ring-num operations
		(rs-sub ?minuend - ring-num ?subtrahend - ring-num ?difference - ring-num)
		(rs-inc ?summand - ring-num ?sum - ring-num)
		(ds-prepared-gate ?m - mps ?g - ds-gate)
		(order-complexity ?ord - order ?com - order-complexity-value)
		(order-base-color ?ord - order ?col - base-color)
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
		(wp-ring1-color ?wp - workpiece ?col - ring-color)
		(wp-ring2-color ?wp - workpiece ?col - ring-color)
		(wp-ring3-color ?wp - workpiece ?col - ring-color)
		(wp-cap-color ?wp - workpiece ?col - cap-color)
		(wp-on-shelf ?wp - workpiece ?m - mps ?spot - shelf-spot)
		(wp-spawned-for ?wp - workpiece ?r - robot)
    	(spot-free ?m - mps ?spot - shelf-spot)
		(comp-state ?comp - component ?state - state)

    (locked ?name - object)
    (location-locked ?m - mps ?s - mps-side)

				(next-reset-mps ?m - mps)
		(last-reset-mps ?m - mps)

		(next-prepare-bs ?m - mps ?side - mps-side ?bc - base-color)
		(last-prepare-bs ?m - mps ?side - mps-side ?bc - base-color)

		(next-prepare-ds ?m - mps ?gate - ds-gate)
		(last-prepare-ds ?m - mps ?gate - ds-gate)

		(next-prepare-cs ?m - mps ?op - cs-operation)
		(last-prepare-cs ?m - mps ?op - cs-operation)

		(next-bs-dispense ?r - robot ?m - mps ?side - mps-isde ?wp - workpiece ?basecol - base-color)
		(last-bs-dispense ?r - robot ?m - mps ?side - mps-isde ?wp - workpiece ?basecol - base-color)
	
		(next-cs-mount-cap ?m - mps ?wp - workpiece ?capcol - cap-color)
		(last-cs-mount-cap ?m - mps ?wp - workpiece ?capcol - cap-color)

		(next-cs-retrieve-cap ?m - mps ?cc - cap-carrier ?capcol - cap-color)
		(last-cs-retrieve-cap ?m - mps ?cc - cap-carrier ?capcol - cap-color)
	
		(next-prepare-rs ?m - mps ?rc - ring-color)
		(last-prepare-rs ?m - mps ?rc - ring-color)

		(next-rs-mount-ring1 ?m - mps ?wp - workpiece ?col - ring-color)
		(last-rs-mount-ring1 ?m - mps ?wp - workpiece ?col - ring-color)
	
		(next-rs-mount-ring2 ?m - mps ?wp - workpiece ?col - ring-color ?col1 - ring-color)
		(last-rs-mount-ring2 ?m - mps ?wp - workpiece ?col - ring-color ?col1 - ring-color)
	
		(next-rs-mount-ring3 ?m - mps ?wp - workpiece ?col - ring-color ?col1 - ring-color ?col2 - ring-color)
		(last-rs-mount-ring3 ?m - mps ?wp - workpiece ?col - ring-color ?col1 - ring-color ?col2 - ring-color)
	
		(next-go-wait ?r - robot ?to - waitpoint)
		(last-go-wait ?r - robot ?to - waitpoint)
	
		(next-wait ?r - robot)
		(last-wait ?r - robot)

		(next-move ?r - robot ?to - mps ?to-side - mps-side)
		(last-move ?r - robot ?to - mps ?to-side - mps-side)

		(next-enter-field ?r - robot)
		(last-enter-field ?r - robot)

		(next-wp-discard ?r - robot)
		(last-wp-discard ?r - robot)

		(next-wp-get-shelf ?r - robot)
		(last-wp-get-shelf ?r - robot)

		(next-wp-get ?r - robot)
		(last-wp-get ?r - robot)

		(next-wp-put ?r - robot)
		(last-wp-put ?r - robot)


		(next-wp-put-slide ?r - robot)
		(last-wp-put-slide ?r - robot)

		(next-fulfill-order-c0)
		(last-fulfill-order-c0)

		(next-fulfill-order-c1)
		(last-fulfill-order-c1)

		(next-fulfill-order-c2)
		(last-fulfill-order-c2)

		(next-fulfill-order-c3)
		(last-fulfill-order-c3)

		(next-lock ?name - object)
		(last-lock ?name - object)

		(next-one-time-lock ?name - object)
		(last-one-time-lock ?name - object)

		(next-unlock ?name - object)
		(last-unlock ?name - object)

		(next-eventually-unlock ?name - object)
		(last-eventually-unlock ?name - object)
		
		(next-location-lock ?location - mps ?side - side)
		(last-location-lock ?location - mps ?side - side)

		(next-location-unlock ?location - mps ?side - side)
		(last-location-unlock ?location - mps ?side - side)
	)

;Kind of a hack. actually it should model the removal of present workpieces
	(:action reset-mps
		:parameters (?m - mps)
		:precondition (mps-state ?m BROKEN)
		:effect (mps-state ?m BROKEN)
	)
)

