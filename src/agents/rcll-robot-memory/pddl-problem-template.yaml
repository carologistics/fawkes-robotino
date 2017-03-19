(define (problem rcll-prod-c0)
	(:domain RCLL2016)

	(:objects
		R-1 - robot
		R-2 - robot
    o0 - order
		o1 - order
		o2 - order
		o3 - order
		red-base1 - base
		red-base2 - base
		red-base3 - base
		red-base4 - base
		red-base5 - base
		red-base6 - base
    black-base1 - base
    black-base2 - base
		black-base3 - base
		black-base4 - base
		black-base5 - base
		black-base6 - base
    silver-base1 - base
    silver-base2 - base
		silver-base3 - base
		silver-base4 - base
		silver-base5 - base
		silver-base6 - base

		cc1 cc2 cc3 - cap-carrier
    slot1 slot2 slot3 - shelf-slot
		cc4 cc5 cc6 - cap-carrier
    slot4 slot5 slot6 - shelf-slot
		cc7 cc8 cc9 cc10 cc11 cc12 - cap-carrier
    
    <<#MACHINES|{relation:'machine', team:'CYAN'}>><<name>> - <<mtype>> <</MACHINES>>

    <<#MACHINEPOS|{relation:'machine', team:'CYAN'}>><<name>>-I <<name>>-O <</MACHINEPOS>>
    INS - pos
	)
	 
	(:init
    <<#MACHINEHASPOS|{relation:'machine', team:'CYAN'}>>(has-pos <<name>> <<name>>-I) (pos-free <<name>>-I) (has-pos <<name>> <<name>>-O) (pos-free <<name>>-O) <</MACHINEHASPOS>>

    <<#MACHINEIDLE|{relation:'machine', team:'CYAN'}>>(idle <<name>>) <</MACHINEIDLE>>

    ;TODO FILL RS COLORS FROM MONGODB
    (has-color-one RS1 RING_GREEN)
    (needs-bases RING_GREEN ONE)
    (has-color-two RS1 RING_YELLOW)
    (needs-bases RING_YELLOW TWO)
    (has-color-one RS2 RING_ORANGE)
    (needs-bases RING_ORANGE ONE)
    (has-color-two RS2 RING_BLUE)
    (needs-bases RING_BLUE ZERO)

    (has-shelf-slot CS1 slot1)
    (has-shelf-slot CS1 slot2)
    (has-shelf-slot CS1 slot3)
    (has-shelf-slot CS2 slot4)
    (has-shelf-slot CS2 slot5)
    (has-shelf-slot CS2 slot6)
    (has-cap cc1 CAP_GRAY)
    (has-cap cc2 CAP_GRAY)
    (has-cap cc3 CAP_GRAY)
    (has-cap cc4 CAP_BLACK)
    (has-cap cc5 CAP_BLACK)
    (has-cap cc6 CAP_BLACK)
    (cc-in-slot cc1 slot1)
    (cc-in-slot cc2 slot2)
    (cc-in-slot cc3 slot3)
    (cc-in-slot cc4 slot4)
    (cc-in-slot cc5 slot5)
    (cc-in-slot cc6 slot6)

    (no-cap-in-slide CS1)
    (no-cap-in-slide CS2)

    (has-color red-base1 BASE_RED)
    (has-color red-base2 BASE_RED)
    (has-color red-base3 BASE_RED)
    (has-color red-base4 BASE_RED)
    (has-color red-base5 BASE_RED)
    (has-color red-base6 BASE_RED)
    (has-color black-base1 BASE_BLACK)
    (has-color black-base2 BASE_BLACK)
    (has-color black-base3 BASE_BLACK)
    (has-color black-base4 BASE_BLACK)
    (has-color black-base5 BASE_BLACK)
    (has-color black-base6 BASE_BLACK)
    (has-color silver-base1 BASE_SILVER)
    (has-color silver-base2 BASE_SILVER)
    (has-color silver-base3 BASE_SILVER)
    (has-color silver-base4 BASE_SILVER)
    (has-color silver-base5 BASE_SILVER)
    (has-color silver-base6 BASE_SILVER)

    (has-color cc1 BASE_CLEAR)
    (has-color cc2 BASE_CLEAR)
    (has-color cc3 BASE_CLEAR)
    (has-color cc4 BASE_CLEAR)
    (has-color cc5 BASE_CLEAR)
    (has-color cc6 BASE_CLEAR)

    (wp-at-bs red-base1)
    (wp-at-bs red-base2)
    (wp-at-bs red-base3)
    (wp-at-bs red-base4)
    (wp-at-bs red-base5)
    (wp-at-bs red-base6)
    (wp-at-bs black-base1)
    (wp-at-bs black-base2)
    (wp-at-bs black-base3)
    (wp-at-bs black-base4)
    (wp-at-bs black-base5)
    (wp-at-bs black-base6)
    (wp-at-bs silver-base1)
    (wp-at-bs silver-base2)
    (wp-at-bs silver-base3)
    (wp-at-bs silver-base4)
    (wp-at-bs silver-base5)
    (wp-at-bs silver-base6)

    (order-complexity o0 C0)
    (order-base-color o0 BASE_RED)
    (order-cap-color o0 CAP_GRAY)
    (order-ring-one o0 RING_NONE)
    (order-ring-two o0 RING_NONE)
    (order-ring-three o0 RING_NONE)

    (order-complexity o1 C1)
    (order-base-color o1 BASE_SILVER)
    (order-cap-color o1 CAP_BLACK)
    (order-ring-one o1 RING_GREEN)
    (order-ring-two o1 RING_NONE)
    (order-ring-three o1 RING_NONE)

    (order-complexity o2 C2)
    (order-base-color o2 BASE_BLACK)
    (order-cap-color o2 CAP_BLACK)
    (order-ring-one o2 RING_ORANGE)
    (order-ring-two o2 RING_YELLOW)
    (order-ring-three o2 RING_NONE)

    (order-complexity o3 C3)
    (order-base-color o3 BASE_SILVER)
    (order-cap-color o3 CAP_GRAY)
    (order-ring-one o3 RING_GREEN)
    (order-ring-two o3 RING_BLUE)
    (order-ring-three o3 RING_YELLOW)

    (at-pos R-1 INS)
    (not-holding R-1)
    ;(at-pos R-2 INS)
    ;(not-holding R-2)

  )
  (:goal (and
    ;(wp-in-production silver-base1 RS1)
    ;(wp-in-slide RS1 TWO)
    ;(wp-in-slide RS1 ONE)
    (order-fulfilled o0)
    ;(order-fulfilled o1)
    ;(order-fulfilled o2)
    ;(order-fulfilled o3)
    ;(:goal (wp-at cc1 C-CS1 OUTPUT) )
    ;(holding R-1 cc1)
    ;(at-pos R-1 CS1-I)
    ;(one-wp-in-slide RS2)
    )
  )
)
