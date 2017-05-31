(define (problem rcll-prod-c0)
	(:domain RCLL2016)

	(:objects
		R-1 - robot
		R-2 - robot
    <<#ORDER|{relation:'order'}>>
		o<<id>> - order
    <</ORDER>>
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
    <<#MACHINEHASPOS|{relation:'machine', team:'CYAN'}>>(has-pos <<name>> <<name>>-I)
    (pos-free <<name>>-I)
    (has-pos <<name>> <<name>>-O)
    (pos-free <<name>>-O)<</MACHINEHASPOS>>
    <<#MACHINEIDLE|{relation:'machine', team:'CYAN'}>>
    (idle <<name>>)<</MACHINEIDLE>>

    <<#MACHINERINGCOLOR|{relation:'ring-station', name:/C/}>>
    (has-color-one <<name>> RING_<<availablecolors_0>>)
    (has-color-two <<name>> RING_<<availablecolors_1>>)
    <</MACHINERINGCOLOR>>

    ;TODO translate needed-bases
    (needs-bases RING_GREEN ONE)
    (needs-bases RING_YELLOW TWO)
    (needs-bases RING_ORANGE ONE)
    (needs-bases RING_BLUE ZERO)

    (has-shelf-slot C-CS1 slot1)
    (has-shelf-slot C-CS1 slot2)
    (has-shelf-slot C-CS1 slot3)
    (has-shelf-slot C-CS2 slot4)
    (has-shelf-slot C-CS2 slot5)
    (has-shelf-slot C-CS2 slot6)
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

    <<#AGG_ORDER|{relation:'order',aggregated: true}>>
    (order-complexity o<<id>> <<complexity>>)
    (order-base-color o<<id>> BASE_<<base>>)
    (order-cap-color o<<id>> CAP_<<cap>>)
    (order-ring-one o<<id>> RING_<<rings_0>>)
    (order-ring-two o<<id>> RING_<<rings_1>>)
    (order-ring-three o<<id>> RING_<<rings_2>>)
    <</AGG_ORDER>>

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
