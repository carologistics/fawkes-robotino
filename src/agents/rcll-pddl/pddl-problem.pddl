(define (problem rcll-prod-c0)
	(:domain RCLL2016)

	(:objects
		R-1 - robot
    O1 - order
    O6 - order
    O7 - order
    
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
    
    C-DS - delivery-station
    C-RS2 - ring-station
    C-RS1 - ring-station
    C-CS2 - cap-station
    C-CS1 - cap-station
    C-BS - base-station
    

    
    C-DS-I - input-pos
    C-DS-O - output-pos
    C-RS2-I - input-pos
    C-RS2-O - output-pos
    C-RS1-I - input-pos
    C-RS1-O - output-pos
    C-CS2-I - input-pos
    C-CS2-O - output-pos
    C-CS1-I - input-pos
    C-CS1-O - output-pos
    C-BS-I - input-pos
    C-BS-O - output-pos
    INS - pos
	)
	 
	(:init
    (has-pos C-DS C-DS-I)
    (pos-free C-DS-I)
    (not-locked C-DS-I)
    (has-pos C-DS C-DS-O)
    (pos-free C-DS-O)
    (not-locked C-DS-O)(has-pos C-RS2 C-RS2-I)
    (pos-free C-RS2-I)
    (not-locked C-RS2-I)
    (has-pos C-RS2 C-RS2-O)
    (pos-free C-RS2-O)
    (not-locked C-RS2-O)(has-pos C-RS1 C-RS1-I)
    (pos-free C-RS1-I)
    (not-locked C-RS1-I)
    (has-pos C-RS1 C-RS1-O)
    (pos-free C-RS1-O)
    (not-locked C-RS1-O)(has-pos C-CS2 C-CS2-I)
    (pos-free C-CS2-I)
    (not-locked C-CS2-I)
    (has-pos C-CS2 C-CS2-O)
    (pos-free C-CS2-O)
    (not-locked C-CS2-O)(has-pos C-CS1 C-CS1-I)
    (pos-free C-CS1-I)
    (not-locked C-CS1-I)
    (has-pos C-CS1 C-CS1-O)
    (pos-free C-CS1-O)
    (not-locked C-CS1-O)(has-pos C-BS C-BS-I)
    (pos-free C-BS-I)
    (not-locked C-BS-I)
    (has-pos C-BS C-BS-O)
    (pos-free C-BS-O)
    (not-locked C-BS-O)
    
    (idle C-DS)
    (idle C-RS2)
    (idle C-RS1)
    (idle C-CS2)
    (idle C-CS1)
    (idle C-BS)

    
    (has-color-one C-RS2 RING_GREEN)
    (has-color-two C-RS2 RING_BLUE)
    
    (has-color-one C-RS1 RING_YELLOW)
    (has-color-two C-RS1 RING_ORANGE)
    

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
    (has-cap cc1 CAP_GREY)
    (has-cap cc2 CAP_GREY)
    (has-cap cc3 CAP_GREY)
    (has-cap cc4 CAP_BLACK)
    (has-cap cc5 CAP_BLACK)
    (has-cap cc6 CAP_BLACK)
    (cc-in-slot cc1 slot1)
    (cc-in-slot cc2 slot2)
    (cc-in-slot cc3 slot3)
    (cc-in-slot cc4 slot4)
    (cc-in-slot cc5 slot5)
    (cc-in-slot cc6 slot6)

    (no-cap-in-slide C-CS1)
    (no-cap-in-slide C-CS2)

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

    
    (order-complexity O1 C0)
    (order-base-color O1 BASE_BLACK)
    (order-cap-color O1 CAP_BLACK)
    (order-ring-one O1 RING_NONE)
    (order-ring-two O1 RING_NONE)
    (order-ring-three O1 RING_NONE)
    
    (order-complexity O6 C2)
    (order-base-color O6 BASE_BLACK)
    (order-cap-color O6 CAP_BLACK)
    (order-ring-one O6 RING_ORANGE)
    (order-ring-two O6 RING_YELLOW)
    (order-ring-three O6 RING_NONE)
    
    (order-complexity O7 C3)
    (order-base-color O7 BASE_RED)
    (order-cap-color O7 CAP_BLACK)
    (order-ring-one O7 RING_YELLOW)
    (order-ring-two O7 RING_ORANGE)
    (order-ring-three O7 RING_GREEN)
    

    (at-pos R-1 INS)
    (not-holding R-1)
    (no-lock R-1)
  )
  (:goal (and
    
    (order-fulfilled O1)
    )
  )
)
