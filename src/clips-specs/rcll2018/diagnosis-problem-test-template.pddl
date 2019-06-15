(define (problem rcll)
  (:domain rcll-production)
  (:objects M1 - mps
            M2 - mps
            INPUT - mps-side
            R1 - robot
            WP - workpiece

  )
  (:init (at R1 M1 INPUT)
        (location-free M1 INPUT)
        (location-free M2 INPUT)
        (wp-usable WP)
        (locked M1)
        (locked M2)
        (wp-at WP M1 INPUT)
        (can-hold R1)
        (comp-state MOVE-BASE INIT)
        (comp-state GRIPPER CALIBRATED)
        (comp-state NAVGRAPH LOCALIZED)
        (last-BEGIN)
        (= (total-cost) 0)
  )
  (:goal
    <<GOAL>>
  )
  
  (:metric minimize (total-cost))
)
