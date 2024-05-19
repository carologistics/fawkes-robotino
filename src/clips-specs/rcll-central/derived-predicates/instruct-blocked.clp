(defrule instruct-blocked-positive
" If a running instruct goal targets a machine, no other instruct goal
   should target the same machine.
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
    (goal (class INSTRUCT-BS-DISPENSE-BASE|
        INSTRUCT-RS-MOUNT-RING|
        INSTRUCT-CS-BUFFER-CAP|
        INSTRUCT-CS-MOUNT-CAP|
        INSTRUCT-DS-DELIVER
      )
      (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
      (params $? target-mps ?mps $?)
   )
   (not (domain-fact (name instruct-blocked) (param-values ?mps)))
 =>
   (assert (domain-fact (name instruct-blocked) (param-values ?mps)))
)

(defrule instruct-blocked-negative
" If no instruct goal is running, release the block.
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
   ?f <- (domain-fact (name instruct-blocked) (param-values ?mps))
    (not (goal (class INSTRUCT-BS-DISPENSE-BASE|
            INSTRUCT-RS-MOUNT-RING|
            INSTRUCT-CS-BUFFER-CAP|
            INSTRUCT-CS-MOUNT-CAP|
            INSTRUCT-DS-DELIVER
     )
     (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
     (params $? target-mps ?mps $?)
   ))
 =>
  (retract ?f)
)
