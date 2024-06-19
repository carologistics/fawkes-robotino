(defglobal
  ?*PAYMENT-GOALS* = (create$ PAY-FOR-RINGS-WITH-BASE PAY-FOR-RINGS-WITH-CAP-CARRIER PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF)
)

(defrule rs-fillable-positive-no-payment-goal
" RS is fillable if there is no running payment goal and the slide is not full.
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (domain-fact (name rs-filled-with) (param-values ?mps ?num&:(neq ?num THREE)))
  (not (domain-fact (name rs-fillable) (param-values ?mps)))
  (not (goal (class ?c&:(member$ ?c ?*PAYMENT-GOALS*))
    (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
    (params $? target-mps ?mps $?)))
 =>
  (assert (domain-fact (name rs-fillable) (param-values ?mps)))
)

(defrule rs-fillable-positive-one-payment-goal
" RS is fillable if there is exactly one running payment goal and the slide
  can take at least two more bases.
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (domain-fact (name rs-filled-with) (param-values ?mps ?num&:(< (sym-to-int ?num) 2)))
  (not (domain-fact (name rs-fillable) (param-values ?mps)))
  (goal (id ?pay1) (class ?c&:(member$ ?c ?*PAYMENT-GOALS*))
    (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
    (params $? target-mps ?mps $?))
  (not (goal (id ~?pay1) (class ?c&:(member$ ?c ?*PAYMENT-GOALS*))
    (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
    (params $? target-mps ?mps $?)))
 =>
  (assert (domain-fact (name rs-fillable) (param-values ?mps)))
)

(defrule rs-fillable-negative-slide-full
" RS is not fillable if the slide is full.
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?f <- (domain-fact (name rs-fillable) (param-values ?mps))
  (domain-fact (name rs-filled-with) (param-values ?mps THREE))
 =>
  (retract ?f)
)

(defrule rs-fillable-negative-one-payment-slide-almost-full
" RS is not fillable if the slide is almost full and there is a running payment
  goal.
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?f <- (domain-fact (name rs-fillable) (param-values ?mps))
  (domain-fact (name rs-filled-with) (param-values ?mps TWO|THREE))
  (goal (class ?c&:(member$ ?c ?*PAYMENT-GOALS*))
    (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
    (params $? target-mps ?mps $?))
 =>
  (retract ?f)
)

(defrule rs-fillable-negative-two-payments-running
" We not allow more than two payments at once, hence RS is not fillable,
  if there are already two running payment goals.
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?f <- (domain-fact (name rs-fillable) (param-values ?mps))
  (domain-fact (name rs-filled-with) (param-values ?mps ?))
  (goal (id ?id) (class ?c&:(member$ ?c ?*PAYMENT-GOALS*))
    (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
    (params $? target-mps ?mps $?))
  (goal (id ~?id) (class ?c2&:(member$ ?c2 ?*PAYMENT-GOALS*))
    (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
    (params $? target-mps ?mps $?))
 =>
  (retract ?f)
)
