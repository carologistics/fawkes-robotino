
(defrule asp-remove-exp-tactic
  "Remove every exp-tactic fact, so the exploration reasoning doesn't start."
  (declare (salience 10000))
  ?f <- (exp-tactic ?)
  =>
  (retract ?f)
)

(defrule asp-remove-init-locking
  "Remove every init-locking fact, we don't want to use the locking mechanism."
  (declare (salience 10000))
  ?f <- (init-locking)
  =>
  (retract ?f)
)

