;---------------------------------------------------------------------------
;  Copyright  2016-2017  Bjoern Schaepers
;  Licensed under GPLv2+ license
;---------------------------------------------------------------------------

(defrule asp-remove-exp-tactic
  "Remove every exp-tactic fact, so the exploration reasoning doesn't start."
  (declare (salience 10000))
  ?f <- (exp-tactic ?)
  =>
  (retract ?f)
)
