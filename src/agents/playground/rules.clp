(defrule debug-start
  (init)
  =>
  (printout warn "Hallo, ich bin da" crlf)
  (assert (test-fact))
)

(defrule init-drive
  (start) ;assert this fact in clips-webview
  =>
  (printout warn "Driving" crlf)
  (skill-call motor_move x 1)
  (assert (state driving))
)