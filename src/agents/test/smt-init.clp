;load needed interfaces
(defrule use-smt
  "Loads interfaces for usage in Clips. These interfaces can only be used after loading."
  (ff-feature smt)
  =>
  ;(clips_smt_request "test-request") ,TODO (Igor) How to set the parameter correctly?
  (printout t "Test return value of clips_smt_done: " (clips_smt_done "test-done")  crlf)

  (assert (loaded interfaces))
)
