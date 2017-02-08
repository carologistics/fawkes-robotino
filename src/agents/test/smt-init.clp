;load needed interfaces
(defrule use-smt
  "Loads interfaces for usage in Clips. These interfaces can only be used after loading."
  (ff-feature smt)
  =>
  (clips_smt_request "test-request")
  (clips_smt_done "test-done")
  ;(clips_smt_abort "test-abort")

  (assert (loaded interfaces))
)
