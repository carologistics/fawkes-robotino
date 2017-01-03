;load needed interfaces
(defrule use-smt
  "Loads interfaces for usage in Clips. These interfaces can only be used after loading."
  (ff-feature smt)
  =>
  (clips_smt_dummy "1993")

  (assert (loaded interfaces))
)
