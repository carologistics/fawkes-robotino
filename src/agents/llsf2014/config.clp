(defrule prod-figure-out-waiting-points
   (phase PRODUCTION)
   (confval (path "/clips-agent/llsf2014/wait-for-ins1-point") (value ?ins1-wait-point))
   (confval (path "/clips-agent/llsf2014/wait-for-ins2-point") (value ?ins2-wait-point))
   (confval (path "/clips-agent/llsf2014/wait-for-deliver1-point") (value ?deliver1-wait-point))
   (confval (path "/clips-agent/llsf2014/wait-for-deliver2-point") (value ?deliver2-wait-point))
   =>
  (assert (wait-point Ins1 ?ins1-wait-point) (wait-point deliver1 ?deliver1-wait-point)
          (wait-point Ins2 ?ins2-wait-point) (wait-point deliver2 ?deliver2-wait-point)
  )
)
