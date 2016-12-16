;---------------------------------------------------------------------------
;  test.clp - Main file for the agent
;
;  Created: Fri Dec 16 14:55:56 2016
;  Copyright  2016  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deffacts init 
  "Initializes clips agent."
  (init)
)
 

; Example for loading a feature
;(defrule enable-tf
;  (agent-init)
;  (ff-feature tf)
;  (not (ff-feature-loaded tf))
;  =>
;  (printout error "Requesting tf feature" crlf)
;  (ff-feature-request "tf")
;  ; Only after the request we can load code depending on its templates
;  ; or functions.
;  (path-load "test/test-tf.clp")
;)

(defrule print-features
  "Print all features as they become available"
  (ff-feature ?f)
  =>
  (printout warn "Feature available: " ?f crlf)
)


(defrule agent-init
  "Execute during agent initialization"
  (agent-init)
  =>
  ; Print to "error" to make it appear red and obvious
  (printout error "OK" crlf)
  ; This is how a call could look like
  ; (smt-func "Foo" "Bar")
)
