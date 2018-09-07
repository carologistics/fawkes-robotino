
;---------------------------------------------------------------------------
;  smtstr-wait-init.clp - wait for smt_strategy to be fully up
;
;  Created: Wed Sep 5 10:51
;  Copyright  2018 Igor Bongartz <bongartz@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defglobal
	?*CX-SMT-SINGLE-C3-FILES* = (create$
                                "rcll2017/goal-reasoner-produce-c3.clp"
																"rcll2017/goal-expander.clp"
                                "rcll2017/goal-expander-call-single-c3.clp")
	?*CX-SMT-MULTI-C3-FILES* = (create$
                                "rcll2017/goal-reasoner-produce-c3.clp"
																"rcll2017/goal-expander.clp"
                                "rcll2017/goal-expander-call-multi-c3.clp"
                                "rcll2017/goal-expander-collect-plan-r2-r3.clp")
	?*CX-SMT-MULTI-C3C0-FILES* = (create$
                                "rcll2017/goal-reasoner-produce-c3.clp"
                                "rcll2017/goal-reasoner-produce-c0.clp"
																"rcll2017/goal-expander.clp"
                                "rcll2017/goal-expander-call-multi-c3.clp"
                                "rcll2017/goal-expander-call-multi-c0.clp"
                                "rcll2017/goal-expander-collect-plan-r2-r3.clp"
                                "rcll2017/action-selection-done-r2-r3.clp")
)

(defrule smtstr-wait-init-single-c3-ok
	"Load files for smt_strategy single_c3"
	(not (executive-init-signal (id smtstr-initialized)))
  (confval (path "/clips-executive/specs/rcll2017/parameters/rcll/smt_strategy") (type STRING) (value ?smtstr))
  (test (eq ?smtstr (str-cat single_c3)))
	=>
	(cx-assert-init-requests ?smtstr STAGE-3 TRUE)
	(assert (executive-init-request (state PENDING)	(stage STAGE-3) (order 0)
																	(name smt_strategy) (feature FALSE) (files ?*CX-SMT-SINGLE-C3-FILES*)))
	(printout t "All smt_strategy files for single_c3 are loaded" crlf)
	(assert (executive-init-signal (id smtstr-initialized)))
)

(defrule smtstr-wait-init-multi-c3-ok
	"Load files for smt_strategy multi_c3"
	(not (executive-init-signal (id smtstr-initialized)))
  (confval (path "/clips-executive/specs/rcll2017/parameters/rcll/smt_strategy") (type STRING) (value ?smtstr))
  (test (eq ?smtstr (str-cat multi_c3)))
	=>
	(cx-assert-init-requests ?smtstr STAGE-3 TRUE)
	(assert (executive-init-request (state PENDING)	(stage STAGE-3) (order 0)
																	(name smt_strategy) (feature FALSE) (files ?*CX-SMT-MULTI-C3-FILES*)))
	(printout t "All smt_strategy files for multi_c3 are loaded" crlf)
	(assert (executive-init-signal (id smtstr-initialized)))
)

(defrule smtstr-wait-init-multi-c3c0-ok
	"Load files for smt_strategy multi_c3c0"
	(not (executive-init-signal (id smtstr-initialized)))
  (confval (path "/clips-executive/specs/rcll2017/parameters/rcll/smt_strategy") (type STRING) (value ?smtstr))
  (test (eq ?smtstr (str-cat multi_c3c0)))
	=>
	(cx-assert-init-requests ?smtstr STAGE-3 TRUE)
	(assert (executive-init-request (state PENDING)	(stage STAGE-3) (order 0)
																	(name smt_strategy) (feature FALSE) (files ?*CX-SMT-MULTI-C3C0-FILES*)))
	(printout t "All smt_strategy files for multi_c3c0 are loaded" crlf)
	(assert (executive-init-signal (id smtstr-initialized)))
)
