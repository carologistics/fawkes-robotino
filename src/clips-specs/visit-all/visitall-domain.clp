;---------------------------------------------------------------------------
;  visitall-domain.clp - Simple domain to visit some/all locations
;
;  Created: Thu 26 Oct 2017 21:46:58 CEST
;  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file of cedar
;---------------------------------------------------------------------------

(defglobal
  ?*BBSYNC_PEER_CONFIG* = "/fawkes/bbsync/peers/"
)

(defrule load-domain
  (executive-init)
  (not (domain-facts-loaded))
=>
  (parse-pddl-domain (path-resolve "visit-all/domain.pddl"))
  (assert (domain-fact (name mps-team) (param-values C-BS CYAN))
		  (domain-fact (name mps-team) (param-values C-CS1 CYAN))
  		  (domain-fact (name mps-team) (param-values C-DS CYAN))
			(domain-fact (name mps-team) (param-values C-CS2 CYAN))
			(domain-fact (name mps-team) (param-values C-RS1 CYAN))
			(domain-fact (name mps-team) (param-values C-RS2 CYAN))
			(domain-fact (name mps-team) (param-values C-SS CYAN))
          (domain-fact (name team-color) (param-values CYAN))
          (domain-fact (name mps-type) (param-values C-BS BS))
		  (domain-fact (name mps-type) (param-values C-CS1 CS))
		  (domain-fact (name mps-type) (param-values C-DS DS))
		  (domain-fact (name mps-type) (param-values C-CS2 CS))
		  (domain-fact (name mps-type) (param-values C-RS1 RS))
		  (domain-fact (name mps-type) (param-values C-RS2 RS))
		  (domain-fact (name mps-type) (param-values C-SS SS))
          (domain-object (name C-BS) (type mps))
		  (domain-object (name C-CS1) (type mps))
          (domain-object (name C-DS) (type mps))
		  (domain-object (name C-CS2) (type mps))
		  (domain-object (name C-RS1) (type mps))
		  (domain-object (name C-RS2) (type mps))
		  (domain-object (name C-SS) (type mps))
          (domain-object (name INPUT) (type mps-side))
          (domain-object (name OUTPUT) (type mps-side))
          (domain-object (name CYAN) (type team-color))
          (domain-fact (name mirror-orientation) (param-values 0 180))
          (domain-fact (name mirror-orientation) (param-values 45 135))
          (domain-fact (name mirror-orientation) (param-values 90 90))
          (domain-fact (name mirror-orientation) (param-values 135 45))
          (domain-fact (name mirror-orientation) (param-values 180 0))
          (domain-fact (name mirror-orientation) (param-values 225 315))
          (domain-fact (name mirror-orientation) (param-values 270 270))
          (domain-fact (name mirror-orientation) (param-values 315 225))
  )
  (assert (domain-facts-loaded))
)

(defrule domain-load-active-robots-from-bbsync-peers-config
" Initialize all remote robots using the active bbsync connections."
	(not (confval (path ?p&:(str-prefix ?*BBSYNC_PEER_CONFIG* ?p))))
	; only load the info after initial fact flushing
	(domain-facts-loaded)
	=>
	(config-load ?*BBSYNC_PEER_CONFIG*)
	(delayed-do-for-all-facts ((?cf confval))
		(str-prefix ?*BBSYNC_PEER_CONFIG* ?cf:path)
		(bind ?name (sub-string (str-length ?*BBSYNC_PEER_CONFIG*)
		            (str-length ?cf:path)
		            ?cf:path))
		(bind ?r-active (wm-id-to-key (str-cat ?name)))
		(if (and (= (length$ ?r-active) 2) (eq (nth$ 2 ?r-active) active)
		                                   (eq ?cf:value TRUE))
		  then
		    (printout error ?r-active crlf)
		    (bind ?curr-robot (nth$ 1 ?r-active))
		    (assert (wm-fact (key central agent robot args? r ?curr-robot))
		            (domain-object (name ?curr-robot) (type robot))
		            (domain-fact (name at) (param-values ?curr-robot START INPUT))
		            (domain-fact (name robot-waiting) (param-values ?curr-robot))
		            (domain-fact (name at) (param-values ?curr-robot START INPUT)))
		  else
		    (retract ?cf)
		)
	)
)

(defrule domain-navgraph-compute-wait-positions-finished
" Add the waiting points to the domain once their generation is finished."
	(NavGraphWithMPSGeneratorInterface (id "/navgraph-generator-mps") (final TRUE))
	(or (wm-fact (key config rcll use-static-navgraph) (type BOOL) (value TRUE))
	    (forall
	      (wm-fact (key central agent robot args? r ?robot))
	      (NavGraphWithMPSGeneratorInterface (id ?id&:(eq ?id (remote-if-id ?robot "navgraph-generator-mps"))) (final TRUE))
	    )
	)
	=>
	(printout t "Navgraph generation of waiting-points finished. Getting waitpoints." crlf)
	(do-for-all-facts ((?waitzone navgraph-node)) (str-index "WAIT-" ?waitzone:name)
		(assert
		  ;(domain-object (name (sym-cat ?waitzone:name)) (type waitpoint))
		  (wm-fact (key navgraph waitzone args? name (sym-cat ?waitzone:name))
		           (is-list TRUE) (type INT)
		           (values (nth$ 1 ?waitzone:pos) (nth$ 2 ?waitzone:pos)))
		)
	)
	(assert (wm-fact (key navgraph waitzone generated) (type BOOL) (value TRUE)))
	(delayed-do-for-all-facts ((?wm wm-fact)) (wm-key-prefix ?wm:key (create$ central agent robot))
	  (assert (wm-fact (key central agent robot-waiting args? r (wm-key-arg ?wm:key r))))
	)
)
