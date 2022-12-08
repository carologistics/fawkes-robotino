
;---------------------------------------------------------------------------
;  exploration.clp - Robotino agent for exploration phase
;
;  Copyright  2017-2018 Victor Mataré, Daniel Habering
;
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defglobal
  ?*EXP-MOVEMENT-COMPENSATION* = 0.0
  ?*EXP-SEARCH-LIMIT* = 1
)

(defrule exp-sync-ground-truth
" When the RefBox sends ground-truth of a zone, update the corresponding
  domain fact. But only do so for ground-truth of the own team, this allows to
  deal with halve-fields, where only one set of MPS is present but ground truth
  for all machines is sent.
"
	(wm-fact (key game found-tag zone args? m ?mps) (value ?zone))
	?zc <- (domain-fact (name zone-content) (param-values ?zone ?o-mps&:(neq ?o-mps ?mps)))
	=>
	(modify ?zc (param-values ?zone ?mps))
)

(defrule exp-startup
" Asserts all needed wm-facts for the exploration phase
  line-vis:      value is greater 0 if there was a line detected in the zone
  tag-vis:       value is greater 0 if there was a tag detected in the zone
  time-searched: Number of times this zone was activly explored
  zone:          Which machine is in this zone. Initially every zone has an UNKNOWN machine.
                 If there is reason to believe, that the zone is empty, machine is NONE
                 If a machine was detected, this contains the name of the machine (eg C-CS1)
"
	(not (wm-fact (key domain fact zone-content args? z ?zn m ?machine)))
	(wm-fact (id "/config/rcll/exploration/zone-margin") (type FLOAT) (value ?zone-margin))
	(confval (path ?min-path&:(eq ?min-path (str-cat ?*NAVGRAPH_GENERATOR_MPS_CONFIG* "bounding-box/p1")))
	         (list-value ?x_min ?y_min))
	(confval (path ?max-path&:(eq ?max-path (str-cat ?*NAVGRAPH_GENERATOR_MPS_CONFIG* "bounding-box/p2")))
	         (list-value ?x_max ?y_max))
=>
	(bind ?zones (create$))
	(loop-for-count (?x ?x_min ?x_max)
		(loop-for-count (?y ?y_min ?y_max)
			(if (and (not (or (= ?x 0) (= ?y 0)))
			         ; not within the insertion area
			         (or (> (abs ?y) 1)
			             (< (abs ?x)
			                (- (max (abs ?x_min) (abs ?x_max)) 3))
			         )
			    )
			 then
				(bind ?team-prefix C)
				(if (< ?x 0) then
					(bind ?team-prefix M)
				)
				(bind ?zones (append$ ?zones (sym-cat ?team-prefix -Z (abs ?x) (abs ?y))))
			)
		)
	)
	(assert (exp-zone-margin ?zone-margin))

	(progn$ (?zone ?zones)
		(assert (domain-fact (name zone-content) (param-values ?zone UNKNOWN))
		)
	)
)

(defrule exp-sync-mirrored-zone
" Every knowledge of a zone can be snyced to its counterpart on the other side of the field, since they are symmetrical
  Only syncs NONE machine, since syncing of a found machine is handled in another rule
"
  (wm-fact (key domain fact zone-content args? z ?zn m NONE))
  ?we <- (domain-fact (name zone-content) (param-values ?zn2&:(eq ?zn2 (mirror-name ?zn)) UNKNOWN))
  =>
  (modify ?we (param-values ?zn2 NONE))
  (printout t "Synced zone: " ?zn2 crlf)
)


(defrule exp-sync-tag-finding
" Sync finding of a tag to the other field size
"
  ?wm <- (wm-fact (key exploration fact tag-vis args? zone ?zn) (value ?tv))
  ?we <- (wm-fact (key exploration fact tag-vis args? zone ?zn2&:(eq ?zn2 (mirror-name ?zn))) (value ?tv2&: (< ?tv2 ?tv)))
  =>
  (modify ?we (value ?tv))
  (printout t "Synced tag-finding: " ?zn2 crlf)
)

(defrule exp-stop-when-all-found
	?exp-active <- (wm-fact (key exploration active) (type BOOL) (value TRUE))
	(not (and (wm-fact (key domain fact mps-type args? m ?target-mps $?))
	          (not (domain-fact (name zone-content)
	                            (param-values ?zz ?target-mps))
	)))
	(wm-fact (key refbox phase) (value PRODUCTION))
	=>
	(delayed-do-for-all-facts ((?exp wm-fact))
		(wm-key-prefix ?exp:key (create$ exploration fact))
		(retract ?exp)
	)
	(modify ?exp-active (value FALSE))
)
