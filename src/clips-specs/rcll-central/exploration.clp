
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
  ?*MIRRORED-FIELD* = FALSE
)

(deffunction exp-assert-move
	(?zone)
	(bind ?goal (assert (goal (class EXPLORATION-MOVE)
	        (id (sym-cat EXPLORATION-MOVE- (gensym*)))
	        (sub-type SIMPLE)
	        (priority 1.0)
	        (meta-template goal-meta)
	        (verbosity NOISY) (is-executable FALSE)
	        (params zone ?zone)
	        )))
	(return ?goal)
)


; ---------------- EXPLORATION SYNC ----------------

(defrule exp-sync-zone-content
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
		(assert
		        (domain-fact (name zone-content) (param-values ?zone UNKNOWN))
		)
	)
)

(defrule exp-sync-ground-truth
" When the RefBox sends ground-truth of a zone, update the corresponding
  domain fact. But only do so for ground-truth of the own team, this allows to
  deal with half-fields, where only one set of MPS is present but ground truth
  for all machines is sent.
"
	(wm-fact (key game found-tag zone args? m ?mps) (value ?zone))
	?zc <- (domain-fact (name zone-content) (param-values ?zone ?o-mps&:(neq ?o-mps ?mps)))
	=>
	(modify ?zc (param-values ?zone ?mps))
)

(defrule exp-sync-exclude-zones
" Mark a zone as empty if there can not be a machine according to the rules
"
	(exploration-result (zone ?zn) (machine ?machine) (orientation ?orientation) (team ~UNKNOWN))
	?wm <- (domain-fact (name zone-content) (param-values ?zn2 UNKNOWN))
	(test (eq TRUE (zone-is-blocked ?zn ?orientation ?zn2 ?machine)))
	=>
	(modify ?wm (param-values ?zn2 NONE))
	(printout t "There is a machine in " ?zn " with orientation " ?orientation  " so block " ?zn2 crlf)
)

; ---------------- EXPLORATION SETUP ----------------

(defrule exp-setup-enable
" Exploration is needed as we received an mps-state already without knowing
  the zone."
	(or (and (wm-fact (key domain fact mps-state args? m ?name $?))
		     (not (wm-fact (key domain fact zone-content args? z ? m ?name))))
	    (wm-fact (key refbox phase) (value EXPLORATION)))
	(not (wm-fact (key exploration active) (type BOOL)))
	; the game is running
	(wm-fact (key refbox phase) (value EXPLORATION|PRODUCTION))
	(wm-fact (key game state) (value RUNNING))
	; the agent is ready
	(domain-facts-loaded)

	; all robots entered the field
	(not
	    (and
	        (domain-object (name ?robot) (type robot))
	        (not (wm-fact (key domain fact entered-field args? r ?robot)))
	    )
	)
	=>
	(assert (wm-fact (key exploration active) (type BOOL) (value TRUE)))
)

(defrule exp-setup-conf-get-vmax
	"Reads maximum values for rotating and velocity from the config and stores it as a fact."
	(wm-fact (id "/config/rcll/exploration/low-velocity") (type FLOAT) (value ?low-velocity))
	(wm-fact (id "/config/rcll/exploration/low-rotation") (type FLOAT) (value ?low-rotation))
	(wm-fact (id "/config/rcll/exploration/max-velocity") (type FLOAT) (value ?max-velocity))
	(wm-fact (id "/config/rcll/exploration/max-rotation") (type FLOAT) (value ?max-rotation))
	(wm-fact (key central agent robot args? r ?robot))
	=>
	(assert (exp-navigator-vmax ?robot ?max-velocity ?max-rotation))
	(assert (exp-navigator-vlow ?robot ?low-velocity ?low-rotation))
)

(defrule exp-setup-enable-laser-lines
	"Enable the camera for machine detection at the beginning of exploration."
	(wm-fact (key exploration active) (type BOOL) (value TRUE))
	(wm-fact (key central agent robot args? r ?robot))
	=>
	(laser-lines-enable ?robot)
)

(defrule exp-setup-enable-camera
	"Enable the camera for machine detection at the beginning of exploration."
	(wm-fact (key exploration active) (type BOOL) (value TRUE))
	(wm-fact (key central agent robot args? r ?robot))
	=>
	(exploration-camera-enable ?robot)
)

(defrule exp-setup-create-exploration-timer
	"Start the timer for reporting detected machines."
	(wm-fact (key refbox phase) (value EXPLORATION|PRODUCTION))
	(wm-fact (key game state) (value RUNNING))
	=>
	(assert (timer (name send-machine-reports)))
)

(defrule exp-setup-create-targets
	"Create exploration targets, a list of zones that is targeted in order."
	(not (wm-fact (key exploration targets $?)))
	(wm-fact (key exploration active) (value TRUE))
	=>
	(bind ?zones (create$ M-Z12  M-Z52 M-Z56 M-Z16 M-Z32 M-Z36 M-Z14 M-Z54))
	(assert (wm-fact (key exploration targets args?)
	                 (is-list TRUE)
	                 (values (randomize$ ?zones)))
	)
)

(defrule exp-setup-assert-root
	"Create the exploration root where all exploration-move goals are under."
	(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
	; exploration is active but there is no root yet
	(not (goal (class EXPLORATION-ROOT)))
	(wm-fact (key exploration active) (value TRUE))
	=>
	(bind ?g (goal-tree-assert-central-run-parallel EXPLORATION-ROOT))
	(modify ?g (meta do-not-finish) (priority 0.0))
)

; ---------------- EXPLORATION GOALS ----------------

(defrule exp-goals-create-move-goal
  	"Create a move goal based on an exploration target."
	(goal (id ?root-id) (class EXPLORATION-ROOT) (mode FORMULATED|DISPATCHED))
	?exp-targ <- (wm-fact (key exploration targets args?) (values ?location $?locations))
	(wm-fact (key exploration active) (type BOOL) (value TRUE))
	=>
	(bind ?goal
	      (exp-assert-move ?location)
	)
	(modify ?goal (parent ?root-id))
	(modify ?exp-targ (values ?locations))
)

(defrule exp-goals-move-executable
	"Move to a navgraph node."
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class EXPLORATION-MOVE)
	                          (mode FORMULATED)
	                          (params zone ?target)
	                          (is-executable FALSE))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	=>
	(printout t "Goal EXPLORATION-MOVE executable for " ?robot crlf)
	(modify ?g (is-executable TRUE))
)

(defrule exp-goals-cleanup-failed-completed
	"Remove failed or completed exploration goals."
	?g <- (goal (class EXPLORATION-MOVE) (mode RETRACTED) (outcome FAILED|COMPLETED))
	=>
	(retract ?g)
)

; ---------------- EXPLORATION REPORT ----------------

(deffunction zone-string-to-sym (?zone-string)
	(return (sym-cat (sub-string 1 1 ?zone-string) "_Z" (sub-string 3 4 ?zone-string)))
)

(defrule exp-report-map-interface-to-exploration-result
	"Take the information from the exploration interface and map it to a exploration-result fact."
	(BoxInterface
		(box_type ?mps-string)
		(orientation ?orientation)
		(zone ?zone-string)
	)

	(not (exploration-result (zone ?zone&:(eq ?zone (zone-string-to-sym ?zone-string)))))
	=>
	(assert (exploration-result (team UNKNOWN) (machine (sym-cat ?mps-string)) (zone (zone-string-to-sym ?zone-string)) (orientation ?orientation)))
)

(defrule exp-report-send-partial
	"Send exploration result if team unknown"
	(wm-fact (key refbox phase) (value EXPLORATION|PRODUCTION))
	(wm-fact (key refbox team-color) (value ?team))
	?er <- (exploration-result (team UNKNOWN) (machine ?machine) (zone ?zone) (orientation ?orientation) (status REPORTED|UNREPORTED))

	(wm-fact (key refbox comm peer-id private) (value ?peer))

	(time $?now)
	?ws <- (timer (name send-machine-reports) (time $?t&:(timeout ?now ?t 1)) (seq ?seq))
	=>
	(modify ?ws (time ?now) (seq (+ ?seq 1)))
	(bind ?mre (pb-create "llsf_msgs.MachineReportEntry"))
	(pb-set-field ?mre "type" (str-cat ?machine))
	(pb-set-field ?mre "zone" (protobuf-name ?zone))
	(pb-set-field ?mre "rotation" ?orientation)
	(bind ?mr (pb-create "llsf_msgs.MachineReport"))
	(pb-set-field ?mr "team_color" ?team)
	(pb-add-list ?mr "machines" ?mre)
	(pb-broadcast ?peer ?mr)
	(modify ?er (status REPORTED))
)

(defrule exp-report-send-full
	"Send exploration result if we got feedback from the refbox about the specific machine name"
	(wm-fact (key refbox phase) (value EXPLORATION|PRODUCTION))
	(wm-fact (key refbox team-color) (value ?team))
	?er <- (exploration-result (team ?team) (machine ?machine) (zone ?zone) (orientation ?orientation) (status PARTIAL_CORRECT))

	(wm-fact (key refbox comm peer-id private) (value ?peer))
	=>
	(bind ?mre (pb-create "llsf_msgs.MachineReportEntry"))
	(pb-set-field ?mre "name" (str-cat ?machine))
	(pb-set-field ?mre "zone" (protobuf-name ?zone))
	(pb-set-field ?mre "rotation" ?orientation)
	(bind ?mr (pb-create "llsf_msgs.MachineReport"))
	(pb-set-field ?mr "team_color" ?team)
	(pb-add-list ?mr "machines" ?mre)
	(pb-broadcast ?peer ?mr)
)

(defrule refbox-recv-MachineReportInfo
  ?pb-msg <- (protobuf-msg (type "llsf_msgs.MachineReportInfo") (ptr ?p))
  =>
  (bind ?machines (create$))

  (foreach ?m (pb-field-list ?p "reported_types")
	(do-for-fact ((?er exploration-result))
		(and
				(eq ?er:zone (sym-cat (pb-field-value ?m "zone")))
				(eq ?er:machine (sym-cat (pb-field-value ?m "type")))
		(eq ?er:status REPORTED)
			)
		(bind ?m-name (sym-cat (pb-field-value ?m "name")))
		(bind ?m-color (sym-cat (pb-field-value ?m "team_color")))
		(bind ?m-zone (sym-cat (pb-field-value ?m "zone")))
		(bind ?m-type (sym-cat (pb-field-value ?m "type")))

		(if (not (any-factp ((?er2 exploration-result)) (eq ?er2:zone (mirror-name ?m-zone)))) then
			(if ?*MIRRORED-FIELD* then
				(assert
					(exploration-result
						(machine (mirror-name ?m-name))
						(zone (mirror-name ?m-zone))
						(orientation (mirror-orientation ?m-type ?m-zone ?er:orientation))
						(team (mirror-team ?m-color))
						(status REPORTED)
					)
				)
			else
				(assert
					(exploration-result
						(machine (mirror-name ?m-name))
						(zone  ?m-zone)
						(orientation ?er:orientation)
						(team (mirror-team ?m-color))
						(status REPORTED)
					)
				)
			)
		)
		(modify ?er (status PARTIAL_CORRECT) (machine ?m-name) (team ?m-color))
  	)
  )
)

; ---------------- EXPLORATION STOP ----------------

(defrule exp-stop-when-all-found
	"Stop the exploration if all machines from our team have been found."
	?exp-active <- (wm-fact (key exploration active) (type BOOL) (value TRUE))
	; there is no machine of our team for which we don't know the location
	(wm-fact (key refbox team-color) (value ?color))
	(not (and (wm-fact (key domain fact mps-team args? m ?target-mps col ?color))
	          (not (domain-fact (name zone-content)
	                            (param-values ?zz ?target-mps))
	)))
	=>
	(delayed-do-for-all-facts ((?exp wm-fact))
		(wm-key-prefix ?exp:key (create$ exploration fact))
		(retract ?exp)
	)
	(modify ?exp-active (value FALSE))
)

(defrule exp-stop-fail-goals
	"Exploration is not needed anymore as all machines were found."
	(goal (id ?goal-id) (class EXPLORATION-MOVE) (mode DISPATCHED))
	(wm-fact (key exploration active) (type BOOL) (value FALSE))
	=>
	(assert (wm-fact (key monitoring fail-goal args? g ?goal-id)))
)

(defrule exp-stop-disable-goals
	"Exploration is not needed anymore, as all machines were found.
  	 Remove assignments and retract goal/goal-meta facts."
	(wm-fact (key exploration active) (type BOOL) (value FALSE))
	?gf <-(goal (id ?id) (class EXPLORATION-MOVE) (mode FORMULATED))
  	?gm <- (goal-meta (goal-id ?id) (assigned-to nil))
	=>
	(remove-robot-assignment-from-goal-meta ?gf)
	(retract ?gf ?gm)
)

(defrule exp-stop-disable-camera
	"Disable the camera once the exploration is over."
	(wm-fact (key exploration active) (type BOOL) (value FALSE))
	(wm-fact (key central agent robot args? r ?robot))
	=>
	(exploration-camera-enable ?robot)
)

(defrule exp-remove-selected-exploration-root
	"Sometimes we may get ground-truth before actually starting the exploration.
	This can lead to the exploration root getting stuck at selected."
	(wm-fact (key exploration active) (type BOOL) (value FALSE))
	?g <- (goal (class EXPLORATION-ROOT) (mode SELECTED))
	=>
	(modify ?g (mode FINISHED) (outcome COMPLETED))
)
