
(defrule goal-expander-create-sequence
 ?g <- (goal (mode SELECTED) (id TESTGOAL))
 =>
 (assert
  (plan (id TESTGOAL-PLAN) (goal-id TESTGOAL))
  (plan-action (id 1) (plan-id TESTGOAL-PLAN) (duration 4.0)
        (action-name visit)
        (param-names to) (param-values "C-BS-O"))
  )
 (modify ?g (mode EXPANDED))
)

(defrule goal-expander-send-beacon-signal
  ?g <- (goal (mode SELECTED) (id BEACONACHIEVE))
=>
  (assert
    (plan (id BEACONPLAN) (goal-id BEACONACHIEVE))
    (plan-action (id 1) (plan-id BEACONPLAN) (duration 0.0)
      (action-name send-beacon)))
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-wp-spawn
  ?g <- (goal (mode SELECTED) (id WPSPAWN-ACHIEVE))
=>
  (assert
    (plan (id SPAWNPLAN) (goal-id WPSPAWN-ACHIEVE))
    (plan-action (id 1) (plan-id SPAWNPLAN) (duration 0.0)
      (action-name noop)))
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-enter-field
  ?g <- (goal (mode SELECTED) (id ENTER-FIELD))
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact robot-waiting args? r ?robot) (value TRUE))
=>
  (assert
    (plan (id ENTER-FIELD-PLAN) (goal-id ENTER-FIELD))
    (plan-action (id 1) (plan-id ENTER-FIELD-PLAN) (duration 0.0)
                                 (action-name enter-field)
                                 (param-names r team-color)
                                 (param-values ?robot ?team-color))
    )
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-prefill-cap-station
   "Feed a CS with a cap from its shelf so that afterwards it can directly put the cap on a product."
    ?g <- (goal (mode SELECTED) (id FILL-CAP) (params robot ?robot mps ?mps))
    (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
    ;Reasoning about the wm is in Goal Creation
    =>
    (do-for-fact ((?fact-wp-on-shelf wm-fact) (?fact-wp-cap-color wm-fact))
            (and (wm-key-prefix ?fact-wp-cap-color:key (create$ domain fact wp-cap-color))
                 (wm-key-prefix ?fact-wp-on-shelf:key (create$ domain fact wp-on-shelf))
                 (eq (wm-key-arg ?fact-wp-on-shelf:key m) ?mps))
                 (eq (wm-key-arg ?fact-wp-on-shelf:key wp)
                     (wm-key-arg ?fact-wp-cap-color:key wp))

        (bind ?cc (wm-key-arg ?fact-wp-on-shelf:key wp))
        (bind ?shelf-spot (wm-key-arg ?fact-wp-on-shelf:key spot))
        (bind ?cap-color (wm-key-arg ?fact-wp-cap-color:key col))
    )

    (assert
        (plan (id FILL-CAP-PLAN) (goal-id FILL-CAP))
        (plan-action (id 1) (plan-id FILL-CAP-PLAN) (duration 4.0)
                                    (action-name move)
                                    (param-names r from from-side to to-side )
                                    (param-values ?robot ?curr-location ?curr-side ?mps INPUT))
        (plan-action (id 2) (plan-id FILL-CAP-PLAN) (duration 4.0)
                                    (action-name wp-get-shelf)
                                    (param-names r cc m spot)
                                    (param-values ?robot ?cc ?mps ?shelf-spot))
        (plan-action (id 3) (plan-id FILL-CAP-PLAN) (duration 4.0)
                                    (action-name prepare-cs)
                                    (param-names m op)
                                    (param-values ?mps RETRIEVE_CAP))
        (plan-action (id 4) (plan-id FILL-CAP-PLAN) (duration 4.0)
                                    (action-name wp-put)
                                    (param-names r wp m)
                                    (param-values ?robot ?cc ?mps))
        (plan-action (id 5) (plan-id FILL-CAP-PLAN) (duration 4.0)
                                    (action-name cs-retrieve-cap)
                                    (param-names m cc capcol)
                                    (param-values ?mps ?cc ?cap-color))

    )
    (modify ?g (mode EXPANDED))
)

(defrule goal-remove-empty-base-from-cs
 ?g <- (goal (mode SELECTED) (id CLEAR-CS) (params robot ?robot mps ?mps wp ?wp))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
;Reasoning about the wm is in Goal Creation
 =>
 (assert
  (plan (id CLEAR-CS-PLAN) (goal-id CLEAR-CS))
  (plan-action (id 1) (plan-id CLEAR-CS-PLAN) (duration 4.0)
        (action-name move-wp-get)
        (param-names r from from-side to to-side )
        (param-values ?robot ?curr-location ?curr-side ?mps OUTPUT))
  (plan-action (id 2) (plan-id CLEAR-CS-PLAN) (duration 4.0)
        (action-name wp-get)
        (param-names r wp m side)
        (param-values ?robot ?wp ?mps OUTPUT))

 )
 (modify ?g (mode EXPANDED))
)

(defrule goal-produce-c0
 ?g <- (goal (mode SELECTED) (id PRODUCE-C0))
 =>
 (assert
  (plan (id PRODUCE-C0-PLAN) (goal-id PRODUCE-C0))
  (plan-action (id 1) (plan-id PRODUCE-C0-PLAN) (duration 4.0)
        (action-name move-wp-get)
        (param-names r from from-side to to-side )
        (param-values ))
  (plan-action (id 2) (plan-id PRODUCE-C0-PLAN) (duration 4.0)
        (action-name prepare-bs)
        (param-names m side bc)
        (param-values ))
  (plan-action (id 3) (plan-id PRODUCE-C0-PLAN) (duration 4.0)
        (action-name bs-dispense)
        (param-names r m side wp basecol)
        (param-values ))
  (plan-action (id 2) (plan-id PRODUCE-C0-PLAN) (duration 4.0)
        (action-name wp-get)
        (param-names r wp m side)
        (param-values ))
  (plan-action (id 3) (plan-id PRODUCE-C0-PLAN) (duration 4.0)
        (action-name move-wp-put)
        (param-names r from from-side to)
        (param-values ))
  (plan-action (id 4) (plan-id PRODUCE-C0-PLAN) (duration 4.0)
        (action-name prepare-cs)
        (param-names m op)
        (param-values ))
  (plan-action (id 5) (plan-id PRODUCE-C0-PLAN) (duration 4.0)
        (action-name wp-put)
        (param-names r wp m)
        (param-values ))
   (plan-action (id 6) (plan-id PRODUCE-C0-PLAN) (duration 4.0)
        (action-name cs-mount-cap)
        (param-names m wp capcol)
        (param-values ))
 )
 (modify ?g (mode EXPANDED))

)

(defrule goal-deliver
 ?g <- (goal (mode SELECTED) (id PRODUCE-C0))
 =>
 (assert
  (plan (id PRODUCE-C0-PLAN) (goal-id PRODUCE-C0))
  (plan-action (id 1) (plan-id PRODUCE-C0-PLAN) (duration 4.0)
        (action-name move-wp-get)
        (param-names r from from-side to to-side )
        (param-values ))
  (plan-action (id 2) (plan-id PRODUCE-C0-PLAN) (duration 4.0)
        (action-name wp-get)
        (param-names r wp m side)
        (param-values ))
  (plan-action (id 3) (plan-id PRODUCE-C0-PLAN) (duration 4.0)
        (action-name move-wp-put)
        (param-names r from from-side to)
        (param-values ))
  (plan-action (id 4) (plan-id PRODUCE-C0-PLAN) (duration 4.0)
        (action-name prepare-ds)
        (param-names m gate)
        (param-values ))
  (plan-action (id 5) (plan-id PRODUCE-C0-PLAN) (duration 4.0)
        (action-name wp-put)
        (param-names r wp m)
        (param-values ))
  (plan-action (id 6) (plan-id PRODUCE-C0-PLAN) (duration 4.0)
        (action-name fulfill-order-c0)
        (param-names ord wp m g)
        (param-values ))
 )
 (modify ?g (mode EXPANDED))
)
