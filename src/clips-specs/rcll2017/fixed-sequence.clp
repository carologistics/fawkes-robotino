
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
  (wm-fact (key domain fact robot-waiting args? r ?robot))
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
    (do-for-fact ((?fact-wp-on-shelf wm-fact))
            (and (wm-key-prefix ?fact-wp-on-shelf:key (create$ domain fact wp-on-shelf))
                 (eq (wm-key-arg ?fact-wp-on-shelf:key m) ?mps))

      (bind ?cc (wm-key-arg ?fact-wp-on-shelf:key wp))
      (bind ?shelf-spot (wm-key-arg ?fact-wp-on-shelf:key spot))

      (do-for-fact ((?fact-wp-cap-color wm-fact))
              (and (wm-key-prefix ?fact-wp-cap-color:key (create$ domain fact wp-cap-color))
                    (eq (wm-key-arg ?fact-wp-cap-color:key wp) ?cc))
        (bind ?cap-color (wm-key-arg ?fact-wp-cap-color:key col))
      )
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


(defrule goal-expander-discard-unneeded-base
 ?g <- (goal (mode SELECTED) (id DISCARD-UNKNOWN) (params robot ?robot wp ?wp))
  =>
  (assert
    (plan (id DISCARD-UNKNOWN-PLAN) (goal-id DISCARD-UNKNOWN))
    (plan-action (id 1) (plan-id DISCARD-UNKNOWN-PLAN) (duration 4.0)
          (action-name wp-discard)
          (param-names r cc )
          (param-values ?robot ?wp))
  )
  (modify ?g (mode EXPANDED))
)


(defrule goal-expander-fill-rs
 ?g <- (goal (mode SELECTED) (id FILL-RS) (params robot ?robot
                                                    mps ?mps
                                                    wp ?wp
                                                    rs-before ?rs-before
                                                    rs-after ?rs-after))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  =>
  (assert
    (plan (id FILL-RS-PLAN) (goal-id FILL-RS))
     (plan-action (id 1) (plan-id FILL-RS-PLAN) (duration 4.0)
                                    (action-name move)
                                    (param-names r from from-side to to-side )
                                    (param-values ?robot ?curr-location ?curr-side ?mps INPUT))
    (plan-action (id 2) (plan-id FILL-RS-PLAN) (duration 4.0)
          (action-name wp-put-slide-cc)
          (param-names r wp m rs-before rs-after)
          (param-values ?robot ?wp ?mps ?rs-before ?rs-after))
  )
  (modify ?g (mode EXPANDED))
)


(deffunction random-id ()
  "Return a random task id"
  (return (random 0 1000000000))
)

(deffunction spawn-wp ()
  "Spawn a new WP by Creating relevante domain facts"
  (bind ?wp-id (sym-cat WP (random-id)))
  (assert
    (domain-object (name ?wp-id) (type workpiece))
    (domain-fact (name wp-unused) (param-values ?wp-id))
    (domain-fact (name wp-base-color) (param-values ?wp-id BASE_NONE))
    (domain-fact (name wp-cap-color) (param-values ?wp-id CAP_NONE))
    (domain-fact (name wp-ring1-color) (param-values ?wp-id RING_NONE))
    (domain-fact (name wp-ring2-color) (param-values ?wp-id RING_NONE))
    (domain-fact (name wp-ring3-color) (param-values ?wp-id RING_NONE))
  )
  (return ?wp-id)
)


(defrule goal-produce-c0
 ?g <- (goal (mode SELECTED) (id PRODUCE-C0) (params robot ?robot
                                                      bs ?bs
                                                      bs-side ?bs-side
                                                      bs-color ?base-color
                                                      mps ?mps
                                                      cs-color ?cap-color
                                                      order ?order
                                                      ))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
 =>
 (bind ?wp (spawn-wp))
 (assert
  (plan (id PRODUCE-C0-PLAN) (goal-id PRODUCE-C0))
  (plan-action (id 1) (plan-id PRODUCE-C0-PLAN) (duration 4.0)
        (action-name move)
        (param-names r from from-side to to-side )
        (param-values ?robot ?curr-location ?curr-side ?bs ?bs-side))
  (plan-action (id 2) (plan-id PRODUCE-C0-PLAN) (duration 4.0)
        (action-name prepare-bs)
        (param-names m side bc)
        (param-values ?bs ?bs-side ?base-color))
  (plan-action (id 3) (plan-id PRODUCE-C0-PLAN) (duration 4.0)
        (action-name bs-dispense)
        (param-names r m side wp basecol)
        (param-values ?robot ?bs ?bs-side ?wp ?base-color))
  (plan-action (id 4) (plan-id PRODUCE-C0-PLAN) (duration 4.0)
        (action-name wp-get)
        (param-names r wp m side)
        (param-values ?robot ?wp ?bs ?bs-side))
  (plan-action (id 5) (plan-id PRODUCE-C0-PLAN) (duration 4.0)
        (action-name move-wp-put)
        (param-names r from from-side to)
        (param-values ?robot ?bs ?bs-side ?mps))
  (plan-action (id 6) (plan-id PRODUCE-C0-PLAN) (duration 4.0)
        (action-name prepare-cs)
        (param-names m op)
        (param-values ?mps MOUNT_CAP))
  (plan-action (id 7) (plan-id PRODUCE-C0-PLAN) (duration 4.0)
        (action-name wp-put)
        (param-names r wp m)
        (param-values ?robot ?wp ?mps))
   (plan-action (id 8) (plan-id PRODUCE-C0-PLAN) (duration 4.0)
        (action-name cs-mount-cap)
        (param-names m wp capcol)
        (param-values ?mps ?wp ?cap-color ))
 )
 (modify ?g (mode EXPANDED))
)

(defrule goal-deliver
 ?g <- (goal (mode SELECTED) (id DELIVER) (params robot ?robot
                                                        mps ?mps
                                                        order ?order
                                                        wp ?wp
                                                        ds ?ds
                                                        ds-gate ?gate
                                                        base-color ?base-color
                                                        cap-color ?cap-color
                                                        ))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
 =>
 (assert
  (plan (id DELIVER-PLAN) (goal-id DELIVER))
  (plan-action (id 1) (plan-id DELIVER-PLAN) (duration 4.0)
        (action-name move)
        (param-names r from from-side to to-side )
        (param-values ?robot ?curr-location ?curr-side ?mps OUTPUT))
  (plan-action (id 2) (plan-id DELIVER-PLAN) (duration 4.0)
        (action-name wp-get)
        (param-names r wp m side)
        (param-values ?robot ?wp ?mps OUTPUT))
  (plan-action (id 3) (plan-id DELIVER-PLAN) (duration 4.0)
        (action-name move-wp-put)
        (param-names r from from-side to)
        (param-values ?robot ?mps OUTPUT ?ds))
  (plan-action (id 4) (plan-id DELIVER-PLAN) (duration 4.0)
        (action-name prepare-ds)
        (param-names m gate)
        (param-values ?ds ?gate))
  (plan-action (id 5) (plan-id DELIVER-PLAN) (duration 4.0)
        (action-name wp-put)
        (param-names r wp m)
        (param-values ?robot ?wp ?ds))
  (plan-action (id 6) (plan-id DELIVER-PLAN) (duration 4.0)
        (action-name fulfill-order-c0)
        (param-names ord wp m g basecol capcol)
        (param-values ?order ?wp ?ds ?gate ?base-color  ?cap-color))
 )
 (modify ?g (mode EXPANDED))
)
