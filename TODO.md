## TODO
* clean submission
  * add more tests (e.g. C0)
  * visualize goal tree for documentation

## DONE
* consider multiple products for single order
* consider delivery times
* C0: split up delivery from reserve goals
* remove self facts in domain.pddl
* make all interactions with the machines asynchronous (only bs-dispense done by
robot which is unproblematic because the robot needs to wait)
* check if everything initalized
* handle mps: no special case for rs
* mount ring: read current ring colors locally, add param for ring index, (change names of certain params)
* prefill cap/ring station
* make creation of goals dependent on whether cap is buffered/how many bases are in the rs
* better split of robots at game start (increase preffil capstation prio)
* prioritize certain goals / focus on one construction
* use capcarrier to prefill the ring station
* robots waiting on location-lock for BS
* if a piece is finished and delivery too far in the future, put on storage? (not working)
* retry actions more often?
* handle down/broken machines
* rs-filled-with for parallel goals
* competitive order prio
* look into retrying failed goals (partially done) -> restore consitent states
* remove some unnecessary actions, e.g., go wait at plan end
* Generalize rules for cleaning up work stations (e.g. BS is still handled explicitely), use disposable wps
* no unnecessary prefills after all orders have been posted/a certain time
* increase prioritization of unfinished products still being held by a robot, e.g., next ring mount

## Scratched
* select robots smartly (e.g. distance)
* lower priority/ no robot assignment if needed machine down?
* limit amount of productions to 2 at a time (solved through later start)
* balance prefill of bases in rs (slight advantage to 2 base requiring station)
* get base locked from other side of the map when other robot already waiting -> benefit by early dispense
* dynamically prioritize goals with closer submission time
* retract and clean up a goal if we can no longer complete it in reasonable time

## NOTES
* verify: go-wait actions before move actions when machine is not reserved
* why does prepare of machines sometimes fail/not set the postconditions?
