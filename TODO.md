## TODO
* production
  * competitive order prio
  * limit amount of productions to 2 at a time
* stability
  * look into retrying failed goals (partially done)
  * handle down/broken machines
* clean submission
  * add more tests (e.g. C0)
  * visualize goal tree for documentation
* optimization
  * select robots smartly (e.g. distance)
  * retract and clean up a goal if we can no longer complete it in reasonable time
  * no unnecessary prefills after all orders have been posted/a certain time
  * dynamically prioritize goals with closer submission time
  * increase prioritization of unfinished products still being held by a robot, e.g., next ring mount
  * if a piece is finished and delivery too far in the future, put on storage?
  * remove some unnecessary actions, e.g., go wait at plan end
  * lower priority/ no robot assignment if needed machine down?


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


## NOTES
* verify: go-wait actions before move actions when machine is not reserved
* why does prepare of machines sometimes fail/not set the postconditions?
