## TODO
* production
  * prefill cap/ring station
  * make creation of goals dependent on whether cap is buffered/how many bases are in the rs
  * limit amount of productions to 2 at a time
  * handle down/broken machines
  * consider delivery times
  * consider multiple products for single order
* look into retrying failed goals
* select robots smartly (e.g. distance)
* add more tests (e.g. C0)
* documentation: visualize goal tree


## DONE
* C0: split up delivery from reserve goals
* remove self facts in domain.pddl
* make all interactions with the machines asynchronous (only bs-dispense done by
robot which is unproblematic because the robot needs to wait)
* check if everything initalized
* handle mps: no special case for rs
* mount ring: read current ring colors locally, add param for ring index, (change names of certain params)

## NOTES
* verify: go-wait actions before move actions when machine is not reserved
* why does prepare of machines sometimes fail/not set the postconditions?
