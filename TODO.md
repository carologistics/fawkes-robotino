## TODO
* production
  * prefill cap/ring station
  * make creation of goals dependent on whether cap is buffered/how many bases are in the rs
  * limit amount of productions to 2 at a time
  * make all interactions with the machines asynchronous
  * handle down/broken machines
  * consider delivery times
* remove self facts in domain.pddl
* look into retrying failed goals
* select robots smartly (e.g. distance)
* add more tests (e.g. C0)
* documentation: visualize goal tree

## DONE
* C0: split up delivery from reserve goals

## NOTES
* verify: go-wait actions before move actions when machine is not reserved
* why does prepare of machines sometimes fail/not set the postconditions?
