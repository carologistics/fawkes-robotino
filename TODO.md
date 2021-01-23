TODO:
prefill cap/ring station
make creation of goals dependent on whether cap is buffered/how many bases are in the rs
(*) look into retrying failed goals
(*) select robots smartly (e.g. distance)
limit amount of productions to 2 at a time
make all interactions with the machines asynchronous
remove self facts in domain.pddl
handle down/broken machines
consider delivery times
C0: split up delivery from reserve goals
(*) add more tests (e.g. C0)

documentation: visualize goal tree

NOTES:
workaround for wp-get: lock + unlock
verify: go-wait actions before move actions when machine is not reserved
