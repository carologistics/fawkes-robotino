% Defined programs:
% ourTeam(t)                     Setup the mount ring tasks, that are possible, as soon as we know the special
%                                locations.
% transition(gt)                 Update implications from the mount tasks.
%
% In this file defined predicates:
% duration(mountRing(_), D)      How long it takes to put the product into ring cap station, once the robot has arrived.
% free(L, gt)                    The machine side L is freed in time step GT.
% inUse(L, GT)                   The machine side L is in use for time step GT.
% location(mountRing(_), L)      Where the mounting will take place.
% points(mountRing(_), P)        How many points are awarded for the mounting.
% possible(R, mountRing(_), GT)  If robot R can perform the mount task at GT.
% process(M, P, GT)              The machine M starts the processing of the product P at GT.
% putDown(R, L, P, GT)           Robot R puts product P down at location L in GT.
% ringMounted(M, P, C, GT)       Machine M mounted a ring of color C on product P from in GT.
% task(mountRing(L, O, Q, R))    The task to mount the R-th ring on a product of (O,Q) at L.
%
% Additionally used predicates:
% base(O, B)                     Defined in orders.lp
% base(P, B)                     Defined in products.lp
% begin(R, T, GT)                Defined in state.lp
% cap(P, C, GT)                  Defined in products.lp
% csPrepared(M, GT)              Defined in machines.lp
% deliveryTimes(L, U)            Defined in facts.lp
% end(R, T, GT)                  Defined in state.lp
% failure(R, T, GT)              Defined in state.lp
% holding(R, P, GT)              Defined in robots.lp
% location(R, L, GT)             Defined in state.lp
% order(O)                       Defined in facts.lp
% quantity(Q)                    Defined in facts.lp
% ring(O, R, C)                  Defined in orders.lp
% ring(P, R, C, GT)              Defined in products.lp
% ringColorCost(Col, Cost)       Defined in facts.lp
% ringLocation(L)                Defined in facts.lp
% rsFillState(M, FS, GT)         Defined in machines.lp
% storing(M, P, GT)              Defined in machines.lp


#program ourTeam(t).

% For the points we cheat a little bit. The points are only awarded for the last ring, but this is difficult to model
% so we award for every mount and subtract the before awarded from the points for the next ring. So we don't get 80
% points for the 3rd ring of a C3, but only 50, because we got 10 for the 1st and 20 for the 2nd, in total 80.
ringTask(mountRing(L, O, Q, R))       :- order(O), quantity(Q), ringLocation(_, L), R = 1..3.
task(T)                               :- ringTask(T).
points(mountRing(L, O, Q, 1), 10)     :- task(mountRing(L, O, Q, 1)).
points(mountRing(L, O, Q, 2), 20)     :- task(mountRing(L, O, Q, 2)).
points(mountRing(L, O, Q, 3), 50)     :- task(mountRing(L, O, Q, 3)).
duration(T, @mountRingTaskDuration()) :- ringTask(T).
location(mountRing(L, O, Q, R), L)	  :- task(mountRing(L, O, Q, R)).

#program transition(gt).

#external spawnTask(mountRing(L, O, Q, R), gt) : ringTask(mountRing(L, O, Q, R)), deliveryTimes(B, _), B >= gt, not orderSpawned(O).

inUse(L, gt) :- begin(R, T, gt), ringTask(T), location(T, L).
free(L, gt)  :- end(_, T, gt), ringTask(T), location(T, L).

possible(R, mountRing(L, O, Q, 1), gt) :- ringTask(mountRing(L, O, Q, 1)), holding(R, P, gt), ringLocation(M, L),
										  rsFillState(M, FS, gt), not storing(M, _, gt), not inUse(L, gt),
										  base(P, B), base(O, B), ring(P, 1, none, gt), cap(P, none, gt),
										  ringColorCost(Color, Cost), Cost <= FS, robot(R, gt).

possible(R, mountRing(L, O, Q, 2), gt) :- ringTask(mountRing(L, O, Q, 1)), holding(R, P, gt), ringLocation(M, L),
										  rsFillState(M, FS, gt), not storing(M, _, gt), not inUse(L, gt),
										  base(P, B), base(O, B), not ring(P, 1, none, gt), ring(P, 2, none, gt),
										  cap(P, none, gt), ringColorCost(Color, Cost), Cost <= FS, robot(R, gt).

possible(R, mountRing(L, O, Q, 3), gt) :- ringTask(mountRing(L, O, Q, 1)), holding(R, P, gt), ringLocation(M, L),
										  rsFillState(M, FS, gt), not storing(M, _, gt), not inUse(L, gt),
										  base(P, B), base(O, B), not ring(P, 2, none, gt), ring(P, 3, none, gt),
										  cap(P, none, gt), ringColorCost(Color, Cost), Cost <= FS, robot(R, gt).

%Todo: mache aus possible() mehrere impossible und dann possible :- not impossible
%Todo: remove the fake choice rules
{ putDown(R, L, P, gt);
  ringMounted(M, P, Color, gt);
  produced(M, gt, P)
  } = 3 :- end(R, mountRing(L, O, Q, Ring), gt), not failure(mountRing(L, O, Q, Ring), gt), ringLocation(M, L),
		   holding(R, P, gt), ring(O, Ring, Color).

