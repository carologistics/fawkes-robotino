% Defined programs:
% ourTeam(t)                     Setup the mount cap tasks, that are possible, as soon as we know the special locations.
% transition(gt)                 Update implications from the mount tasks.
%
% In this file defined predicates:
% capMounted(M, P, GT)           Machine M mounted a cap on product P from in GT.
% duration(mountCap(_), D)       How long it takes to put the product into the cap station, once the robot has arrived.
% free(L, gt)                    The machine side L is freed in time step GT.
% inUse(L, GT)                   The machine side L is in use for time step GT.
% location(mountCap(_), L)       Where the mounting will take place.
% points(mountCap(_), P)         How many points are awarded for the mounting.
% possible(R, mountCap(_), GT)   If robot R can perform the mount task at GT.
% process(M, P, GT)              The machine M starts the processing of the product P at GT.
% putDown(R, L, P, GT)           Robot R puts product P down at location L in GT.
% task(mountCap(L, O, Q))        The task to mount a cap on a product of (O,Q) at L.
%
% Additionally used predicates:
% base(O, B)                     Defined in orders.lp
% base(P, B)                     Defined in products.lp
% begin(R, T, GT)                Defined in state.lp
% cap(P, C, GT)                  Defined in products.lp
% capLocation(L)                 Defined in facts.lp
% csPrepared(M, GT)              Defined in machines.lp
% end(R, T, GT)                  Defined in state.lp
% failure(R, T, GT)              Defined in state.lp
% holding(R, P, GT)              Defined in robots.lp
% location(R, L, GT)             Defined in state.lp
% order(O)                       Defined in facts.lp
% quantity(Q)                    Defined in facts.lp
% ring(O, R, C)                  Defined in orders.lp
% ring(P, R, C, GT)              Defined in products.lp
% storing(M, P, GT)              Defined in machines.lp


#program ourTeam(t).

task(mountCap(L, O, Q))                              :- order(O), quantity(Q), capLocation(_, L).
points(mountCap(L, O, Q), 10)                        :- task(mountCap(L, O, Q)).
duration(mountCap(L, O, Q), @mountCapTaskDuration()) :- task(mountCap(L, O, Q)).
location(mountCap(L, O, Q), L)	                     :- task(mountCap(L, O, Q)).

#program transition(gt).
inUse(L, gt) :- begin(R, mountCap(L, O, Q), gt), not end(R, mountCap(L, O, Q), gt).
free(L, gt)  :- end(_, mountCap(L, _, _), gt).

possible(R, mountCap(L, O, Q), gt) :- task(mountCap(L, O, Q)), holding(R, P, gt), capLocation(M, L),
									  csPrepared(M, gt), not storing(M, _, gt), not inUse(L, gt), base(P, B),
									  base(O, B), ring(P, 1, R1, gt), ring(O, 1, R1), ring(O, 2, R2),
									  ring(P, 2, R2, gt), ring(P, 3, R3, gt), ring(O, 3, R3), cap(P, none, gt).

{ putDown(R, L, P, gt);
  capMounted(M, P, gt);
  process(M, P, gt)
  } = 3 :- end(R, mountCap(L, O, Q), gt), not failure(mountCap(L, O, Q), gt), capLocation(M, L), holding(R, P, gt).

