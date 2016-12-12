% Defined programs:
% ourTeam(t)                     Setup the delivery tasks, that are possible, as soon as we know the special locations.
% transition(gt)                 Update implications from the delivery tasks.
%
% In this file defined predicates:
% delivered(P, GT)               Product P was delivered at GT.
% duration(deliver(_), D)        How long it takes to put the product into the delivery station, once the robot has
%                                arrived.
% free(L, gt)                    The machine side L is freed in time step GT.
% inUse(L, GT)                   The machine side L is in use for time step GT.
% location(deliver(_), L)        Where the delivery will take place.
% points(deliver(_), P)          How many points are awarded for the delivery.
% possible(R, deliver(_), GT)    If robot R can perform the delivery task at GT.
% putDown(R, L, P, GT)           Robot R puts product P down at location L in GT.
% task(deliver(L, O, Q))         The task to deliver a product of (O,Q) to L within the time for the order.
% task(lateDeliver(L, O, Q))     The task to deliver a product of (O,Q) to L after the time for the order.
%
% Additionally used predicates:
% base(O, B)                     Defined in orders.lp
% base(P, B)                     Defined in products.lp
% begin(R, T, GT)                Defined in state.lp
% cap(O, C)                      Defined in orders.lp
% cap(P, C, GT)                  Defined in products.lp
% deliverLocation(L)             Defined in facts.lp
% driveDuration(L1, L2, D)       Defined in facts.lp
% end(R, T, GT)                  Defined in state.lp
% failure(R, T, GT)              Defined in state.lp
% holding(R, P, GT)              Defined in robots.lp
% location(R, L, GT)             Defined in state.lp
% order(O)                       Defined in facts.lp
% orderDeadline(O, D)            Defined in orders.lp
% quantity(Q)                    Defined in facts.lp
% ring(O, R, C)                  Defined in orders.lp
% ring(P, R, C, GT)              Defined in products.lp


#program ourTeam(t).
task(deliver(L, O, Q))                             :- order(O), quantity(Q), deliverLocation(L).
points(deliver(L, O, Q), 20)                       :- task(deliver(L, O, Q)).
duration(deliver(L, O, Q), @deliverTaskDuration()) :- task(deliver(L, O, Q)).
location(deliver(L, O, Q), L)	                   :- task(deliver(L, O, Q)).

task(lateDeliver(L, O, Q))                             :- order(O), quantity(Q), deliverLocation(L).
points(lateDeliver(L, O, Q), 5)                        :- task(lateDeliver(L, O, Q)).
duration(lateDeliver(L, O, Q), @deliverTaskDuration()) :- task(lateDeliver(L, O, Q)).
location(lateDeliver(L, O, Q), L)	                   :- task(lateDeliver(L, O, Q)).

#program transition(gt).
inUse(L, gt) :- begin(R, deliver(L, O, Q), gt), not end(R, deliver(L, O, Q), gt).
free(L, gt)  :- end(_, deliver(L, _, _), gt).

inUse(L, gt) :- begin(R, lateDeliver(L, O, Q), gt), not end(R, lateDeliver(L, O, Q), gt).
free(L, gt)  :- end(_, lateDeliver(L, _, _), gt).

possible(R, deliver(L, O, Q), gt) :- task(deliver(L, O, Q)), holding(R, P, gt), not inUse(L, gt), base(P, B),
									 base(O, B), ring(P, 1, R1, gt), ring(O, 1, R1), ring(O, 2, R2), ring(P, 2, R2, gt),
									 ring(P, 3, R3, gt), ring(O, 3, R3), cap(P, C, gt), cap(O, C),
									 location(R, RL, gt), driveDuration(RL, L, DD), duration(deliver(L, O, Q), TD),
									 D = TD + DD, D < E, orderDeadline(O, E).

possible(R, lateDeliver(L, O, Q), gt) :- task(lateDeliver(L, O, Q)), holding(R, P, gt), not inUse(L, gt), base(P, B),
										 base(O, B), ring(P, 1, R1, gt), ring(O, 1, R1), ring(O, 2, R2),
										 ring(P, 2, R2, gt), ring(P, 3, R3, gt), ring(O, 3, R3), cap(P, C, gt),
										 cap(O, C).

{ putDown(R, L, P, gt);
  delivered(P, gt);
  end(R, lateDeliver(L, O, Q), gt)
  } = 3 :- end(R, deliver(L, O, Q), gt), not failure(deliver(L, O, Q), gt), deliverLocation(L), holding(R, P, gt).

{ putDown(R, L, P, gt);
  delivered(P, gt);
  end(R, deliver(L, O, Q), gt)
  } = 3 :- end(R, lateDeliver(L, O, Q), gt), not failure(lateDeliver(L, O, Q), gt), deliverLocation(L),
		   holding(R, P, gt).

