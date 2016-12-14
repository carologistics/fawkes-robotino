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
deliverTask(deliver(L, O, Q))          :- order(O), quantity(Q), deliverLocation(L).
task(T)                                :- deliverTask(T).
points(T, 20)                          :- deliverTask(T).
duration(T, @deliverTaskDuration())    :- deliverTask(T).
location(T, L)	                       :- deliverTask(T), deliverLocation(L).

lateDeliverTask(lateDeliver(L, O, Q))  :- order(O), quantity(Q), deliverLocation(L).
task(T)                                :- lateDeliverTask(T).
points(T, 5)                           :- lateDeliverTask(T).
duration(T, @deliverTaskDuration())    :- lateDeliverTask(T).
location(T, L)	                       :- lateDeliverTask(T), deliverLocation(L).

#program transition(gt).

#external spawnTask(T, gt) : deliverTask(T).
#external spawnTask(T, gt) : lateDeliverTask(T).

inUse(L, gt) :- begin(R, T, gt), deliverTask(T), location(T, L).
free(L, gt)  :- end(_, T, gt), deliverTask(T), location(T, L).

inUse(L, gt) :- begin(R, T, gt), lateDeliverTask(T), location(T, L).
free(L, gt)  :- end(_, T, gt), lateDeliverTask(T), location(T, L).

possible(R, deliver(L, O, Q), gt) :- deliverTask(deliver(L, O, Q)), holding(R, P, gt), not inUse(L, gt), base(P, B),
									 base(O, B), ring(P, 1, R1, gt), ring(O, 1, R1), ring(O, 2, R2), ring(P, 2, R2, gt),
									 ring(P, 3, R3, gt), ring(O, 3, R3), cap(P, C, gt), cap(O, C), robot(R, gt),
									 location(R, RL, gt).%, driveDuration(RL, L, DD), duration(deliver(L, O, Q), TD),
									 %D = TD + DD, D < E, orderDeadline(O, E).

possible(R, lateDeliver(L, O, Q), gt) :- lateDeliverTask(lateDeliver(L, O, Q)), holding(R, P, gt), not inUse(L, gt),
										 base(P, B), base(O, B), ring(P, 1, R1, gt), ring(O, 1, R1), ring(O, 2, R2),
										 ring(P, 2, R2, gt), ring(P, 3, R3, gt), ring(O, 3, R3), cap(P, C, gt),
										 cap(O, C), robot(R, gt).

{ putDown(R, L, P, gt);
  delivered(P, gt);
  end(R, lateDeliver(L, O, Q), gt)
  } = 3 :- end(R, deliver(L, O, Q), gt), not failure(deliver(L, O, Q), gt), deliverLocation(L), holding(R, P, gt).

{ putDown(R, L, P, gt);
  delivered(P, gt);
  end(R, deliver(L, O, Q), gt)
  } = 3 :- end(R, lateDeliver(L, O, Q), gt), not failure(lateDeliver(L, O, Q), gt), deliverLocation(L),
		   holding(R, P, gt).

