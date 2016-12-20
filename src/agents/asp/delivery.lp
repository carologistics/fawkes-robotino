% Defined programs:
% ourTeam(t)                     Setup the delivery tasks, that are possible, as soon as we know the special locations.
%
% In this file defined predicates:
% free(L, GT)                    The machine side L is freed in time step GT.
% inUse(L, R, GT)                The machine side L is in use by robot R for time step GT.
% points(deliver(_), P)          How many points are awarded for the delivery.
% possible(R, deliver(_), GT)    If robot R can perform the delivery task at GT.
% putDown(R, P, GT)              Robot R puts product P down in GT.
% task(deliver(L, O, Q))         The task to deliver a product of (O,Q) to L within the time for the order.
% task(lateDeliver(L, O, Q))     The task to deliver a product of (O,Q) to L after the time for the order.
% taskDuration(deliver(_), D)    How long it takes to put the product into the delivery station, once the robot has
%                                arrived.
% taskLocation(deliver(_), L)    Where the delivery will take place.
% toBeDone(deliver(_), 0)        The task is possible.
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
% holding(R, P, GT)              Defined in robots.lp
% location(R, L, GT)             Defined in state.lp
% order(O)                       Defined in facts.lp
% orderBegin(O, B)               Defined in orders.lp
% orderDeadline(O, D)            Defined in orders.lp
% quantity(Q)                    Defined in facts.lp
% ring(O, R, C)                  Defined in orders.lp
% ring(P, R, C, GT)              Defined in products.lp


#program ourTeam(t).
deliverTask(deliver(L, O, Q))         :- order(O), quantity(Q), deliverLocation(L).
order(deliver(L, O, Q), O)            :- deliverTask(deliver(L, O, Q)).
points(T, 20)                         :- deliverTask(T).

lateDeliverTask(lateDeliver(L, O, Q)) :- order(O), quantity(Q), deliverLocation(L).
order(lateDeliver(L, O, Q), O)        :- lateDeliverTask(lateDeliver(L, O, Q)).
points(T, 5)                          :- lateDeliverTask(T).

allDeliverTasks(T) :- deliverTask(T).
allDeliverTasks(T) :- lateDeliverTask(T).

task(T)                                 :- allDeliverTasks(T).
taskDuration(T, @deliverTaskDuration()) :- allDeliverTasks(T).
taskLocation(T, L)	                    :- allDeliverTasks(T), deliverLocation(L).

#external toBeDone(T, 0) : allDeliverTasks(T).

inUse(L, R, GT) :- begin(R, T, GT), allDeliverTasks(T), taskLocation(T, L).
free(L, GT) :- end(_, T, GT), allDeliverTasks(T), taskLocation(T, L).

possible(R, T, GT) :- allDeliverTasks(T), robot(R), GT = 0..@horizon()-1, not impossible(R, T, GT).

% The robot has to hold a product.
impossible(R, T, GT) :- allDeliverTasks(T), robot(R), not holding(R, _, GT), GT = 0..@horizon()-1.
% The product has to have the correct base color.
impossible(R, T, GT) :- allDeliverTasks(T), holding(R, P, GT), order(T, O), base(O, B), not base(P, B).
% The product has to have the correct cap color.
impossible(R, T, GT) :- allDeliverTasks(T), holding(R, P, GT), order(T, O), cap(O, C), not cap(P, C, GT).
% The all rings has to be mounted.
impossible(R, T, GT) :- allDeliverTasks(T), holding(R, P, GT), order(T, O), ring(O, Ring, Color),
						not ring(P, Ring, Color, GT).

% Do not deliver before its time.
impossible(R, T, GT) :- deliverTask(T), currentTime(CT), order(T, O), orderBegin(O, B), B < GT + CT, robot(R),
						GT = 0..@horizon()-1.
% Do not deliver after its time.
impossible(R, T, GT) :- deliverTask(T), currentTime(CT), order(T, O), orderDeadline(O, D), D > GT + CT, robot(R),
						GT = 0..@horizon()-1.
% Do not late deliver before its time.
impossible(R, T, GT) :- lateDeliverTask(T), currentTime(CT), order(T, O), orderDeadline(O, D), D < GT + CT,
						robot(R), GT = 0..@horizon()-1.


putDown(R, P, GT) :- end(R, T, GT), allDeliverTasks(T), holding(R, P, GT-1).
