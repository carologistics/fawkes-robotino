% Defined programs:
% ourTeam(t)                     Setup the feed rs tasks, that are possible, as soon as we know the special locations.
%
% In this file defined predicates:
% rsFed(M, GT)                   The ring station M has received an additional base for the advanced colors in GT.
% inUse(L, R, GT)                The machine side L is in use by robot R for time step GT.
% points(feedRS(_), P)           How many points are awarded for the feeding.
% possible(R, feedRS(_), GT)     If robot R can perform the feed task at GT.
% putDown(R, GT)                 Robot R puts his product down in GT.
% rsFed(M, GT)                   Machine M was fed with a product in GT.
% task(feedRS(L))                The task to feed a product into ring station L.
% taskDuration(feedRS(_), D)     How long it takes to put the product onto the ring stations slide, once the robot has
%                                arrived.
% taskLocation(feedRS(_), L)     Where the feeding will take place.
% toBeDone(feedRS(_), GT)        The feed RS tasks are possible.
%
% Additionally used predicates:
% doing(R, T, D, GT)             Defined in state.lp
% end(R, T, GT)                  Defined in state.lp
% holding(R, P, GT)              Defined in robots.lp
% horizon(H)                     Defined in facts.lp
% locationToMachineMap(L, M)     Defined in facts.lp
% needsFillment(M)               Defined in facts.lp
% productionStarted              Defined in facts.lp
% productCap                     Defined in products.lp
% prodcutRing                    Defined in products.lp
% ringColorCost(Col, Cost)       Defined in facts.lp
% ringCost(C)                    Defined in facts.lp
% ringLocation(L)                Defined in facts.lp
% ringStationAssignment(M, C)    Defined in facts.lp
% robot(R)                       Defined in facts.lp


#program ourTeam(t).

feedRSTask(feedRS(L))                  :- ringLocation(L).
task(T)                                :- feedRSTask(T).
points(T, 2)                           :- feedRSTask(T).
taskDuration(T, @feedRSTaskDuration()) :- feedRSTask(T).
taskLocation(feedRS(L), L)             :- task(feedRS(L)).
toBeDone(T, GT)                        :- feedRSTask(T), productionStarted, horizon(H), GT=0..H-1.

inUse(L, R, GT) :- doing(R, T, _, GT), feedRSTask(T), taskLocation(T, L).
inUse(L, R, GT) :- end(R, T, GT), feedRSTask(T), taskLocation(T, L).

% Maybe get more sophisticated and only allow transparent bases for the last fillment.
possible(R, T, GT) :- feedRSTask(T), robot(R), horizon(H), GT = 0..H-1, not impossible(R, T, GT).

% The robot has to hold a product.
impossible(R, T, GT) :- feedRSTask(T), robot(R), not holding(R, _, GT), horizon(H), GT = 0..H-1.
% The machine has to serve a color which needs fillment.
impossible(R, T, GT) :- feedRSTask(T), robot(R), taskLocation(T, L), locationToMachineMap(L, M), not needsFillment(M),
                        horizon(H), GT = 0..H-1.
% The machine is not full yet.
impossible(R, T, GT) :- feedRSTask(T), robot(R), taskLocation(T, L), locationToMachineMap(L, M), rsFillState(M, 3, GT).
% Do not feed a product with a cap.
impossible(R, T, GT) :- feedRSTask(T), robot(R), holding(R, P, GT), productCap(P, _, GT).
% Do not feed a product with a ring.
impossible(R, T, GT) :- feedRSTask(T), robot(R), holding(R, P, GT), productRing(P, 1, _, GT).
% Do not feed a broken machine.
impossible(R, T, GT) :- feedRSTask(T), robot(R), taskLocation(T, L), brokenLocation(L, GT).

rsFedByRobot(R, M, P, GT) :- end(R, T, GT), feedRSTask(T), taskLocation(T, L), locationToMachineMap(L, M),
                             holding(R, P, GT-1).

rsFed(M, GT)   :- rsFedByRobot(_, M, _, GT).
putDown(R, GT) :- rsFedByRobot(R, _, _, GT).
