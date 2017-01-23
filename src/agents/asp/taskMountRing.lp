% Defined programs:
% ourTeam(t)                     Setup the mount ring tasks, that are possible, as soon as we know the special
%                                locations.
%
% In this file defined predicates:
% inUse(L, R, GT)                The machine side L is in use by robot R for time step GT.
% oneShotTask(mountRing(_))      The task is only doable one time.
% points(mountRing(_), P)        How many points are awarded for the mounting.
% possible(R, mountRing(_), GT)  If robot R can perform the mount task at GT.
% process(M, P, GT)              The machine M starts the processing of the product P at GT.
% putDown(R, GT)                 Robot R puts his product down in GT.
% ringMounted(M, P, C, GT)       Machine M mounted a ring of color C on product P from in GT.
% task(mountRing(L, O, Q, R))    The task to mount the R-th ring on a product of (O,Q) at L.
% taskDuration(mountRing(_), D)  How long it takes to put the product into ring cap station, once the robot has arrived.
% taskLocation(mountRing(_), L)  Where the mounting will take place.
% toBeDone(mountRing(_), 0)      The task is possible.
%
% Additionally used predicates:
% base(O, B)                     Defined in orders.lp
% doing(R, T, D, GT)             Defined in state.lp
% end(R, T, GT)                  Defined in state.lp
% holding(R, P, GT)              Defined in robots.lp
% locationToMachineMap(L, M)     Defined in facts.lp
% order(O)                       Defined in facts.lp
% quantity(Q)                    Defined in facts.lp
% productBase(P, B)              Defined in products.lp
% productCap(P, C, GT)           Defined in products.lp
% productRing(P, R, C, GT)       Defined in products.lp
% quantitiy(Q)                   Defined in facts.lp
% ring(O, R, C)                  Defined in orders.lp
% ringColorCost(Col, Cost)       Defined in facts.lp
% ringLocation(L)                Defined in facts.lp
% rsFillState(M, FS, GT)         Defined in machines.lp
% storing(M, P, GT)              Defined in machines.lp


#program ourTeam(t).

% For the points we cheat a little bit. The points are only awarded for the last ring, but this is difficult to model
% so we award for every mount and subtract the before awarded from the points for the next ring. So we don't get 80
% points for the 3rd ring of a C3, but only 50, because we got 10 for the 1st and 20 for the 2nd, in total 80.
ringTask(mountRing(L, O, Q, R))           :- order(O), quantity(Q), ringLocation(L), R = 1..3.
taskLocation(mountRing(L, O, Q, R), L)	  :- ringTask(mountRing(L, O, Q, R)).
order(mountRing(L, O, Q, R), O)           :- ringTask(mountRing(L, O, Q, R)).
ring(mountRing(L, O, Q, R), R)            :- ringTask(mountRing(L, O, Q, R)).
points(mountRing(L, O, Q, 1), 10)         :- ringTask(mountRing(L, O, Q, 1)).
points(mountRing(L, O, Q, 2), 20)         :- ringTask(mountRing(L, O, Q, 2)).
points(mountRing(L, O, Q, 3), 50)         :- ringTask(mountRing(L, O, Q, 3)).
task(T)                                   :- ringTask(T).
taskDuration(T, @mountRingTaskDuration()) :- ringTask(T).
oneShotTask(T)                            :- ringTask(T).

#external toBeDone(T, 0) : ringTask(T).

inUse(L, R, GT) :- doing(R, T, _, GT), ringTask(T), taskLocation(T, L).
inUse(L, R, GT) :- end(R, T, GT), ringTask(T), taskLocation(T, L).

possible(R, T, GT) :- ringTask(T), robot(R), GT = 0..@horizon()-1, not impossible(R, T, GT).

% The machine has to have an empty output.
impossible(R, T, GT) :- ringTask(T), robot(R), taskLocation(T, L), locationToMachineMap(L, M), storing(M, _, GT).
% The machine has to be in idle.
impossible(R, T, GT) :- ringTask(T), robot(R), taskLocation(T, L), locationToMachineMap(L, M), processing(M, _, _, GT).
% The robot has to hold a product.
impossible(R, T, GT) :- ringTask(T), robot(R), not holding(R, _, GT), GT = 0..@horizon()-1.
% The product has to have the correct base color.
impossible(R, T, GT) :- ringTask(T), holding(R, P, GT), order(T, O), base(O, B), not productBase(P, B).
% The product is not allowed to have a cap.
impossible(R, T, GT) :- ringTask(T), holding(R, P, GT), productCap(P, _, GT).
% The all lower rings has to be mounted.
impossible(R, T, GT) :- ringTask(T), holding(R, P, GT), order(T, O), ring(T, TaskRing), ring(O, Ring, Color),
                        Ring < TaskRing, not productRing(P, Ring, Color, GT).
% The fill state of the ring station has to be high enough.
impossible(R, T, GT) :- ringTask(T), robot(R), ring(T, Ring), order(T, O), taskLocation(T, L),
                        locationToMachineMap(L, M), ring(O, Ring, Color), ringColorCost(Color, Cost),
                        rsFillState(M, FS, GT), FS < Cost.

ringMountedByRobot(R, P, M, C, GT) :- end(R, T, GT), ringTask(T), taskLocation(T, L), locationToMachineMap(L, M),
                                      holding(R, P, GT-1), order(T, O), ring(T, Ring), ring(O, Ring, C).

putDown(R, GT)           :- ringMountedByRobot(R, _, _, _, GT).
ringMounted(M, P, C, GT) :- ringMountedByRobot(_, P, M, C, GT).
process(M, P, GT)        :- ringMountedByRobot(_, P, M, _, GT).


