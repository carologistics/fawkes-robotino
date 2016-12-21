% Defined programs:
% ourTeam(t)                     Setup the mount cap tasks, that are possible, as soon as we know the special locations.
%
% In this file defined predicates:
% capMounted(M, P, GT)           Machine M mounted a cap on product P from in GT.
% free(L, GT)                    The machine side L is freed in time step GT.
% inUse(L, R, GT)                The machine side L is in use by robot R for time step GT.
% points(mountCap(_), P)         How many points are awarded for the mounting.
% possible(R, mountCap(_), GT)   If robot R can perform the mount task at GT.
% process(M, P, GT)              The machine M starts the processing of the product P at GT.
% putDown(R, GT)                 Robot R puts his product in GT.
% task(mountCap(L, O, Q))        The task to mount a cap on a product of (O,Q) at L.
% taskDuration(mountCap(_), D)   How long it takes to put the product into the cap station, once the robot has arrived.
% taskLocation(mountCap(_), L)   Where the mounting will take place.
% toBeDone(mountCap(_), 0)       The task has to be done.
%
% Additionally used predicates:
% base(O, B)                     Defined in orders.lp
% begin(R, T, GT)                Defined in state.lp
% capLocation(L)                 Defined in facts.lp
% csPrepared(M, GT)              Defined in machines.lp
% end(R, T, GT)                  Defined in state.lp
% holding(R, P, GT)              Defined in robots.lp
% order(O)                       Defined in facts.lp
% productBase(P, B)              Defined in products.lp
% productCap(P, C, GT)           Defined in products.lp
% productRing(P, R, C, GT)       Defined in products.lp
% quantity(Q)                    Defined in facts.lp
% ring(O, R, C)                  Defined in orders.lp
% storing(M, P, GT)              Defined in machines.lp


#program ourTeam(t).

capTask(mountCap(L, O, Q))               :- order(O), quantity(Q), capLocation(_, L).
order(mountCap(L, O, Q), O)              :- capTask(mountCap(L, O, Q)).
taskLocation(mountCap(L, O, Q), L)       :- capTask(mountCap(L, O, Q)).
task(T)                                  :- capTask(T).
points(T, 10)                            :- capTask(T).
taskDuration(T, @mountCapTaskDuration()) :- capTask(T).

#external toBeDone(T, 0) : capTask(T).

inUse(L, R, GT) :- begin(R, T, GT), capTask(T), taskLocation(T, L).
free(L, GT) :- end(_, T, GT), capTask(T), taskLocation(T, L).

possible(R, T, GT) :- not impossible(R, T, GT), capTask(T), robot(R), GT = 0..@horizon()-1.

% The machine has to be prepared.
impossible(R, T, GT) :- capTask(T), robot(R), taskLocation(T, L), capLocation(M, L), not csPrepared(M, GT),
						GT = 0..@horizon()-1.
% The robot has to hold a product.
impossible(R, T, GT) :- capTask(T), robot(R), not holding(R, _, GT), GT = 0..@horizon()-1.
% The product has to have the correct base color.
impossible(R, T, GT) :- capTask(T), holding(R, P, GT), order(T, O), base(O, B), not productBase(P, B).
% The product is not allowed to have a cap.
impossible(R, T, GT) :- capTask(T), holding(R, P, GT), productCap(P, _, GT).
% The all rings has to be mounted.
impossible(R, T, GT) :- capTask(T), holding(R, P, GT), order(T, O), ring(O, Ring, Color),
						not productRing(P, Ring, Color, GT).

capMountedByRobot(R, M, P, GT) :- end(R, T, GT), capTask(T), taskLocation(T, L), capLocation(M, L), holding(R, P, GT-1).

putDown(R, GT)       :- capMountedByRobot(R, _, _, GT).
capMounted(M, P, GT) :- capMountedByRobot(_, M, P, GT).
process(M, P, GT)    :- capMountedByRobot(_, M, P, GT).

