% Defined programs:
% ourTeam(t)                     Setup the prepare cs tasks, that are possible, as soon as we know the special
%                                locations.
%
% In this file defined predicates:
% csPrepared(M, GT)              Also defined in machines.lp
% generateProduct(R, B, GT)      The robot R generated a new product with base color B at GT.
% inUse(L, R, GT)                The machine side L is in use by robot R for time step GT.
% points(prepareCS(_), P)        How many points are awarded for the preparation.
% possible(R, prepareCS(_), GT)  If robot R can perform the preparation task at GT.
% process(M, P, GT)              The machine M starts the processing of the product P at GT.
% task(prepareCS(L))             The task to prepare the cap station L. S is to enumerate how often a robot is able to
%                                perform this task. Preparing means get a dummy product from the shelf and insert it in
%                                the cap station for cap unmountment.
% taskDuration(prepareCS(_), D)  How long it takes to prepare the cap station, once the robot has arrived.
% taskLocation(prepareCS(_), L)  Where the preparation will take place.
% toBeDone(prepareCS(_), GT)     The prepare CS tasks are possible.
%
% Additionally used predicates:
% capLocation(L)                 Defined in facts.lp
% doing(R, T, D, GT)             Defined in state.lp
% end(R, T, GT)                  Defined in state.lp
% generatedProduct(R, P, GT)     Defined in products.lp
% holding(R, P, GT)              Defined in robots.lp
% horizon(H)                     Defined in facts.lp
% locationToMachineMap(L, M)     Defined in facts.lp
% productionStarted              Defined in facts.lp
% robot(R)                       Defined in facts.lp
% storing(M, P, GT)              Defined in machines.lp


#program ourTeam(t).

prepCSTask(prepareCS(L))                  :- capLocation(L).
task(T)                                   :- prepCSTask(T).
points(T, 3)                              :- prepCSTask(T).
taskDuration(T, @prepareCSTaskDuration()) :- prepCSTask(T).
taskLocation(prepareCS(L), L)             :- task(prepareCS(L)).
toBeDone(T, GT)                           :- prepCSTask(T), productionStarted, horizon(H), GT = 0..H-1.

inUse(L, R, GT) :- doing(R, T, _, GT), prepCSTask(T), taskLocation(T, L).
inUse(L, R, GT) :- end(R, T, GT), prepCSTask(T), taskLocation(T, L).

possible(R, T, GT) :- not impossible(R, T, GT), prepCSTask(T), robot(R), GT = 0..@horizon()-1.

% The machine has to have an empty output.
impossible(R, T, GT) :- prepCSTask(T), robot(R), taskLocation(T, L), locationToMachineMap(L, M), storing(M, _, GT).
% The machine has to be in idle.
impossible(R, T, GT) :- prepCSTask(T), robot(R), taskLocation(T, L), locationToMachineMap(L, M),
                        processing(M, _, _, GT).
% The machine has to be not prepared.
impossible(R, T, GT) :- prepCSTask(T), robot(R), taskLocation(T, L), locationToMachineMap(L, M), csPrepared(M, GT),
                        GT = 0..@horizon()-1.
% The robot has to hold no product.
impossible(R, T, GT) :- prepCSTask(T), robot(R), holding(R, _, GT), GT = 0..@horizon()-1.

csPreparedByRobot(R, M, GT) :- end(R, T, GT), prepCSTask(T), taskLocation(T, L), locationToMachineMap(L, M).

csPrepared(M, GT)                     :- csPreparedByRobot(_, M, GT).
generateProduct(R, "TRANSPARENT", GT) :- csPreparedByRobot(R, _, GT).
process(M, P, GT)                     :- csPreparedByRobot(R, M, GT), generatedProduct(R, P, GT).
