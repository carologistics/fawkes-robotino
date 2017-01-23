% Defined programs:
% ourTeam(t)                     Setup the get product tasks, that are possible, as soon as we know the special
%                                locations.
%
% In this file defined predicates:
% inUse(L, R, GT)                The machine side L is in use by robot R for time step GT.
% pickup(R, P, GT)               The robot R picked up product P at time step GT.
% possible(R, getProduct(_), GT) If robot R can perform the retreival task at GT.
% task(getProduct(L))            The task to fetch a product from the base station L.
% taskDuration(getProduct(_), D) How long it takes to retreive the product from the machine, once the robot has arrived.
% taskLocation(getProduct(_), L) Where the retreival will take place.
% toBeDone(getProduct(_), GT)    The get product tasks are possible.
%
% Additionally used predicates:
% doing(R, T, D, GT)             Defined in state.lp
% end(R, T, GT)                  Defined in state.lp
% getLocation(L)                 Defined in facts.lp
% holding(R, P, GT)              Defined in robots.lp
% horizon(H)                     Defined in facts.lp
% locationToMachineMap(L, M)     Defined in facts.lp
% productionStarted              Defined in facts.lp
% robot(R)                       Defined in facts.lp
% storing(M, P, GT)              Defined in machines.lp


#program ourTeam(t).

getProductTask(getProduct(L))              :- getLocation(L).
task(T)                                    :- getProductTask(T).
taskDuration(T, @getProductTaskDuration()) :- getProductTask(T).
taskLocation(getProduct(L), L)             :- task(getProduct(L)).
toBeDone(T, GT)                            :- getProductTask(T), productionStarted, horizon(H), GT=0..H-1.

inUse(L, R, GT) :- doing(R, T, _, GT), getProductTask(T), taskLocation(T, L).
inUse(L, R, GT) :- end(R, T, GT), getProductTask(T), taskLocation(T, L).

possible(R, T, GT) :- getProductTask(T), taskLocation(T, L), robot(R), not holding(R, _, GT),
                      locationToMachineMap(L, M), storing(M, P, GT), horizon(H), GT = 0..H-1.

pickUp(R, P, GT) :- end(R, T, GT), getProductTask(T), taskLocation(T, L), locationToMachineMap(L, M),
                    storing(M, P, GT-1).

