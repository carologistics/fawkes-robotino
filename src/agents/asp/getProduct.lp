% Defined programs:
% ourTeam(t)                     Setup the get product tasks, that are possible, as soon as we know the special locations.
%
% In this file defined predicates:
% free(L, GT)                    The machine side L is freed in time step GT.
% inUse(L, R, GT)                The machine side L is in use by robot R for time step GT.
% pickup(R, P, L)                The robot R picked up product P at location L.
% possible(R, getProduct(_), GT) If robot R can perform the retreival task at GT.
% task(getProduct(L, S))         The task to fetch a product from the base station L. S is to enumerate how often a
%                                robot is able to perform this task.
% taskDuration(getProduct(_), D) How long it takes to retreive the product from the machine, once the robot has arrived.
% taskLocation(getProduct(_), L) Where the retreival will take place.
% toBeDone(getProduct(_), 0)     The get product tasks are possible.
%
% Additionally used predicates:
% begin(R, T, GT)                Defined in state.lp
% end(R, T, GT)                  Defined in state.lp
% failure(R, T, GT)              Defined in state.lp
% getLocation(M, L)              Defined in facts.lp
% holding(R, P, GT)              Defined in robots.lp
% productionStarted              Defined in facts.lp
% robot(R)                       Defined in facts.lp
% storing(M, P, GT)              Defined in machines.lp


#program ourTeam(t).

getProductTask(getProduct(L, S))           :- getLocation(_, L), S = 1..@maxGetProduct().
task(T)                                    :- getProductTask(T).
taskDuration(T, @getProductTaskDuration()) :- getProductTask(T).
taskLocation(getProduct(L, S), L)          :- task(getProduct(L, S)).
toBeDone(T, 0)                             :- getProductTask(T), productionStarted.

inUse(L, R, GT) :- begin(R, getProduct(L, S), GT).
free(L, GT) :- end(R, getProduct(L, _), GT).

possible(R, getProduct(L, S), GT) :- task(getProduct(L, S)), robot(R), not holding(R, _, GT), getLocation(M, L),
									 storing(M, P, GT), GT = 0..@horizon()-1.

pickUp(R, P, M, GT) :- end(R, getProduct(L, S), GT), getLocation(M, L), storing(M, P, GT-1).

