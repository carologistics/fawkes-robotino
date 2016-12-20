% Defined programs:
% ourTeam(t)                     Setup the get base tasks, that are possible, as soon as we know the special locations.
%
% In this file defined predicates:
% taskDuration(getBase(_), D)    How long it takes to retreive the base from the base station, once the robot has
%                                arrived.
% free(L, gt)                    The machine side L is freed in time step GT.
% generateProduct(R, B, GT)      The robot R generated a new product with base color B at GT.
% inUse(L, R, GT)                The machine side L is in use by robot R for time step GT.
% taskLocation(getBase(_), L)    Where the retreival will take place.
% pickup(R, P, L)                The robot R picked up product P at location L.
% possible(R, getBase(_), GT)    If robot R can perform the retreival task at GT.
% toBeDone(getBase(_), 0)        The get base tasks are possible.
% task(getBase(L, B, S))         The task to get a new base of color Bfrom the base station L. S is to enumerate how
%                                often a robot is able to perform this task.
%
% Additionally used predicates:
% baseLocation(L)                Defined in facts.lp
% begin(R, T, GT)                Defined in state.lp
% end(R, T, GT)                  Defined in state.lp
% generatedProduct(R, P, GT)     Defined in products.lp
% holding(R, P, GT)              Defined in robots.lp
% productionStarted              Defined in facts.lp
% robot(R)                       Defined in facts.lp


#program ourTeam(t).

% The tasks are always existent.
getBaseTask(getBase(L, B, S))       :- baseLocation(L), S = 1..@maxGetBase(), baseColor(B).
task(T)                             :- getBaseTask(T).
% Prevents a senseless getBase task, but has a great negative impact on the solving process.
%points(T, -1)                       :- getBaseTask(T).
taskDuration(T, @getTaskDuration()) :- getBaseTask(T).
taskLocation(getBase(L, B, S), L)   :- task(getBase(L, B, S)).
toBeDone(T, 2)                      :- getBaseTask(T), productionStarted.

inUse(L, R, GT) :- begin(R, T, GT), getBaseTask(T), baseLocation(L).
free(L, GT)  :- end(R, T, GT), getBaseTask(T), baseLocation(L).

possible(R, T, GT) :- getBaseTask(T), robot(R), not holding(R, _, GT), GT = 0..@horizon()-1.

generateProduct(R, B, GT) :- end(R, getBase(_, B, _), GT).
pickUp(R, P, "BS", GT) :- end(R, getBase(_, B, _), GT), generatedProduct(R, P, GT).
