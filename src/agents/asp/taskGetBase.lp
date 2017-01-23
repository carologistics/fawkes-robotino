% Defined programs:
% ourTeam(t)                     Setup the get base tasks, that are possible, as soon as we know the special locations.
%
% In this file defined predicates:
% generateProduct(R, B, GT)      The robot R generated a new product with base color B at GT.
% inUse(L, R, GT)                The machine side L is in use by robot R for time step GT.
% pickup(R, P, GT)               The robot R picked up product P at time step GT.
% possible(R, getBase(_), GT)    If robot R can perform the retreival task at GT.
% task(getBase(L, B))            The task to get a new base of color Bfrom the base station L.
% taskBase(T, B)                 The base color B fetched by task T.
% taskDuration(getBase(_), D)    How long it takes to retreive the base from the base station, once the robot has
%                                arrived.
% taskLocation(getBase(_), L)    Where the retreival will take place.
% toBeDone(getBase(_), GT)       The get base tasks are possible.
%
% Additionally used predicates:
% baseLocation(L)                Defined in facts.lp
% doing(R, T, D, GT)             Defined in state.lp
% end(R, T, GT)                  Defined in state.lp
% generatedProduct(R, P, GT)     Defined in products.lp
% holding(R, P, GT)              Defined in robots.lp
% horizon(H)                     Defined in facts.lp
% productionStarted              Defined in facts.lp
% robot(R)                       Defined in facts.lp


#program ourTeam(t).

getBaseTask(getBase(L, B))          :- baseLocation(L), baseColor(B).
task(T)                             :- getBaseTask(T).
taskBase(getBase(L, B), B)          :- task(getBase(L, B)).
% Prevents a senseless getBase task, but has a great negative impact on the solving process.
%points(T, -1)                       :- getBaseTask(T).
taskDuration(T, @getTaskDuration()) :- getBaseTask(T).
taskLocation(getBase(L, B), L)      :- task(getBase(L, B)).
toBeDone(T, GT)                     :- getBaseTask(T), productionStarted, horizon(H), GT=0..H-1.

inUse(L, R, GT) :- doing(R, T, _, GT), getBaseTask(T), baseLocation(L).
inUse(L, R, GT) :- end(R, T, GT), getBaseTask(T), baseLocation(L).

possible(R, T, GT) :- getBaseTask(T), robot(R), not holding(R, _, GT), horizon(H), GT = 0..H-1.

generateProduct(R, B, GT) :- end(R, T, GT), getBaseTask(T), taskBase(T, B).
pickUp(R, P, GT) :- end(R, T, GT), getBaseTask(T), generatedProduct(R, P, GT).
