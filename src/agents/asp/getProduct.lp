% Defined programs:
% ourTeam(t)                     Setup the get product tasks, that are possible, as soon as we know the special locations.
% transition(gt)                 Update implications from the pickup tasks.
%
% In this file defined predicates:
% duration(getProduct(_), D)     How long it takes to retreive the product from the machine, once the robot has arrived.
% free(L, gt)                    The machine side L is freed in time step GT.
% inUse(L, GT)                   The machine side L is in use for time step GT.
% location(getProduct(_), L)     Where the retreival will take place.
% pickup(R, P, L)                The robot R picked up product P at location L.
% points(getProduct(_), P)       How many points are awarded for the retreival.
% possible(R, getProduct(_), GT) If robot R can perform the retreival task at GT.
% spawn(getProduct(_), T)        The get product tasks are possible after T, see state.lp
% task(getProduct(L, R, S))      The task to fetch a product from the base station L by robot R. S is to enumerate how
%                                often a robot is able to perform this task.
%
% Additionally used predicates:
% begin(R, T, GT)                Defined in state.lp
% end(R, T, GT)                  Defined in state.lp
% failure(R, T, GT)              Defined in state.lp
% getLocation(M, L)              Defined in facts.lp
% holding(R, P, GT)              Defined in robots.lp
% storing(M, P, GT)              Defined in machines.lp


#program ourTeam(t).

task(getProduct(L, R, S))                                :- robot(R), getLocation(_, L), S = 1..@maxGetProduct().
points(getProduct(L, R, S), 0)                           :- task(getProduct(L, R, S)).
duration(getProduct(L, R, S), @getProductTaskDuration()) :- task(getProduct(L, R, S)).
location(getProduct(L, R, S), L)                         :- task(getProduct(L, R, S)).
spawn(getProduct(L, R, S), T)                            :- task(getProduct(L, R, S)), deliveryTimes(T, _).

#program transition(gt).
inUse(L, gt) :- begin(R, getProduct(L, R, S), gt), not end(R, getProduct(L, R, S), gt).
free(L, gt)  :- end(R, getProduct(L, R, _), gt).

possible(R, getProduct(L, R, S), gt) :- task(getProduct(L, R, S)), not holding(R, _, gt), getLocation(M, L),
										storing(M, P, gt), not inUse(L, gt).

{ pickUp(R, P, M, gt)
  } = 1 :- end(R, getProduct(L, R, S), gt), not failure(getProduct(L, R, S), gt), getLocation(M, L), storing(M, P, gt).

