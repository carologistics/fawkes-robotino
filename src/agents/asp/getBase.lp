% Defined programs:
% ourTeam(t)                     Setup the get base tasks, that are possible, as soon as we know the special locations.
% transition(gt)                 Update implications from the pickup tasks.
%
% In this file defined predicates:
% duration(getBase(_), D)        How long it takes to retreive the base from the base station, once the robot has
%                                arrived.
% free(L, gt)                    The machine side L is freed in time step GT.
% genProduct(R, B, GT)           The robot R generated a new product with base color B at GT.
% inUse(L, GT)                   The machine side L is in use for time step GT.
% location(getBase(_), L)        Where the retreival will take place.
% pickup(R, P, L)                The robot R picked up product P at location L.
% points(getBase(_), P)          How many points are awarded for the retreival.
% possible(R, getBase(_), GT)    If robot R can perform the retreival task at GT.
% spawn(getBase(_), T)           The get base tasks are possible after T, see state.lp
% task(getBase(L, R, B, S))      The task to get a new base of color Bfrom the base station L by robot R. S is to
%                                enumerate how often a robot is able to perform this task.
%
% Additionally used predicates:
% baseLocation(L)                Defined in facts.lp
% begin(R, T, GT)                Defined in state.lp
% deliveryTimes(L, U)            Defined in facts.lp
% end(R, T, GT)                  Defined in state.lp
% failure(R, T, GT)              Defined in state.lp
% holding(R, P, GT)              Defined in robots.lp


#program ourTeam(t).

% Each robot can get each color from each side 8 times, that should suffice.
% The tasks are spawned immediatly, but with a negative score, so the maximize won't choose them just for fun.
task(getBase(L, R, B, S))                         :- robot(R), baseLocation(L), S = 1..@maxGetBase(), baseColor(B).
points(getBase(L, R, B, S), -1)                   :- task(getBase(L, R, B, S)).
duration(getBase(L, R, B, S), @getTaskDuration()) :- task(getBase(L, R, B, S)).
location(getBase(L, R, B, S), L)                  :- task(getBase(L, R, B, S)).
spawnTask(getBase(L, R, B, S), T)                 :- task(getBase(L, R, B, S)), deliveryTimes(T, _).

#program transition(gt).
inUse(L, gt) :- begin(R, getBase(_, R, _, _), gt), not end(R, getBase(_, R, _, _), gt), baseLocation(L).
free(L, gt)  :- end(R, getBase(_, R, _, _), gt), baseLocation(L).

possible(R, getBase(L, R, B, S), gt) :- task(getBase(L, R, B, S)), not holding(R, _, gt), not inUse(L, gt).

{ genProduct(R, B, gt);
  pickUp(R, product(R, gt), L)
  } = 2 :- end(R, getBase(L, R, B, S), gt), not failure(getBase(L, R, B, S), gt).


