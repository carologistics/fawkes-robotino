% Defined programs:
% ourTeam(t)                     Setup the prepare cs tasks, that are possible, as soon as we know the special
%                                locations.
% transition(gt)                 Update implications from the prepare tasks.
%
% In this file defined predicates:
% csPrepared(M, GT)              Also defined in machines.lp
% duration(prepareCS(_), D)      How long it takes to prepare the cap station, once the robot has arrived.
% free(L, gt)                    The machine side L is freed in time step GT.
% genProduct(R, I, B, GT)        The robot R generated a new product with index I and base color B at GT.
% inUse(L, GT)                   The machine side L is in use for time step GT.
% location(prepareCS(_), L)      Where the preparation will take place.
% points(prepareCS(_), P)        How many points are awarded for the preparation.
% possible(R, prepareCS(_), GT)  If robot R can perform the preparation task at GT.
% process(M, P, GT)              The machine M starts the processing of the product P at GT.
% spawn(prepareCS(_), T)         The prepare CS tasks are possible after T, see state.lp
% task(prepareCS(L, R, S))       The task to prepare the cap station L by robot R. S is to enumerate how often a robot
%                                is able to perform this task. Preparing means get a dummy product from the shelf and
%                                insert it in the cap station for cap unmountment.
%
% Additionally used predicates:
% begin(R, T, GT)                Defined in state.lp
% capLocation(L)                 Defined in facts.lp
% deliveryTimes(L, U)            Defined in facts.lp
% end(R, T, GT)                  Defined in state.lp
% failure(R, T, GT)              Defined in state.lp
% holding(R, P, GT)              Defined in robots.lp
% nextID(R, I, GT)               Defined in products.lp
% storing(M, P, GT)              Defined in machines.lp


#program ourTeam(t).

% This means each robot can prepare each CS up to 3 times, even when one robot is doing all the preparations we can
% produce at least 3 products.
#const maxPrepCS = 3.

task(prepareCS(L, R, S))                               :- robot(R), capLocation(_, L), S = 1..maxPrepCS.
points(prepareCS(L, R, S), 3)                          :- task(prepareCS(L, R, S)).
duration(prepareCS(L, R, S), @prepareCSTaskDuration()) :- task(prepareCS(L, R, S)).
location(prepareCS(L, R, S), L)	                       :- task(prepareCS(L, R, S)).
spawn(prepareCS(L, R, S), T)                           :- task(prepareCS(L, R, S)), deliveryTimes(T, _).

#program transition(gt).
inUse(L, gt) :- begin(R, prepareCS(L, R, S), gt), not end(R, prepareCS(L, R, S), gt).
free(L, gt)  :- end(R, prepareCS(L, R, _), gt).

possible(R, prepareCS(L, R, S), gt) :- task(prepareCS(L, R, S)), holding(R, none, gt), capLocation(M, L),
									   not csPrepared(M, gt), storing(M, none, gt), not inUse(L, gt).

{ csPrepared(M, gt);
  genProduct(R, I, "TRANSPARENT", gt);
  process(M, product(R, I), gt)
  } = 3 :- end(R, prepareCS(L, R, S), gt), not failure(prepareCS(L, R, S), gt), nextID(R, I, gt), capLocation(M, L).

