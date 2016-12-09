% Everythin concerning the getBase task.

% In outTeam the locations are generated.
#program ourTeam(t).

% Each robot can get each color from each side 8 times, that should suffice.
% The tasks are spawned immediatly, but with a negative score, so the maximize won't choose them just for fun.
task(getBase(L, R, B, S))                         :- robot(R), baseLocation(L), S = 1..8, baseColor(B).
points(getBase(L, R, B, S), -S)                   :- task(getBase(L, R, B, S)).
duration(getBase(L, R, B, S), @getTaskDuration()) :- task(getBase(L, R, B, S)).
location(getBase(L, R, B, S), L)                  :- task(getBase(L, R, B, S)).
spawnTask(getBase(L, R, B, S), T)                 :- task(getBase(L, R, B, S)), deliveryTimes(T, _).

#program transition(gt).
inUse(L, gt) :- begin(_, getBase(_, _, _, _), gt), not end(_, getBase(_, _, _, _), gt), baseLocation(L).
free(L, gt) :- end(_, getBase(_, _, _, _), gt), baseLocation(L).

possible(R, getBase(L, R, B, S), gt) :- task(getBase(L, R, B, S)), holding(R, gt, none), not inUse(L, gt).

{ genProduct(R, I, B);
  pickUp(R, product(R, I), L)
  } = 2 :- end(R, getBase(L, R, B, S), gt), not failure(getBase(L, R, B, S), gt), nextID(R, I, gt).


