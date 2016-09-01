% zoneToExplore(Z, GT): Zone Z isn't explored at gametime GT.

zoneToExplore(Z, 0) :- zone(Z).

% Introduce task "explore(Z)" and every robot is capable of doing it at any time (the zone must be explored).
task(explore(Z)) :- zone(Z).
location(explore(Z), Z) :- zone(Z). %todo: What location?
duration(explore(Z), explorationTaskDuration) :- zone(Z).

#program transition(gt). %Extra explorationTransition and ground only when something to explore?
zoneToExplore(Z, gt) :- zoneToExplore(Z, gt - 1), not zoneExplored(Z, gt).
possible(R, explore(Z), GT) :- robot(R, GT), zone(Z), zoneToExplore(Z, GT). %todo: Ist zone(Z) überflüssig/hinderlich?

#program skipExploration.
zoneExplored(Z, 1) :- zone(Z).

#program zoneExplored(z, gt).
zoneExplored(z, gt).

#program foundMachine(m, gt).
foundMachine(m, gt).
everythingExplored :- 6 { foundMachine(M, gt) : teamMachine(_, M) }.
