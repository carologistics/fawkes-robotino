% Introduce task "explore(Z)" and every robot is capable of doing it at any time (the zone must be explored).
task(explore(Z)) :- zone(Z).
location(explore(Z), Z) :- zone(Z). %todo: What location?
duration(explore(Z), explorationTaskDuration) :- zone(Z).
points(explore(Z), explorationPoints) :- zone(Z).

everythingExplored :- 6 { foundMachine(M, _) : teamMachine(_, M) }.

#program transition(gt). %Extra explorationTransition and ground only when something to explore?
zoneToExplore(Z, gt) :- zoneToExplore(Z, gt - 1), not zoneExplored(Z, gt).
possible(R, explore(Z), gt) :- robot(R, gt), zone(Z), zoneToExplore(Z, gt). %todo: Ist zone(Z) überflüssig/hinderlich?

#program skipExploration.
zoneExplored(Z, 1) :- zone(Z).

#program zoneExplored(z, gt).
zoneExplored(z, gt).

#program foundMachine(m, gt).
foundMachine(m, gt).

#program zoneToExplore(z, gt).
% zoneToExplore(Z, GT): Zone Z isn't explored at gametime GT.
zoneToExplore(z, gt) :- zone(z).
toBeDone(explore(z), gt) :- zone(z).

