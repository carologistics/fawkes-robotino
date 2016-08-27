zoneToExplore(Z, 0) :- zone(Z).

#program explorationTransition(gt).
zoneToExplore(Z, gt) :- zoneToExplore(Z, gt - 1), not zoneExplored(Z, gt).

#program skipExploration.
zoneExplored(Z, 1) :- zone(Z).

#program zoneExplored(z, gt).
zoneExplored(z, gt).

#program foundMachine(m, gt).
foundMachine(m, gt).
everythingExplored :- 6 { foundMachine(M, gt) : teamMachine(_, M) }.
