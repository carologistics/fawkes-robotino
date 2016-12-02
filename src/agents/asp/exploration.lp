#program explore.

% Introduce task "explore(Z)" and every robot is capable of doing it at any time (the zone must be explored).
task(explore(Z)) :- zone(Z).
location(z(Z)) :- zone(Z).
location(explore(Z), z(Z)) :- zone(Z).
duration(explore(Z), @explorationTaskDuration()) :- zone(Z).
points(explore(Z), explorationPoints) :- zone(Z).

#program explorationTransition(gt).

#external toBeDone(explore(Z), gt) : zone(Z).
#external robot(R, gt)             : robot(R).

possible(R, explore(Z), gt) :- robot(R, gt), toBeDone(explore(Z), gt).

#program zoneExplored(z, gt).
success(explore(z), gt).

#program zoneToExplore(z, gt).
toBeDone(explore(z), gt) :- zone(z).

