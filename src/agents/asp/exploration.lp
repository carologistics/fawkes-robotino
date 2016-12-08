% This file only gets grounded when there is at least one machine to explore.
% After all (6) Machines are found Clingo is reset and restarted without this file.
% Because of that this external statement is located here, if we know all machines we can add the driveDurations as normal fact and don't rely on #external.
#external driveDuration(L1, L2, D) : location(L1), location(L2), D = 0..@maxDriveDuration().

zone(1..24).

% Introduce task "explore(Z)" and every robot is capable of doing it at any time (the zone must be explored).
task(explore(Z)) :- zone(Z).
location(z(Z)) :- zone(Z).
location(explore(Z), z(Z)) :- zone(Z).
duration(explore(Z), @explorationTaskDuration()) :- zone(Z).
points(explore(Z), 100) :- zone(Z).

#program transition(gt).

possible(R, explore(Z), gt) :- robot(R, gt), toBeDone(explore(Z), gt).

#program zoneExplored(z, gt).
success(explore(z), gt).

#program zoneToExplore(z, gt).
toBeDone(explore(z), gt).

