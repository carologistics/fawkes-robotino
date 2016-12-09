% Create the tasks when we know which team we are, since we can then use the special locations.
#program ourTeam(t).

task(getProduct(L, O, Q, S))               :- getLocation(L),   order(O), quantitiy(Q), step(S).
task(bringProdct(L, O, Q, S))              :- bringLocation(L), order(O), quantitiy(Q), step(S).
task(getProductFromShelf(L, Sh, O, Q, St)) :- shelfLocation(L), order(O), quantitiy(Q), step(St), shelf(Sh).
task(bringProdctSlide(L, O, Q, S))         :- slideLocation(L), order(O), quantitiy(Q), step(S).

#external nextTask(T1, T2) : task(T1), task(T2).

% Set the the ring color assignments and the cost for a color.
#program setRingColor(m, color, cost).
ringStationAssignment(m, color).
:- ringStationAssignment(M, color), ringStation(M), M != m.

ringColorCost(color, cost).
:- ringColorCost(color, Cost), ringCost(Cost), Cost != cost.

% Adds the drive durations once we know the whole map.
#program setDuration(l1, l2, d).
driveDuration(l1, l2, d).

#program transition(gt).
% Update inUse, we block a machine one time step longer than actually used, this is because of the time resolution.
inUse(L, gt) :- inUse(L, gt-1), not free(L, gt-1).

