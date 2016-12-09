% Everything concerning the mountRing task.
% The task prepares the ring station to mount a ring and delivers a product for mountment.

% In ourTeam the locations are generated.
#program ourTeam(t).

% For the points we cheat a little bit. The points are only awarded for the last ring, but this is difficult to model
% so we award for every mount and subtract the before awarded from the points for the next ring. So we don't get 80
% points for the 3rd ring of a C3, but only 50, because we got 10 for the 1st and 20 for the 2nd, in total 80.
task(mountRing(L, O, Q, R))                               :- order(O), quantity(Q), capLocation(_, L), R = 1..3.
points(mountRing(L, O, Q, 1), 10)                         :- task(mountRing(L, O, Q, 1)).
points(mountRing(L, O, Q, 2), 20)                         :- task(mountRing(L, O, Q, 2)).
points(mountRing(L, O, Q, 3), 50)                         :- task(mountRing(L, O, Q, 3)).
duration(mountRing(L, O, Q, R), @mountRingTaskDuration()) :- task(mountRing(L, O, Q, R)).
location(mountRing(L, O, Q, R), L)	                      :- task(mountRing(L, O, Q, R)).

#program transition(gt).
inUse(L, gt) :- begin(R, mountRing(L, O, Q, Ring), gt), not end(R, mountRing(L, O, Q, Ring), gt).
free(L, gt)  :- end(_, mountRing(L, _, _, _), gt).

possible(R, mountRing(L, O, Q, 1), gt) :- task(mountRing(L, O, Q, 1)), holding(R, gt, P), ringLocation(M, L),
										  rsFillState(M, FS, gt), storing(M, gt, none), not inUse(L, gt),
										  base(P, B), base(O, B), ringOne(P, none, gt), cap(P, none, gt),
										  ringColorCost(Color, Cost), Cost <= FS.

possible(R, mountRing(L, O, Q, 2), gt) :- task(mountRing(L, O, Q, 1)), holding(R, gt, P), ringLocation(M, L),
										  rsFillState(M, FS, gt), storing(M, gt, none), not inUse(L, gt),
										  base(P, B), base(O, B), not ringOne(P, none, gt), ringTwo(P, none, gt),
										  cap(P, none, gt), ringColorCost(Color, Cost), Cost <= FS.

possible(R, mountRing(L, O, Q, 3), gt) :- task(mountRing(L, O, Q, 1)), holding(R, gt, P), ringLocation(M, L),
										  rsFillState(M, FS, gt), storing(M, gt, none), not inUse(L, gt),
										  base(P, B), base(O, B), not ringTwo(P, none, gt), ringThree(P, none, gt),
										  cap(P, none, gt), ringColorCost(Color, Cost), Cost <= FS.

{ putDown(R, L, P, gt);
  mountedRing(M, P, Color, gt);
  produced(M, gt, P)
  } = 3 :- end(R, mountRing(L, O, Q, Ring), gt), not failure(mountRing(L, O, Q, Ring), gt), ringLocation(M, L),
		   holding(R, gt, P), ring(O, Ring, Color).

