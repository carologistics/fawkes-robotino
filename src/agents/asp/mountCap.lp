% Everything concerning the mountCap task.
% The task delivers a product to the cap station after it is prepared to mount the cap.

% In ourTeam the locations are generated.
#program ourTeam(t).

task(mountCap(L, O, Q))                              :- order(O), quantity(Q), capLocation(_, L).
points(mountCap(L, O, Q), 10)                        :- task(mountCap(L, O, Q)).
duration(mountCap(L, O, Q), @mountCapTaskDuration()) :- task(mountCap(L, O, Q)).
location(mountCap(L, O, Q), L)	                     :- task(mountCap(L, O, Q)).

#program transition(gt).
inUse(L, gt) :- begin(R, mountCap(L, O, Q), gt), not end(R, mountCap(L, O, Q), gt).
free(L, gt)  :- end(_, mountCap(L, _, _), gt).

possible(R, mountCap(L, O, Q), gt) :- task(mountCap(L, O, Q)), holding(R, gt, P), capLocation(M, L),
									  csPrepared(M, gt), storing(M, gt, none), not inUse(L, gt), base(P, B), base(O, B),
									  ringOne(P, R1, gt), ring(O, 1, R1), ring(O, 2, R2), ringTwo(P, R2, gt),
									  ringThree(P, R3, gt), ring(O, 2, R3), cap(P, none, gt).

{ putDown(R, L, P, gt);
  mountedCap(M, P, gt);
  produced(M, gt, P)
  } = 3 :- end(R, mountCap(L, O, Q), gt), not failure(mountCap(L, O, Q), gt), capLocation(M, L), holding(R, gt, P).

