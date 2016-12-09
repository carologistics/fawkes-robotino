% Everything concerning the delivery task.
% The task delivers a fiished product to the delivery station.

% In ourTeam the locations are generated.
#program ourTeam(t).

task(deliver(L, O, Q))                             :- order(O), quantity(Q), deliverLocation(L).
points(deliver(L, O, Q), 20)                       :- task(deliver(L, O, Q)).
duration(deliver(L, O, Q), @deliverTaskDuration()) :- task(deliver(L, O, Q)).
location(deliver(L, O, Q), L)	                   :- task(deliver(L, O, Q)).

task(lateDeliver(L, O, Q))                             :- order(O), quantity(Q), deliverLocation(L).
points(lateDeliver(L, O, Q), 5)                        :- task(lateDeliver(L, O, Q)).
duration(lateDeliver(L, O, Q), @deliverTaskDuration()) :- task(lateDeliver(L, O, Q)).
location(lateDeliver(L, O, Q), L)	                   :- task(lateDeliver(L, O, Q)).

#program transition(gt).
inUse(L, gt) :- begin(R, deliver(L, O, Q), gt), not end(R, deliver(L, O, Q), gt).
free(L, gt)  :- end(_, deliver(L, _, _), gt).

inUse(L, gt) :- begin(R, lateDeliver(L, O, Q), gt), not end(R, lateDeliver(L, O, Q), gt).
free(L, gt)  :- end(_, lateDeliver(L, _, _), gt).

possible(R, deliver(L, O, Q), gt) :- task(deliver(L, O, Q)), holding(R, gt, P), not inUse(L, gt), base(P, B),
									 base(O, B), ringOne(P, R1, gt), ring(O, 1, R1), ring(O, 2, R2), ringTwo(P, R2, gt),
									 ringThree(P, R3, gt), ring(O, 2, R3), cap(P, C, gt), cap(O, C),
									 location(R, RL, gt), driveDuration(RL, L, DD), duration(deliver(L, O, Q), TD),
									 D = TD + DD, D < E, orderTimeframe(O, _, E).

possible(R, lateDeliver(L, O, Q), gt) :- task(lateDeliver(L, O, Q)), holding(R, gt, P), not inUse(L, gt), base(P, B),
										 base(O, B), ringOne(P, R1, gt), ring(O, 1, R1), ring(O, 2, R2),
										 ringTwo(P, R2, gt), ringThree(P, R3, gt), ring(O, 2, R3), cap(P, C, gt),
										 cap(O, C).

{ putDown(R, L, P, gt);
  delivered(P, gt)
  } = 2 :- end(R, deliver(L, O, Q), gt), not failure(deliver(L, O, Q), gt), deliverLocation(L), holding(R, gt, P).

{ putDown(R, L, P, gt);
  delivered(P, gt)
  } = 2 :- end(R, lateDeliver(L, O, Q), gt), not failure(lateDeliver(L, O, Q), gt), deliverLocation(L), holding(R, gt, P).

