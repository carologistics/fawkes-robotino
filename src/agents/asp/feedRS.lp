% Everything concerning the feedRS task.
% The task feeds the product a robot holds into the ring station for the advanced colors.

% In ourTeam the locations are generated.
#program ourTeam(t).

% This means each robot can feed each RS up to 10 times.
#const maxFeedRS = 10.

task(feedRS(L, R, S))                            :- robot(R), ringLocation(_, L), S = 1..maxFeedRS.
points(feedRS(L, R, S), 2)                       :- task(feedRS(L, R, S)).
duration(feedRS(L, R, S), @feedRSTaskDuration()) :- task(feedRS(L, R, S)).
location(feedRS(L, R, S), L)	                 :- task(feedRS(L, R, S)).
spawn(feedRS(L, R, S), T)                        :- task(feedRS(L, R, S)), deliveryTimes(T, _).

#program transition(gt).
inUse(L, gt) :- begin(R, feedRS(L, R, S), gt), not end(R, feedRS(L, R, S), gt).
free(L, gt)  :- end(R, feedRS(L, R, _), gt).

possible(R, feedRS(L, R, S), gt) :- task(feedRS(L, R, S)), not holding(R, gt, none), ringLocation(M, L),
									ringStationAssignment(M, Color), ringColorCost(Color, Cost), ringCost(Cost),
									Cost > 0, not inUse(L, gt).

{ rsFed(M, gt);
  putDown(R, L, P, gt)
  } = 2 :- end(R, feedRS(L, R, S), gt), not failure(feedRS(L, R, S), gt), ringLocation(M, L), holding(R, P, gt).

