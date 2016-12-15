% Defined programs:
% ourTeam(t)                     Setup the feed rs tasks, that are possible, as soon as we know the special locations.
% transition(gt)                 Update implications from the feed tasks.
%
% In this file defined predicates:
% rsFed(M, GT)                   The ring station M has received an additional base for the advanced colors in GT.
% duration(feedRS(_), D)         How long it takes to put the product onto the ring stations slide, once the robot has
%                                arrived.
% free(L, gt)                    The machine side L is freed in time step GT.
% inUse(L, GT)                   The machine side L is in use for time step GT.
% location(feedRS(_), L)         Where the feeding will take place.
% points(feedRS(_), P)           How many points are awarded for the feeding.
% possible(R, feedRS(_), GT)     If robot R can perform the feed task at GT.
% putDown(R, L, P, GT)           Robot R puts product P down at location L in GT.
% spawnTask(feedRS(_), T)        The feed RS tasks are possible after T, see state.lp
% task(feedRS(L, R, S))          The task to feed a product into ring station L by robot R. S is to enumerate how often
%                                a robot is able to perform this task.
%
% Additionally used predicates:
% begin(R, T, GT)                Defined in state.lp
% end(R, T, GT)                  Defined in state.lp
% failure(R, T, GT)              Defined in state.lp
% holding(R, P, GT)              Defined in robots.lp
% ringColorCost(Col, Cost)       Defined in facts.lp
% ringCost(C)                    Defined in facts.lp
% ringLocation(M, L)             Defined in facts.lp
% ringStationAssignment(M, C)    Defined in facts.lp


#program ourTeam(t).

task(feedRS(L, R, S))                            :- robot(R), ringLocation(_, L), S = 1..@maxFeedRS().
points(feedRS(L, R, S), 2)                       :- task(feedRS(L, R, S)).
duration(feedRS(L, R, S), @feedRSTaskDuration()) :- task(feedRS(L, R, S)).
location(feedRS(L, R, S), L)	                 :- task(feedRS(L, R, S)).
spawnTask(feedRS(L, R, S), T)                    :- task(feedRS(L, R, S)), deliveryTimes(T, _).

#program transition(gt).
inUse(L, gt) :- begin(R, feedRS(L, R, S), gt).
free(L, gt)  :- end(R, feedRS(L, R, _), gt).

% Maybe get more sophisticated and only allow transparent bases for the last fillment.
possible(R, feedRS(L, R, S), gt) :- task(feedRS(L, R, S)), holding(R, P, gt), ringLocation(M, L),
									ringStationAssignment(M, Color), ringColorCost(Color, Cost), ringCost(Cost),
									Cost > 0, not inUse(L, gt), rsFillState(M, FS, gt), FS < 3,
									gt >= U, deliveryTimes(U, _), robot(R, gt).

{ rsFed(M, gt);
  putDown(R, L, P, gt)
  } = 2 :- end(R, feedRS(L, R, S), gt), not failure(feedRS(L, R, S), gt), ringLocation(M, L), holding(R, P, gt).

