% Defined programs:
% ourTeam(t)                     Setup the feed rs tasks, that are possible, as soon as we know the special locations.
%
% In this file defined predicates:
% rsFed(M, GT)                   The ring station M has received an additional base for the advanced colors in GT.
% free(L, GT)                    The machine side L is freed in time step GT.
% inUse(L, R, GT)                The machine side L is in use by robot R for time step GT.
% points(feedRS(_), P)           How many points are awarded for the feeding.
% possible(R, feedRS(_), GT)     If robot R can perform the feed task at GT.
% putDown(R, L, P, GT)           Robot R puts product P down at location L in GT.
% task(feedRS(L, S))             The task to feed a product into ring station L, S is to enumerate how often a robot is
%                                able to perform this task.
% taskDuration(feedRS(_), D)     How long it takes to put the product onto the ring stations slide, once the robot has
%                                arrived.
% taskLocation(feedRS(_), L)     Where the feeding will take place.
% toBeDone(feedRS(_), 0)         The feed RS tasks are possible.
%
% Additionally used predicates:
% begin(R, T, GT)                Defined in state.lp
% end(R, T, GT)                  Defined in state.lp
% holding(R, P, GT)              Defined in robots.lp
% productionStarted              Defined in facts.lp
% ringColorCost(Col, Cost)       Defined in facts.lp
% ringCost(C)                    Defined in facts.lp
% ringLocation(M, L)             Defined in facts.lp
% ringStationAssignment(M, C)    Defined in facts.lp
% robot(R)                       Defined in facts.lp


#program ourTeam(t).

feedRSTask(feedRS(L, S))               :- ringLocation(_, L), S = 1..@maxFeedRS().
task(T)                                :- feedRSTask(T).
points(T, 1)                           :- feedRSTask(T).
taskDuration(T, @feedRSTaskDuration()) :- feedRSTask(T).
taskLocation(feedRS(L, S), L)          :- task(feedRS(L, S)).
toBeDone(T, 0)                         :- feedRSTask(T), productionStarted.

inUse(L, R, GT) :- begin(R, feedRS(L, _), GT).
free(L, GT) :- end(R, feedRS(L, _), GT).

% Maybe get more sophisticated and only allow transparent bases for the last fillment.
possible(R, feedRS(L, S), GT) :- task(feedRS(L, S)), robot(R), holding(R, P, GT), ringLocation(M, L),
								 ringStationAssignment(M, Color), ringColorCost(Color, Cost), ringCost(Cost),
								 Cost > 0, rsFillState(M, FS, GT), FS < 3, GT = 0..@horizon()-1.

rsFedByRobot(R, M, P, GT) :- end(R, feedRS(L, _), GT), ringLocation(M, L), holding(R, P, GT).

rsFed(M, GT)      :- rsFedByRobot(_, M, _, GT).
putDown(R, P, GT) :- rsFedByRobot(R, _, P, GT).

