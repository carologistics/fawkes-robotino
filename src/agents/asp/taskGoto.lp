% Defined programs:
% ourTeam(t)                     Setup the goto tasks, that are possible, as soon as we know the special locations.
%
% In this file defined predicates:
% inUse(L, R, GT)                The machine side L is in use by robot R for time step GT.
% possible(R, goto(_), GT)       If robot R can perform the goto task at GT. (Yes he can.)
% task(goto(L))                  The robot goes to location L.
% taskDuration(goto(_), D)       How long the task is.
% taskLocation(goto(_), L)       Where the goto will end.
% toBeDone(goto(_), GT)          The task is possible.
%
% Additionally used predicates:
% doing(R, T, D, GT)             Defined in state.lp
% end(R, T, GT)                  Defined in state.lp
% location(L)                    Defined in facts.lp
% robot(R)                       Defined in facts.lp


#program ourTeam(t).

gotoTask(goto(L))        :- location(L).
taskLocation(goto(L), L) :- gotoTask(goto(L)).
task(T)                  :- gotoTask(T).
% The duration is 0, because the task duration says how long the task takes, once the robot has reached the location.
% But the sole purpose of the goto task is to go to a location, so its duration is equal to the drive duration. These
% calculations are done in the state.lp
taskDuration(T, 0)       :- gotoTask(T).
% The goto task is never really finished, because it has nothing to do with the orders or so.
toBeDone(T, GT)          :- gotoTask(T), GT = 0..@horizon()-1.

inUse(L, R, GT) :- doing(R, T, _, GT), gotoTask(T), taskLocation(T, L).
inUse(L, R, GT) :- end(R, T, GT), gotoTask(T), taskLocation(T, L).

possible(R, T, GT) :- gotoTask(T), robot(R), GT = 0..@horizon()-1.
