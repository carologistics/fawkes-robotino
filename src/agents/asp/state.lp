%---------------------------------------------------------------------------
%  Copyright  2016-2017  Bjoern Schaepers
%  Licensed under GPLv2+ license, cf. LICENSE file
%---------------------------------------------------------------------------
% In this file defined predicates:
% begin(R, T, GT)                The execution of task T by robot R starts in GT.
% doing(R, T, D, GT)             The remaining estimation for the execution of task T by robot R at GT is D.
% end(R, T, GT)                  Robot R has finished its execution of T in GT.
% fullDuration(T, L, D)          The duration it would take to execute task T, starting from location L is D.
% robotDoing(R, T, D)            The robot R is doing task T in gt 0 for an estimated duration of D.
% robotLocation(R, L, GT)        The robot R is at location L in GT, that means particually he is not moving (the whole
%                                time unit).
% toBeDone(T, GT)                Task T has still to be done at GT.
%
% Additionally used predicates:
% brokenLocation(L, GT)          Defined in machines.lp
% driveDuration(L1, L2, D)       Defined in facts.lp
% horizon(H)                     Defined in facts.lp
% ignoreBrokenTask(T)            Defined in some task descriptions.
% oneShotTask(T)                 Defined in some task descriptions.
% possible(R, T, GT)             Defined in all task descriptions.
% robot(R)                       Defined in facts.lp
% task(T)                        Defined in all task descriptions.
% taskLocation(T, L)             Defined in all task descriptions.


#program ourTeam(t).

#external robotDoing(R, T, D) : robot(R), task(T), taskDuration(T, TD), DD = @maxDriveDuration(), D = 1..TD+DD.

fullDuration(T, L, DD + TD) :- taskDuration(T, TD), taskLocation(T, TL), driveDuration(L, TL, DD).

% Can start at max one task per robot.
{ begin(R, T, GT) : task(T) } 1 :- robot(R), horizon(H), GT = 0..H-1.

% Only start a task, if we have a location (a.k.a. not doing something else).
:- begin(R, _, GT), not robotLocation(R, _, GT).
% Only start tasks that are not done.
:- begin(_, T, GT), not toBeDone(T, GT).
% Only start tasks that are possible.
:- begin(R, T, GT), not possible(R, T, GT).
% Start tasks as early as possible.
:- begin(R, T, GT),  possible(R, T, GT-1), robotLocation(R, _, GT-1), toBeDone(T, GT-1).

begin(R, T, GT) :- robotLocation(R, L, GT), getLocation(L), possible(R, T, GT), taskLocation(T, L).

% Initialize and update doing.
doing(R, T, D, GT) :- begin(R, T, GT), robotLocation(R, L, GT), fullDuration(T, L, D).
% A task ignoring the broken state of a machine just advances one step.
doing(R, T, D, GT) :- doing(R, T, D+1, GT-1), D > 0, not horizon(GT), ignoreBrokenTask(T).
% The not ignoring tasks have to check wether the machine is broken/down or not and only advance if not.
doing(R, T, D, GT) :- doing(R, T, D+1, GT-1), D > 0, not horizon(GT), not ignoreBrokenTask(T), taskLocation(T, L), not brokenLocation(L, GT).
doing(R, T, D, GT) :- doing(R, T, D,   GT-1),        not horizon(GT), not ignoreBrokenTask(T), taskLocation(T, L),     brokenLocation(L, GT).
% External initializing of the doing predicate.
doing(R, T, D, 0) :- robotDoing(R, T, D).

% Max one robot per task.
:- 2 { doing(R, T, _, GT) : robot(R) }, task(T), horizon(H), GT = 0..H-1.
% Max one task per robot.
:- 2 { doing(R, T, _, GT) : task(T) }, robot(R), horizon(H), GT = 0..H-1.

% Setting end, we have to do the same checks as in doing regarding brokenLocation and ignoreBrokenTask.
end(R, T, GT) :- doing(R, T, 1, GT-1), not horizon(GT), ignoreBrokenTask(T).
end(R, T, GT) :- doing(R, T, 1, GT-1), not horizon(GT), not ignoreBrokenTask(T), taskLocation(T, L), not brokenLocation(L, GT).

% Update robot location.
% Set robot location to location of the task it just finished.
robotLocation(R, L, GT) :- end(R, T, GT), taskLocation(T, L).
% In the moment a robot starts a task we do not store its location.
robotLocation(R, L, GT) :- robotLocation(R, L, GT-1), not begin(R, _, GT-1), not horizon(GT).

% Update Task status.
toBeDone(T, GT) :- toBeDone(T, GT-1), not end(_, T, GT), not horizon(GT), oneShotTask(T).
