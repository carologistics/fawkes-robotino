% Defined programs:
% begin(r, t, gt)                Fixes the beginning of a task by a robot.
% failure(t, gt)                 Reenabled a task, because its execution was a failure.
% past(gt)                       Fixes the past. We don't expect any new updates for gt.
% past(gt, r)                    Fixes also the past, but will only be grounded for a robot which hasn't send any
%                                messages. (Not counting the "I'm alive messages".)
% tranisiton(gt)                 Updates the task handling.
% update(r, t, gt, d)            Updates the time estimation.
%
% In this file defined predicates:
% begin(R, T, GT)                The execution of task T by robot R starts in GT.
% doing(R, T, GT, D)             The remaining estimation for the execution of task T by robot R at GT is D.
% duration(R, T, GT, D)          If robot R starts T in GT we estimate it will take him D time units to execute the
%                                task, this includes the time to drive to the location of the task.
% end(R, T, GT)                  Robot R has finished its execution of T in GT.
% failure(T, GT)                 The task T was unsuccessful executed at GT.
% location(R, L, GT)             The robot R is at location L in GT, that means particually he is not moving (the whole
%                                time unit).
% toBeDone(T, GT)                Task T has still to be done at GT.
% update(R, T, GT, D)            The task duration estimation for task T was at GT updated to D from robot R.
%
% Additionally used predicates:
% driveDuration(L1, L2, D)       Defined in facts.lp
% location(T, L)                 Defined in all task descriptions.
% possible(R, T, GT)             Defined in all task descriptions.
% robot(R)                       Defined in facts.lp
% spawnTask(T)                   Defined in multiple task descriptions and order.lp
% task(T)                        Defined in all task descriptions.


#program transition(gt).

#external update(R, T, gt, D) : robot(R), task(T), D = 0..@maxTaskDuration().
#external spawnTask(T, gt)    : task(T).
#external failure(T, gt)      : task(T).

% Can start at max one task and only if not doing something else right now.
{ begin(R, T, gt) : toBeDone(T, gt), possible(R, T, gt) } 1 :- location(R, _, gt).

% Initialize and update doing.
doing(R, T, gt, D) :- begin(R, T, gt), duration(R, T, gt, D).
doing(R, T, gt, D) :- doing(R, T, gt-1, D+1), D > 0, not update(R, T, gt, _).
doing(R, T, gt, D) :- update(R, T, gt, D), D != 0.

% Setting end.
end(R, T, gt) :- doing(R, T, gt-1, 1), not update(R, T, gt, D), D = 1..@maxTaskDuration().
end(R, T, gt) :- update(R, T, gt, 0).

% Update robot location.
% Set robot location to location of the task it just finished. (This seems reasonable.)
location(R, L, gt) :- end(R, T, gt), location(T, L), not setLocation(R, _, gt, _).
% In the moment a robot starts a task we do not store its location.
location(R, L, gt) :- robot(R, gt), location(R, L, gt-1), not begin(R, _, gt-1), not setLocation(R, _, gt, _).

% Max one robot per task.
:- 2 { doing(R, T, gt, _) : robot(R, gt) }, toBeDone(T, gt).
% Max one task per robot.
:- 2 { doing(R, T, gt, _) : toBeDone(T, gt) }, robot(R, gt).

% Calculate complete duration per robot.
duration(R, T, gt, D) :- possible(R, T, gt), duration(T, TD), location(T, TL), location(R, RL, gt), driveDuration(RL, TL, DD), D = TD + DD.

% Update Task status.
toBeDone(T, gt) :- spawnTask(T, gt).
toBeDone(T, gt) :- toBeDone(T, gt-1), not end(_, T, gt).
toBeDone(T, gt) :- failure(T, gt).

#program update(r, t, gt, d).
update(r, t, gt, d).

#program begun(r, t, gt).
:- not begin(r, t, gt), robot(r, gt).
:- begin(R, t, _), R != r, robot(r, gt).

#program failure(t, gt).
failure(t, gt).

#program past(r, gt).
:- begin(r, _, gt).
:- update(r, _, gt, _).

#program past(gt).
%:- horizon(gt).
:- failure(T, gt), task(T).
