% The planning horizon, how far (regarding gametime) we should plan.
#external horizon(h).

% Given from other files:
% task(T): There is a task T.
% location(T, L): The task T is located at L.
% duration(T, D): The task T will take D time units.
% points(T, P): The task T will award P points if executed successfully.
% possible(R, T, GT): If the robot R can begin with task T at gametime GT.
%
% The basic task idea is: (GT is measured in time units.)
% begin(R, T, GTb): Robot R begins task T at gametime GTb.
% end(R, T, GTe): Robot R ends with task T at gametime GTe.
% doing(R, T, GT, D): Robot R is doing task T at gametime GT, with GT in [GTb, GTe). And we estimate still D time units for doing so.
% possible(R, T, GT): Task T is doable by robot R at GT. E.g. he is holding the needed workpiece.
%
% success(T, GT): Task T is successful executed.
% toBeDone(T, GT): Task T is yet to be done.
%
% duration(R, T, GT, D): Task T should be done in D seconds from robot R at gametime GT. This includes the time for getting there.
% update(R, T, GT, D): Update the time estimation for robot R on Task T at GT to D time units.
%
% location(R, GT, L): The robot R is at gametime GT at location L.

% Called for every gametime we like to consider. %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#program transition(gt).

#external update(R, T, gt, D) : robot(R, gt), task(T), D = 1..maxDuration.

% Can start at max one task and only if not doing something else right now.
{ begin(R, T, gt) : task(T) } 1 :- robot(R, gt), toBeDone(T, gt), not doing(R, T2, gt, _), task(T2), T != T2.

% Initialize and update doing.
doing(R, T, gt, D) :- begin(R, T, gt), duration(R, T, gt, D).
%doing(R, T, gt, D) :- doing(R, T, gt-1, D+1), D > 1, not update(R, T, gt, _).
%doing(R, T, gt, D) :- update(R, T, gt, D).

% Setting end.
%end(R, T, gt) :- doing(R, T, gt-1, 1), not update(R, T, gt, _).

% Only one robot per task.
%:- { doing(R, T, gt, _) : robot(R, gt) } != 1, task(T).

% Calculate complete duration per robot.
duration(R, T, gt, D) :- robot(R, gt), task(T), duration(T, D1), location(R, RL, gt), location(T, TL), driveDuration(RL, TL, D2), D = D1 + D2.

% Called to update the time estimation for a robot. %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#program update(r, t, gt, d).
update(r, t, gt, d).

