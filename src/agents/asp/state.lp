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

% Called for every gametime we like to consider. %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#program transition(gt).

#external update(R, T, gt, D)    : robot(R, gt), task(T), D = 1..maxDuration.
#external toBeDone(T, gt)        : task(T).
#external success(T, gt)         : task(T).
#external driveDuration(R, L, D) : robot(R, gt), location(L), D = 1..@maxDriveDuration().

% Can start at max one task and only if not doing something else right now.
{ begin(R, T, gt) : task(T) } 1 :- robot(R, gt), possible(R, T, gt), toBeDone(T, gt).

% Initialize and update doing.
doing(R, T, gt, D) :- begin(R, T, gt), duration(R, T, gt, D).
doing(R, T, gt, D) :- doing(R, T, gt-1, D+1), D > 0, not update(R, T, gt, _).
doing(R, T, gt, D) :- update(R, T, gt, D).

% Setting end.
end(R, T, gt) :- doing(R, T, gt-1, 1), not update(R, T, gt, _).

% Max one robot per task.
:- 2 { doing(R, T, gt, _) : robot(R, gt) }, toBeDone(T, gt).
% Max one task per robot.
:- 2 { doing(R, T, gt, _) : toBeDone(T, gt) }, robot(R, gt).

% Calculate complete duration per robot.
duration(R, T, gt, D) :- robot(R, gt), task(T), duration(T, TD), location(T, TL), driveDuration(R, TL, DD), D = TD + DD.

% Update Task status.
toBeDone(T, gt) :- toBeDone(T, gt-1), not end(_, T, gt).

% The planning horizon, how far (regarding gametime) we should plan.
#external horizon(gt).

% Our goal.
#maximize{ P@2,T,GT : begin(_, T, GT), points(T, P), horizon(gt) }.

#minimize{ GT@1,T : begin(_, T, GT), task(T) }.

% Called to update the time estimation for a robot. %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#program update(r, t, gt, d).
update(r, t, gt, d).

