% The planning horizon, how far (regarding gametime) we should plan.
#external horizon(h).

% The basic task idea is:
% begin(R, T, GTb): Robot R begins task T at gametime GTb.
% end(R, T, GTe): Robot R ends with task T at gametime GTe.
% doing(R, T, GT, D): Robot R is doing task T at gametime GT, with GT in [GTb, GTe]. And we estimate still D seconds for doing so.
% possible(R, T, GT): Task T is doable by robot R at GT. E.g. he is holding the needed workpiece.
%
% success(T): Task T is successful executed.
% toBeDone(T): Task T is yet to be done.
%
% duration(T, D): Task T should be done in D seconds.
% duration(R, T, GT, D): Task T should be done in D seconds from robot R at gametime GT. This includes the time for getting there.
% update(R, T, GT, D): Update the time estimation for robot R on Task T at GT to D seconds.
% location(T, L): The task T is to be done at location L.
% location(R, GT, L): The robot R is at gametime GT at location L.

driveDuration(robot, Z, 5) :- zone(Z). %Todo: This should be a function call!
%And this is not needed then!
%location(loc("Z", Z)) :- zone(Z).
%location(robot).

% Called for every gametime we like to consider. %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#program transition(gt).

% A robot is only doing one task at a given time.
{ doing(R, T, gt) : task(T) } 1 :- robot(R, gt).

% Robot R can start task T at gt.
:- begin(R, T, gt), not possible(R, T, gt).

location(R, gt, robot) :- robot(R, gt). %todo: Get actual location of the robot!

doing(R, T, gt, D) :- begin(R, T, gt), duration(R, T, gt, D).
doing(R, T, gt, D) :- doing(R, T, gt - 1, D + 1), not update(R, T, gt, _), D >= 0.
doing(R, T, gt, D) :- doing(R, T, gt - 1, _), update(R, T, gt, D).

doing(R, T, gt) :- doing(R, T, gt, _).

end(R, T, gt) :- doing(R, T, gt, 0).

% Update the task duration per robot.
duration(R, T, gt, D) :- robot(R, gt), task(T), duration(T, TD), location(R, gt, RL), location(T, TL), driveDuration(RL, TL, DD), D = TD + DD.

% Called to update the time estimation for a robot. %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#program update(r, t, gt, d).
update(r, t, gt, d).

%#program updateGametime(gt, t).
%gameTime(gt, t).

