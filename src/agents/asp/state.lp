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

#external update(R, T, gt, D) : robot(R), task(T), D = 0..@maxTaskDuration().
#external toBeDone(T, gt)     : task(T).
#external success(T, gt)      : task(T).

% Can start at max one task and only if not doing something else right now.
{ begin(R, T, gt) : toBeDone(T, gt) } 1 :- location(R, _, gt), possible(R, T, gt).

% Initialize and update doing.
doing(R, T, gt, D) :- begin(R, T, gt), duration(R, T, gt, D).
doing(R, T, gt, D) :- doing(R, T, gt-1, D+1), D > 0, not update(R, T, gt, _).
doing(R, T, gt, D) :- update(R, T, gt, D).

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
duration(R, T, gt, D) :- toBeDone(T, gt), duration(T, TD), location(T, TL), location(R, RL, gt), driveDuration(RL, TL, DD), D = TD + DD.

% Update Task status.
toBeDone(T, gt) :- toBeDone(T, gt-1), not end(_, T, gt).

% The planning horizon, how far (regarding gametime) we should plan.
#external horizon(gt).

% Our goal.
#maximize{ P@2,T,GT : begin(_, T, GT), points(T, P), horizon(gt) }.

#heuristic begin(_,T,GT). [GT,false]
%#heuristic begin(_,T,GT). [-1,sign]

%#minimize{ GT@1,T : begin(_, T, GT), task(T) }.

% Called to update the time estimation for a robot. %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#program update(r, t, gt, d).
update(r, t, gt, d).
:- begin(r, T, gt), T != t.

#program begun(r, t, gt).
:- not begin(r, t, gt), robot(r, gt).
:- begin(R, t, _), R != r, robot(r, gt).

#program ended(r, t, gt).
end(r, t, gt).

#program past(r, gt).
:- begin(r, _, gt).
:- update(r, _, gt, _).

#program past(gt).
:- horizon(gt).
