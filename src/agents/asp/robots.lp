% Defined programs:
% addRobot(r, gt, t)             Adds the robot r at gt as tick t.
% killRobot(r, gt, t)            Removes the robot r at gt as tick t.
% setLocation(r, l, gt, t)       Sets the location of robot r to l at gt as tick t.
% tranisiton(gt)                 Updates the robots state.
%
% In this file defined predicates:
% holding(R, P, GT)              Robot R holds product P at GT.
% killRobot(R, GT, T)            The robot R was removed at GT as tick T.
% location(R, L, GT)             The robot R is at L in GT.
% robot(R, GT)                   The robot R is alive at GT.
% robot(R, GT, T)                The robot R was added at GT as tick T.
% setLocation(R, L, GT, T)       The location of robot R was set to L at GT as tick T.
%
% Additionally used predicates:
% location(L)                    Defined in facts.lp
% pickUp(R, P, L, GT)            Defined in some task descriptions.
% putDown(R, P, L, GT)           Defined in some task descriptions.
% robot(R)                       Defined in facts.lp


#program addRobot(r, gt, t).
robot(r, gt, t).

#program removeRobot(r, gt, t).
killRobot(r, gt, t).

#program setRobotLocation(r, l, gt, t).
setLocation(r, l, gt, t).

#program transition(gt).

#external robot(R, gt, T)          : robot(R), T = 0..@maxTicks().
#external killRobot(R, gt, T)      : robot(R), T = 0..@maxTicks().
#external setLocation(R, L, gt, T) : robot(R), location(L), T = 0..@maxTicks().

robot(R, gt) :- robot(R, gt, _), not killRobot(R, gt, _).
robot(R, gt) :- robot(R, gt, T1), killRobot(R, gt, T2), T1 > T2.
robot(R, gt) :- robot(R, gt-1), not killRobot(R, gt, _).

%Todo: What happens with the product if a robot is taken out with a product in his gripper?
holding(R, none, gt) :- robot(R, gt, _), not killRobot(R, gt, _).
holding(R, none, gt) :- robot(R, gt, T1), killRobot(R, gt, T2), T1 > T2.
holding(R, P   , gt) :- holding(R, P, gt-1), not killRobot(R, gt, _), not putDown(R, _, _, gt).
holding(R, P   , gt) :- not killRobot(R, gt, _), pickUp(R, P, _, gt).
holding(R, none, gt) :- putDown(R, _, _, gt).

location(R, L, gt) :- setLocation(R, L, gt, _), not killRobot(R, gt, _).
location(R, L, gt) :- setLocation(R, L, gt, T1), killRobot(R, gt, T2), T1 > T2.

