% Defined programs:
% addRobot(r, l, gt, t)          Adds the robot r with location l at gt as tick t.
% killRobot(r, gt, t)            Removes the robot r at gt as tick t.
% tranisiton(gt)                 Updates the robots state.
%
% In this file defined predicates:
% addRobot(R, L, GT, T)          The robot R with location L was added at GT as tick T.
% holding(R, P, GT)              Robot R holds product P at GT.
% killRobot(R, GT, T)            The robot R was removed at GT as tick T.
% location(R, L, GT)             The robot R is at L in GT.
% robot(R, GT)                   The robot R is alive at GT.
% usedTick(R, T)                 The tick T for robot R was used.
%
% Additionally used predicates:
% location(L)                    Defined in facts.lp
% pickUp(R, P, L, GT)            Defined in some task descriptions.
% putDown(R, P, L, GT)           Defined in some task descriptions.
% robot(R)                       Defined in facts.lp


#program addRobot(r, l, gt, t).
addRobot(r, l, gt, t).
usedTick(r, t).

#program removeRobot(r, gt, t).
killRobot(r, gt, t).
usedTick(r, t).

#program transition(gt).

#external addRobot(R, L, gt, T) : robot(R), T = 0..@maxTicks(), location(L), not usedTick(R, T).
#external killRobot(R, gt, T)   : robot(R), T = 0..@maxTicks(), not usedTick(R, T).

robot(R, gt) :- addRobot(R, _, gt, _), not killRobot(R, gt, _).
robot(R, gt) :- addRobot(R, _, gt, T), killRobot(R, gt, T-1).
robot(R, gt) :- robot(R, gt-1), not killRobot(R, gt, _).

%Todo: What happens with the product if a robot is taken out with a product in his gripper?
holding(R, P, gt) :- holding(R, P, gt-1), not putDown(R, P, _, gt).
holding(R, P, gt) :- pickUp(R, P, _, gt).

location(R, L, gt) :- addRobot(R, L, gt, _), not killRobot(R, gt, _).
location(R, L, gt) :- addRobot(R, L, gt, T), killRobot(R, gt, T-1).

