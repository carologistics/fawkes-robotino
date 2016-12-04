robot("Tuvok";"T'Pol";"Spock").

#program addRobot(r, gt, t).
robot(r, gt, t).

#program removeRobot(r, gt, t).
killRobot(r, gt, t).

#program setRobotLocation(r, l, gt).
location(r, l, gt).

#program transition(gt).

#external robot(R, gt, T)     : robot(R), T = 0..maxTicks.
#external killRobot(R, gt, T) : robot(R), T = 0..maxTicks.

robot(R, gt) :- robot(R, gt, _), not killRobot(R, gt, _).
robot(R, gt) :- robot(R, gt, T1), killRobot(R, gt, T2), T1 > T2.
robot(R, gt) :- robot(R, gt-1),  not killRobot(R, gt, _).
