#program addRobot(x, gt, t).
robot(x, gt, t).

#program removeRobot(x, gt, t).
killRobot(x, gt, t).

#program transition(gt).
robot(X, gt) :- robot(X, gt, _), not killRobot(X, gt, _).
robot(X, gt) :- robot(X, gt-1), not killRobot(X, gt, _).

