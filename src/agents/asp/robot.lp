#program transition(gt).
robot(X, gt) :- robot(X, gt-1), not killRobot(X, gt).

#program addRobot(x, gt).
:- robot(x, gt).

#program removeRobot(x, gt).
killRobot(x, gt).
