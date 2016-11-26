% @todo Kann das irgendwie sinnvoll umgangen werden? Oder muss man echt auf Zahlen zurück fallen?
robot("Tuvok";"T'Pol";"Spock").

#program addRobot(r, gt, t).
robot(r, gt, t).

#program removeRobot(r, gt, t).
killRobot(r, gt, t).

#program setRobotLocation(r, gt, x, y).
location(r, p(x, y), gt, set).

#program transition(gt).

#external robot(R, gt, T)               : robot(R), T = 1..maxTicks.
#external killRobot(R, gt, T)           : robot(R), T = 1..maxTicks.
#external location(R, p(X, Y), gt, set) : robot(R), X = mapMinX..mapMaxX, Y = mapMinY..mapMaxY.

robot(R, gt) :- robot(R, gt, _), not killRobot(R, gt, _).
robot(R, gt) :- robot(R, gt-1), not killRobot(R, gt, _).

% The set location has the highest priority, it is extracted from the robots beacon.
% The estimated location is what the planner thinks a robot has after a task is done.
% Else just keep the current location, if the robot isn't driving.
% If the robot is driving, we do not update it's location wihtin ASP, only new locations from the beacon are taken.

location(R, L, gt) :- location(R, L, gt, set).
location(R, L, gt) :- not location(R, _, gt, set), location(R, L, gt, estimated).
location(R, L, gt) :- not location(R, _, gt, _), not driving(R, gt), location(R, L, gt-1).

