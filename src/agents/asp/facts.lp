zone(1..24).

% todo: Do we need the coordinates in the planer? I hope not! the duration/4 should help with it.
% Coordinates in cm
%zoneCenter(Z, X, Y) :- zone(Z), Z <= 12, X = ((Z - 1) / 4) * 200 + 100, Y = ((Z - 1) \ 4) * 150 + 75.
%zoneCenter(Z, X, Y) :- zone(Z), Z >= 13, zoneCenter(Z - 12, X2, Y), X = 0 - X2.

%zoneManhattanDistance(Z1, Z2, D) :- zone(Z1), zone(Z2), zoneCenter(Z1, X1, Y1), zoneCenter(Z2, X2, Y2), D = |X1 - X2| + |Y1 - Y2|.

% todo: Calculate real distance, possible with python or lua.
%zoneSquareDistance(Z1, Z2, D) :- zone(Z1), zone(Z2), zoneCenter(Z1, X1, Y1), zoneCenter(Z2, X2, Y2), D = |X1 - X2| ** 2 + |Y1 - Y2| ** 2.
%zoneDistance(Z1, Z2, D) :-zoneManhattanDistance(Z1, Z2, D).

% Machines as string, so they can be in upper case.
machine("BS").
machine("CS1").
machine("CS2").
machine("RS1").
machine("RS2").
machine("DS").

team("C").
team("M").

% Ground with the team, creates e.g. teamMachine("C", "BS"). The C++ part can use string concatenation to create "C-BS".
#program ourTeam(t).
%todo This or an external?
teamMachine(t, M) :- team(t), machine(M).

