zone(1..24).

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
% @todo This or an external?
teamMachine(t, M) :- team(t), machine(M).

