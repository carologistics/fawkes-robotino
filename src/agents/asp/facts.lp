% Machines as string, so they can be in upper case.
machine("BS"; "CS1"; "CS2"; "RS1"; "RS2"; "DS").

% Machine sides.
side("I"; "O").

team("C"; "M").

% Valid locations are:
% m(T, M, S) The machine sides, e.g. m("C", "BS", "I") for the input side of the cyan base machine.
% z(Z) The zones, defined in exploration.lp
location(m(T, M, S)) :- machine(M), team(T), side(S).

% Ground with the team, creates e.g. teamMachine("C", "BS"). The C++ part can use string concatenation to create "C-BS".
#program ourTeam(t).
teamMachine(t, M) :- team(t), machine(M).
