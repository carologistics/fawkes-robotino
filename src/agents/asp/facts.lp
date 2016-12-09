% Machines as string, so they can be in upper case.
baseStation("BS").
capStation("CS1"; "CS2").
ringStation("RS1"; "RS2").
deliveryStation("DS").

machine(M) :- baseStation(M).
machine(M) :- capStation(M).
machine(M) :- ringStation(M).
machine(M) :- deliveryStation(M).

% Machine sides.
side("I"; "O").

team("C"; "M").

% Valid locations are:
% m(T, M, S) The machine sides, e.g. m("C", "BS", "I") for the input side of the cyan base machine.
% z(Z) The zones, defined in exploration.lp
location(m(T, M, S)) :- baseStation(M),     team(T), side(S).
location(m(T, M, S)) :- capStation(M),      team(T), side(S).
location(m(T, M, S)) :- ringStation(M),     team(T), side(S).
location(m(T, M, S)) :- deliveryStation(M), team(T), S = "I".

baseColor("RED"; "BLACK"; "SILVER"). %There exists also transparent bases, but they are not used for production.
capColor("BLACK"; "GREY").
ringColor("GREEN"; "ORANGE"; "YELLOW"; "BLUE").
ringCost(0..2).

% How many "new" products a robot can spawn, spawning happens when a new base is fetched or a CS is prepared.
productIndex(1..30).
nextID(R, 1, 0) :- robot(R).

capStationAssignment("CS1", @capColor(1)).
capStationAssignment("CS2", @capColor(2)).

#external ringStationAssignment(M, RingColor) : ringStation(M), ringColor(RingColor).
#external ringColorCost(RingColor, Cost)      : ringColor(RingColor), ringCost(Cost).

% Ground with the team, create only as few as possible special locations.
#program ourTeam(t).
baseLocation(m(t, M, S)) :- baseStation(M), side(S).

getLocation(m(t, M, "O")) :- capStation(M).
getLocation(m(t, M, "O")) :- ringStation(M).

bringLocation(m(t, M, "I")) :- deliveryStation(M).
bringLocation(m(t, M, "I")) :- capStation(M).
bringLocation(m(t, M, "I")) :- ringStation(M).

capLocation(M, m(t, M, "I")) :- capStation(M).

slideLocation(m(t, M, "I")) :- ringStation(M).

