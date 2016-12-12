% Defined programs:
% base                           The static facts.
% setDriveDuration(l1, l2, d)    Sets driveDuration, see there for parameters.
% setRingInfo(col, cost, m)      Sets the cost for ring color col and assigns it to machine m.
% ourTeam(t)                     Sets the locations when we know our color. As cyan we are only interested in C-RS1-I
%                                for example. To keep the locations for task spawning small we define them in this
%                                program.
%
% In this file defined predicates:
% baseColor(C)                   All base colors for products.
% baseLocation(L)                The locations where a base can be retreived.
% baseStation(M)                 All the base stations (without team info).
% capColor(C)                    All cap colors.
% capLocation(M, L)              The locations L where the cap station get its input, for unmounting as well as for
%                                mounting and its machine M.
% capStation(M)                  All the cap stations (without team info).
% capStationAssignment(M, C)     The mapping from cap color C to the correct cap station M.
% deliverLocation(L)             The location L of our delivery station.
% deliveryStation(M)             All the delivery stations (without team info).
% driveDuration(L1, L2, D)       It will take D time units for a robot to drive from L1 to L2, not accounting for
%                                obstacles on its way.
% getLocation(M, L)              The locations L where a product can be picked up, with its machine M.
% location(m(T, M, S))           The navgraph locations for all machine sides.
% machine(M)                     All machines (without team info).
% order(O)                       All orders O.
% quantity(Q)                    All possible quantities for orders Q.
% ringColor(C)                   All ring colors.
% ringColorCost(Col, Cost)       The mapping from ring color Col to its corresponding cost Cost.
% ringLocation(M, L)             The location for the input of the ring stations, both the slide and the "normal" input.
% ringStationAssignment(M, C)    The mapping from ring color C to the correct ring station M. Defined as external.
% ringCost(C)                    All possible additional costs for a ring.
% ringStation(M)                 All the ring stations (without team info).
% robot(R)                       All possible available robots.
% side(S)                        All sides of a machine.
% team(T)                        All possible teams.
%
% Additionally used predicates:
% -- none --                     Or else this wouldn't be facts, or would they?


robot(@robots()).

% How many orders there are at most an their quantity.
% If we try to add an order with a higher number or quantity it will not be considered.
order(1..@maxOrders()).
quantity(1..@maxQuantity()).

deliveryTimes(@minDeliveryTime(), @maxDeliveryTime()).

% Machines as string, so they can be in upper case.
baseStation("BS").
capStation("CS1"; "CS2").
ringStation("RS1"; "RS2").
deliveryStation("DS").

machine(M) :- baseStation(M).
machine(M) :- capStation(M).
machine(M) :- ringStation(M).
machine(M) :- deliveryStation(M).

side("I"; "O").

team("C"; "M").

location(m(T, M, S)) :- baseStation(M),     team(T), side(S).
location(m(T, M, S)) :- capStation(M),      team(T), side(S).
location(m(T, M, S)) :- ringStation(M),     team(T), side(S).
location(m(T, M, S)) :- deliveryStation(M), team(T), side(S).

baseColor("RED"; "BLACK"; "SILVER"). %There exists also transparent bases, but they are not used for production.
capColor("BLACK"; "GREY").
ringColor("GREEN"; "ORANGE"; "YELLOW"; "BLUE").
ringCost(0..2).

capStationAssignment("CS1", @capColor(1)).
capStationAssignment("CS2", @capColor(2)).

#external ringStationAssignment(M, RingColor) : ringStation(M), ringColor(RingColor).
#external ringColorCost(RingColor, Cost)      : ringColor(RingColor), ringCost(Cost).

#program ourTeam(t).
baseLocation(m(t, M, S)) :- baseStation(M), side(S).

getLocation(M, m(t, M, "O")) :- capStation(M).
getLocation(M, m(t, M, "O")) :- ringStation(M).

deliverLocation(m(t, M, "I")) :- deliveryStation(M).

capLocation(M, m(t, M, "I")) :- capStation(M).

ringLocation(M, m(t, M, "I")) :- ringStation(M).

#program setDriveDuration(l1, l2, d).
driveDuration(l1, l2, d).

#program setRingInfo(col, cost, m).
ringStationAssignment(m, col).
:- ringStationAssignment(M, col), ringStation(M), M != m.

ringColorCost(col, cost).
:- ringColorCost(col, C), ringCost(C), C != cost.
