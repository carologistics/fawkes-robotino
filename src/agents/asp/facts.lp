%---------------------------------------------------------------------------
%  Copyright  2016-2017  Bjoern Schaepers
%  Licensed under GPLv2+ license, cf. LICENSE file
%---------------------------------------------------------------------------
% Defined programs:
% base                           The static facts.
% setDriveDuration(l1, l2, d)    Sets driveDuration, see there for parameters.
% setRingInfo(col, cost, m)      Sets the cost for ring color col and assigns it to machine m.
% startProduction                The production phase has started, so we can activate the tasks for that phase.
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
% horizon(X)                     How far in the future we want to look, i.e. planning covers time steps [0,X).
% location(m(T, M, S))           The navgraph locations for all machine sides.
% machine(M)                     All machines (without team info).
% needsFillment(M)               The RS M has at least one color which needs extra bases to be filled. (Task feedRS.)
% order(O)                       All orders O.
% productionStarted              The production phase has started.
% quantity(Q)                    All possible quantities for orders Q.
% ringColor(C)                   All ring colors.
% ringColorCost(Col, Cost)       The mapping from ring color Col to its corresponding cost Cost.
% ringLocation(M, L)             The location for the input of the ring stations, both the slide and the "normal" input.
% ringStationAssignment(M, C)    The mapping from ring color C to the correct ring station M. Defined as external.
% ringCost(C)                    All possible additional costs for a ring.
% ringStation(M)                 All the ring stations (without team info).
% robot(R)                       All possible available robots.
% side(S)                        All sides of a machine.
% zone(Z)                        All zones in the game.
%
% Additionally used predicates:
% -- none --                     Or else this wouldn't be facts, or would they?


robot(@robots()).

horizon(@horizon()).
zone(1..24).

% How many orders there are at most an their quantity.
% If we try to add an order with a higher number or quantity it will not be considered.
order(1..@maxOrders()).
quantity(1..@maxQuantity()).

deliveryTimes(@minDeliveryTime(), @maxDeliveryTime()).

#external currentTime(GT) : GT = 0..@maxDeliveryTime().
#external productionStarted.

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

location("ins-out").

baseColor("RED"; "BLACK"; "SILVER"). %There exists also transparent bases, but they are not used for production.
capColor("BLACK"; "GREY").
ringColor("GREEN"; "ORANGE"; "YELLOW"; "BLUE").
ringCost(0..2).

capStationAssignment("CS1", @capColor(1)).
capStationAssignment("CS2", @capColor(2)).

#external ringStationAssignment(M, RingColor) : ringStation(M), ringColor(RingColor).
#external ringColorCost(RingColor, Cost)      : ringColor(RingColor), ringCost(Cost).
#external needsFillment(M)                    : ringStation(M).

#program ourTeam(t).

#external setDriveDuration(L1, L2, D) : location(L1), location(L2), L1 != L2, D = 1..@maxDriveDuration().
driveDuration(L1, L2, D) :- setDriveDuration(L1, L2, D).
driveDuration(L1, L2, D) :- setDriveDuration(L2, L1, D).
driveDuration(L, L, 0) :- location(L).

location(m(t, M, S)) :- baseStation(M),     side(S).
location(m(t, M, S)) :- capStation(M),      side(S).
location(m(t, M, S)) :- ringStation(M),     side(S).
location(m(t, M, S)) :- deliveryStation(M), S = "I". % The output side of the delivery station is not used in the game.

baseLocation(m(t, M, S)) :- baseStation(M), side(S).

getLocation(m(t, M, "O")) :- capStation(M).
getLocation(m(t, M, "O")) :- ringStation(M).

deliverLocation(m(t, M, "I")) :- deliveryStation(M).

capLocation(m(t, M, "I")) :- capStation(M).

ringLocation(m(t, M, "I")) :- ringStation(M).

locationToMachineMap(m(t, M, S), M) :- side(S), machine(M).

#program setDriveDuration(l1, l2, d).
setDriveDuration(l1, l2, d).

#program setRingInfo(col, cost, m).
ringStationAssignment(m, col).

ringColorCost(col, cost).

needsFillment(m) :- cost > 0.

#program startProduction.
productionStarted.
