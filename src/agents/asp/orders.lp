% Defined programs:
% newOrder(o, q, b, c, r1, r2,   A new order has arrived, its number is o, its quantity q. The ordered products are of
%          r3, delBegin,         the form (b, r1, r2, r3, c), but the rs can be "none". The order is posted at gt and
%          delEnd, gt)           its delivery windows is [delBegin, delEnd].
%
% In this file defined predicates:
% base(O, B)                     The base color for order O is B.
% cap(O, C)                      The cap color for order O is C.
% orderDeadline(O, D)            The time frame for the delivery of the order O ends at D.
% ring(O, R, C)                  The color of the R-th ring for order O is C.
% spawnTask(T, GT)               Spawns the task for a order.
%
% Additionally used predicates:
% baseColor(B)                   Defined in facts.lp
% capColor(C)                    Defined in facts.lp
% capLocation(M, L)              Defined in facts.lp
% capStationAssignment(M, C)     Defined in facts.lp
% deliverLocation(L)             Defined in facts.lp
% deliveryTimes(L, U)            Defined in facts.lp
% order(O)                       Defined in facts.lp
% ringColor(B)                   Defined in facts.lp
% ringLocation(M, L)             Defined in facts.lp
% ringStationAssignment(M, R)    Defined in facts.lp


#external base(O, B) : order(O), baseColor(B).
#external cap(O, C)  : order(O), capColor(C).
#external ring(O, R, C) : order(O), R = 1..3, ringColor(C).
% This is too much, but we need this information.
%#external orderDeadline(O, D) : order(O), deliveryTimes(L, U), D = L..U.

#program newOrder(o, q, b, c, r1, r2, r3, deliveryBegin, deliveryEnd, gt).
%orderDeadline(o, deliveryEnd).
%:- orderDeadline(o, D), deliveryTimes(L, U), D >= L, D <= U, D != deliveryEnd.
base(o, b).
:- base(o, B), baseColor(B), B != b.
cap(o, c).
:- cap(o, C), capColor(C), C != c.
ring(o, 1, r1).
:- ring(o, 1, R), ringColor(R), R != r1.
ring(o, 2, r2).
:- ring(o, 2, R), ringColor(R), R != r2.
ring(o, 3, r3).
:- ring(o, 3, R), ringColor(R), R != r3.

spawnTask(mountCap(L, o, Q), gt) :- Q = 1..q, capLocation(M, L), capStationAssignment(M, c).
spawnTask(mountRing(L, o, Q, 1), gt) :- Q = 1..q, ringLocation(M, L), ringStationAssignment(M, r1), r1 != "none".
spawnTask(mountRing(L, o, Q, 2), gt) :- Q = 1..q, ringLocation(M, L), ringStationAssignment(M, r2), r2 != "none".
spawnTask(mountRing(L, o, Q, 3), gt) :- Q = 1..q, ringLocation(M, L), ringStationAssignment(M, r3), r3 != "none".
spawnTask(deliver(L, o, Q), deliveryBegin)   :- Q = 1..q, deliverLocation(L).
spawnTask(lateDeliver(L, o, Q), deliveryEnd) :- Q = 1..q, deliverLocation(L).
