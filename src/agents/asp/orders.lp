% How many orders there are at most an their quantity.
% If we try to add an order with a higher number or quantity it will not be considered.
order(1..@maxOrders()).
quantity(1..@maxQuantity()).

deliveryTimes(@minDeliveryTime(), @maxDeliveryTime()).

#external base(O, B) : order(O), baseColor(B).
#external cap(O, C)  : order(O), capColor(C).
#external ring(O, R, C) : order(O), R = 1..3, ringColor(C).
#external orderTimeframe(O, B, E) : order(O), deliveryTimes(L, U), B = L..U, E = L..U, B < E.

#program newOrder(o, q, b, c, r1, r2, r3, deliveryBegin, deliveryEnd, gt).
orderTimeframe(order, deliveryBegin, deliveryEnd).
base(o, b).
:- base(order, B), baseColor(B), B != b.
cap(order, c).
:- cap(order, C), capColor(C), C != c.
ring(order, 1, r1).
:- ring(order, 1, R), ringColor(R), R != r1.
ring(order, 2, r2).
:- ring(order, 2, R), ringColor(R), R != r2.
ring(order, 3, r3).
:- ring(order, 3, R), ringColor(R), R != r3.

spawnTask(mountCap(L, o, Q), gt) :- Q = 1..q, capLocation(M, L), capStationAssignment(M, c).
spawnTask(mountRing(L, o, Q, 1), gt) :- Q = 1..q, ringLocation(M, L), ringStationAssignment(M, r1), r1 != none.
spawnTask(mountRing(L, o, Q, 2), gt) :- Q = 1..q, ringLocation(M, L), ringStationAssignment(M, r2), r2 != none.
spawnTask(mountRing(L, o, Q, 3), gt) :- Q = 1..q, ringLocation(M, L), ringStationAssignment(M, r3), r3 != none.
spawnTask(deliver(L, o, Q), deliveryBegin)   :- Q = 1..q, deliverLocation(L).
spawnTask(lateDeliver(L, o, Q), deliveryEnd) :- Q = 1..q, deliverLocation(L).
