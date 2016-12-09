% How many orders there are at most an their quantity.
% If we try to add an order with a higher number or quantity it will not be considered.
order(1..@maxOrders()).
quantity(1..@maxQuantity()).

deliveryTimes(@minDeliveryTime(), @maxDeliveryTime()).

#external base(O, B) : order(O), baseColor(B).
#external cap(O, C)  : order(O), capColor(C).
#external orderTimeframe(O, B, E) : order(O), deliveryTimes(L, U), B = L..U, E = L..U, B < E.

#program newOrder(order, quantity, b, c, r1, r2, r3, deliveryBegin, deliveryEnd).
orderTimeframe(order, deliveryBegin, deliveryEnd).
base(order, b).
:- base(order, B), baseColor(B), B != b.
cap(order, c).
:- cap(order, C), capColor(C), C != c.
ringOne(order, r1).
:- ringOne(order, R), ringColor(R), R != r1.
ringTwo(order, r2).
:- ringTwo(order, R), ringColor(R), R != r2.
ringThree(order, r3).
:- ringThree(order, R), ringColor(R), R != r3.
