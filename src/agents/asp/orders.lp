% How many orders there are at most an their quantity.
% If we try to add an order with a higher number or quantity it will not be considered.
order(1..@maxOrders()).
quantitiy(1..@maxQuantity()).

deliveryTimes(@minDeliveryTime(), @maxDeliveryTime()).

#external base(O, B) : order(O), baseColor(B).
#external cap(O, C)  : order(O), capColor(C).
#external orderTimeframe(O, B, E) : order(O), deliveryTimes(L, U), B = L..U, E = L..U.

% Common Information about orders.
#program newOrder(order, quantity, base, cap, deliveryBegin, deliveryEnd).
orderTimeframe(order, deliveryBegin, deliveryEnd).
base(order, base).
:- base(order, B), baseColor(B), B != b.
cap(order, cap).
:- cap(order, C), capColor(C), C != c.

% An order for a C0.
#program newOrder(order, quantity).
