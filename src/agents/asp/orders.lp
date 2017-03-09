% Defined programs:
% newOrder(o, q, b, c, r1, r2,   A new order has arrived, its number is o, its quantity q. The ordered products are of
%          r3, delBegin,         the form (b, r1, r2, r3, c), but the rs can be "none". The order is posted at gt and
%          delEnd, gt)           its delivery windows is [delBegin, delEnd].
%
% In this file defined predicates:
% base(O, B)                     The base color for order O is B.
% cap(O, C)                      The cap color for order O is C.
% orderBegin(O, B)               The time frame for the delivery of the order O begins at B.
% orderDeadline(O, D)            The time frame for the delivery of the order O ends at D.
% ring(O, R, C)                  The color of the R-th ring for order O is C.
%
% Additionally used predicates:
% baseColor(B)                   Defined in facts.lp
% capColor(C)                    Defined in facts.lp
% deliveryTimes(L, U)            Defined in facts.lp
% order(O)                       Defined in facts.lp
% quantity(Q)                    Defined in facts.lp
% ringColor(B)                   Defined in facts.lp


#external base(O, B) : order(O), baseColor(B).
#external cap(O, C)  : order(O), capColor(C).
#external ring(O, R, C) : order(O), R = 1..3, ringColor(C).
#external orderBegin(O, B) : order(O), deliveryTimes(L, U), B = L..U.
#external orderDeadline(O, D) : order(O), deliveryTimes(L, U), D = L..U.

% todo: Add orderBegin and deadline for the last order after the game. 5 minutes overtime

#program newOrder(o, q, b, c, r1, r2, r3, deliveryBegin, deliveryEnd).
orderBegin(o, deliveryBegin).
orderDeadline(o, deliveryEnd).
base(o, b).
cap(o, c).
ring(o, 1, r1) :- r1 != "none".
ring(o, 2, r2) :- r2 != "none".
ring(o, 3, r3) :- r3 != "none".

