% In this file defined predicates:
% availableRobot(R)              The robot R is available for planning.
% holding(R, P, GT)              Robot R holds product P at GT.
% robotLocation(R, L, GT)        The robot R is at L in GT.
%
% Additionally used predicates:
% horizon(GT)                    Defined in facts.lp
% location(L)                    Defined in facts.lp
% pickUp(R, P, GT)               Defined in some task descriptions.
% product(P)                     Defined in product.lp
% putDown(R, GT)                 Defined in some task descriptions.
% robot(R)                       Defined in facts.lp

#external availableRobot(R) : robot(R).
#external holding(R, P, 0) : robot(R), product(P).

#program ourTeam(t).

#external robotLocation(R, L, 0) : robot(R), location(L).

holding(R, P, GT) :- holding(R, P, GT-1), not putDown(R, GT), not horizon(GT).
holding(R, P, GT) :- pickUp(R, P, GT), not horizon(GT).

