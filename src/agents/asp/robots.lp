% In this file defined predicates:
% availableRobot(R)              The robot R is available for planning.
% holding(R, P, GT)              Robot R holds product P at GT.
% robotLocation(R, L, GT)        The robot R is at L in GT.
%
% Additionally used predicates:
% location(L)                    Defined in facts.lp
% pickUp(R, P, L, GT)            Defined in some task descriptions.
% product(P)                     Defined in product.lp
% putDown(R, P,  GT)             Defined in some task descriptions.
% robot(R)                       Defined in facts.lp

#external availableRobot(R) : robot(R).
#external holding(R, P, 0) : robot(R), product(P).

#program ourTeam(t).

#external robotLocation(R, L, 0) : robot(R), location(L).

%Todo: What happens with the product if a robot is taken out with a product in his gripper?
holding(R, P, GT) :- holding(R, P, GT-1), not putDown(R, P, GT), not horizon(GT).
holding(R, P, GT) :- pickUp(R, P, _, GT), not horizon(GT).

