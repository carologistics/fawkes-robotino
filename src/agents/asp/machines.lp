% Defined programs:
% base                           The static facts, regarding machines.
%
% In this file defined predicates:
% csPrepared(M, GT)              If the cap station M is prepared to mount a cap in GT.
% inUse(L, R, GT)                The machine side L is in use by robot R for time step GT.
% rsFillState(M, S, GT)          Ring station M has S additional bases loaded in GT.
% storing(M, P, GT)              Machine M stores product P in GT.
%
% Additionally used predicates:
% capMounted(M, P, GT)           Defined in mountCap.lp
% deliveryTimes(B, E)            Defined in facts.lp
% free(L, GT)                    Defined in the task descriptions.
% pickUp(R, P, M, GT)            Defined in some tasks descriptions.
% process(M, P, GT)              Defined in some tasks descriptions.
% product(P)                     Defined in product.lp
% ringColorCost(Col, Cost)       Defined in facts.lp
% ringCost(C)                    Defined in facts.lp
% ringMounted(M, P, C, GT)       Defined in mountRing.lp
% ringStation(M)                 Defined in facts.lp
% rsFed(M, GT)                   Defined in feedRS.lp

#external csPrepared(M, 0) : capStation(M).
#external rsFillState(M, S, 0) : ringStation(M), S = 0..3.
#external storing(M, P, 0) : capStation(M), product(P).
#external storing(M, P, 0) : ringStation(M), product(P).

#program ourTeam(t).
% Update inUse, we block a machine one time step longer than actually used, this is because of the time resolution.
inUse(L, R, GT) :- inUse(L, R, GT-1), not free(L, GT-1), not horizon(GT).

% Each location is used by at most one robot.
:- inUse(L, R1, GT), inUse(L, R2, GT), R1 != R2.
% Benchmark them against each other!
%:- 2 { inUse(L, R, GT) : robot(R) }.

storing(M, P, GT) :- storing(M, P, GT-1), not pickUp(_, P, M, GT), not horizon(GT).
storing(M, P, GT) :- process(M, P, GT). %TODO: Consider processing time.

csPrepared(M, GT) :- csPrepared(M, GT-1), not capMounted(M, _, GT), not horizon(GT).

rsFillState(M, S,      GT) :- rsFillState(M, S, GT-1), not rsFed(M, GT), not ringMounted(M, _, _, GT), not horizon(GT).
rsFillState(M, S+1,    GT) :- rsFillState(M, S, GT-1), rsFed(M, GT), not horizon(GT), S <= 2.
rsFillState(M, S-Cost, GT) :- rsFillState(M, S, GT-1), ringMounted(M, _, Col, GT), ringCost(Cost),
							  ringColorCost(Col, Cost), not horizon(GT), S >= Cost.
