% Defined programs:
% ourTeam(t)                     Updates the states of machines.
%
% In this file defined predicates:
% broken(M, D, GT)               The machine M is broken for at least D time steps.
% brokenLocation(L, GT)          The location L is broken.
% csPrepared(M, GT)              If the cap station M is prepared to mount a cap in GT.
% inUse(L, R, GT)                The location L is blocked by robot R in GT.
% processing(M, P, D, GT)        The machine M processes product P for the next D time steps from GT on.
% rsFillState(M, S, GT)          Ring station M has S additional bases loaded in GT.
% storing(M, P, GT)              Machine M stores product P in GT.
%
% Additionally used predicates:
% baseStation(M)                 Defined in facts.lp
% capMounted(M, P, GT)           Defined in mountCap.lp
% capStation(M)                  Defined in facts.lp
% deliveryStation(M)             Defined in facts.lp
% deliveryTimes(B, E)            Defined in facts.lp
% getLocation(L)                 Defined in facts.lp
% horizon(GT)                    Defined in facts.lp
% inUse(L, R, GT)                Defined in the task descriptions.
% locationToMachineMap(L, M)     Defined in facts.lp
% pickUp(R, P, GT)               Defined in some tasks descriptions.
% process(M, P, GT)              Defined in some tasks descriptions.
% product(P)                     Defined in product.lp
% ringColorCost(Col, Cost)       Defined in facts.lp
% ringCost(C)                    Defined in facts.lp
% ringMounted(M, P, C, GT)       Defined in mountRing.lp
% ringStation(M)                 Defined in facts.lp
% robotLocation(R, L, GT)        Defined in robots.lp and state.lp
% rsFed(M, GT)                   Defined in feedRS.lp

#external broken(M, D, 0) : capStation(M), D = 1..@clingoToASP(60).
#external broken(M, D, 0) : ringStation(M), D = 1..@clingoToASP(60).
#external csPrepared(M, 0) : capStation(M).
#external rsFillState(M, S, 0) : ringStation(M), S = 0..3.
#external processing(M, P, D, 0) : machine(M), product(P), D = 1..@maxWorkingDuration(), not baseStation(M).
#external storing(M, P, 0) : capStation(M), product(P).
#external storing(M, P, 0) : ringStation(M), product(P).

#program ourTeam(t).
% A location is in use, when a robot stands there.
inUse(L, R, GT) :- robotLocation(R, L, GT).

% Each location is used by at most one robot.
:- inUse(L, R1, GT), inUse(L, R2, GT), R1 != R2, L != "ins-out".
% Benchmark them against each other!
%:- 2 { inUse(L, R, GT) : robot(R) }, L != "ins-out".

broken(M, D, GT)      :- broken(M, D+1, GT-1), not horizon(GT), D > 0.
brokenLocation(L, GT) :- broken(M, _, GT), locationToMachineMap(L, M).

storing(M, P, GT) :- storing(M, P, GT-1), not pickUp(_, P, GT), not horizon(GT).
storing(M, P, GT) :- processing(M, P, 1, GT-1), not deliveryStation(M), not horizon(GT).

processing(M, P, D, GT) :- process(M, P, GT), D = @machineWorkingDuration(M).
processing(M, P, D, GT) :- processing(M, P, D+1, GT-1), not horizon(GT), D != 0, not broken(M, _, GT).
processing(M, P, D, GT) :- processing(M, P, D, GT-1), not horizon(GT), broken(M, _, GT).

csPrepared(M, GT) :- csPrepared(M, GT-1), not capMounted(M, _, GT), not horizon(GT).

rsFillState(M, S,      GT) :- rsFillState(M, S, GT-1), not rsFed(M, GT), not ringMounted(M, _, _, GT), not horizon(GT).
rsFillState(M, S+1,    GT) :- rsFillState(M, S, GT-1), rsFed(M, GT), not horizon(GT), S <= 2.
rsFillState(M, S-Cost, GT) :- rsFillState(M, S, GT-1), ringMounted(M, _, Col, GT), ringCost(Cost),
							  ringColorCost(Col, Cost), not horizon(GT), S >= Cost.
