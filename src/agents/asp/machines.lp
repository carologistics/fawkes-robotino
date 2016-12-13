% Defined programs:
% base                           The static facts, regarding machines.
% transition(gt)                 Update the machine states.
%
% In this file defined predicates:
% csPrepared(M, GT)              If the cap station M is prepared to mount a cap in GT.
% inUse(L, GT)                   Machine location L is in use in GT.
% rsFillState(M, S, GT)          Ring station M has S additional bases loaded in GT.
% storing(M, P, GT)              Machine M stores product P in GT.
%
% Additionally used predicates:
% capMounted(M, P, GT)           Defined in mountCap.lp
% deliveryTimes(B, E)            Defined in facts.lp
% free(L, GT)                    Defined in the task descriptions.
% machine(M)                     Defined in facts.lp
% ringColorCost(Col, Cost)       Defined in facts.lp
% ringCost(C)                    Defined in facts.lp
% ringMounted(M, P, C, GT)       Defined in mountRing.lp
% ringStation(M)                 Defined in facts.lp
% rsFed(M, GT)                   Defined in feedRS.lp
% pickUp(R, P, M, GT)            Defined in some tasks descriptions.
% process(M, P, GT)              Defined in some tasks descriptions.

rsFillState(M, 0, T-1) :- ringStation(M), deliveryTimes(T, _).

#program transition(gt).
% Update inUse, we block a machine one time step longer than actually used, this is because of the time resolution.
inUse(L, gt) :- inUse(L, gt-1), not free(L, gt-1).

storing(M, P, gt) :- storing(M, P, gt-1), not pickUp(_, P, M, gt).
storing(M, P, gt) :- process(M, P, gt). %TODO: Consider processing time.

csPrepared(M, gt) :- csPrepared(M, gt-1), not capMounted(M, _, gt).

rsFillState(M, S,   gt) :- rsFillState(M, S, gt-1), not rsFed(M, gt), not ringMounted(M, _, C, gt), ringColor(C).
rsFillState(M, S+1, gt) :- rsFillState(M, S, gt-1), rsFed(M, gt).
rsFillState(M, S-C, gt) :- rsFillState(M, S, gt-1), ringMounted(M, _, C, gt), ringCost(Cost), ringColorCost(C, Cost).
