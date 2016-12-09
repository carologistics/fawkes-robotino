storing(M, T-1, none) :- machine(M), deliveryTimes(T, _).

rsFillState(M, 0, 0) :- ringStation(M).

#program transition(gt).
storing(M, gt, P)    :- storing(M, gt-1, P), not pickUp(_, P, M, gt).
storing(M, gt, none) :- storing(M, gt-1, P), pickUp(_, P, M, gt).
storing(M, gt, P)    :- produced(M, gt, P).

csPrepared(M, gt) :- csPrepared(M, gt-1), not mountedCap(M, _, gt).

rsFillState(M, S, gt)   :- rsFillState(M, S, gt-1), not rsFed(M, gt), not mountedRing(M, _, C, gt), ringCost(C).
rsFillState(M, S+1, gt) :- rsFillState(M, S, gt-1), rsFed(M, gt).
rsFillState(M, S-C, gt) :- rsFillState(M, S, gt-1), mountedRing(M, _, C, gt), ringCost(C).
