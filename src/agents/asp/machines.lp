storing(M, T-1, none) :- machine(M), deliveryTimes(T, _).

#program transition(gt).
storing(M, gt, P)    :- storing(M, gt-1, P), not pickUp(_, P, M, gt).
storing(M, gt, none) :- storing(M, gt-1, P), pickUp(_, P, M, gt).
storing(M, gt, P)    :- produced(M, gt, P).

csPrepared(M, gt) :- csPrepared(M, gt-1), not csUsed(M, gt).
