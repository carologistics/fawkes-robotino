% Defines what products there are.

#program transition(gt).
product(R, I) :- genProduct(R, I, _, gt).
{ base(     product(R, I), B);
  cap(      product(R, I), none, gt);
  ringOne(  product(R, I), none, gt);
  ringTwo(  product(R, I), none, gt);
  ringThree(product(R, I), none, gt)
  } = 5 :- genProduct(R, I, B, gt).

cap(P, C, gt) :- cap(P, C, gt-1), not mountedCap(_, P, gt).
cap(P, C, gt) :- mountedCap(M, P, gt), capStationAssignment(M, C).

ringOne(P, R, gt) :- ringOne(P, R, gt-1), not mountedRing(_, P, _, gt).
ringOne(P, R, gt) :- mountedRing(M, P, R, gt), ringOne(P, none, gt-1).

ringTwo(P, R, gt) :- ringOne(P, R, gt-1), not mountedRing(_, P, _, gt).
ringTwo(P, R, gt) :- mountedRing(M, P, R, gt), not ringOne(P, none, gt).

ringThree(P, R, gt) :- ringOne(P, R, gt-1), not mountedRing(_, P, _, gt).
ringThree(P, R, gt) :- mountedRing(M, P, R, gt), not ringTwo(P, none, gt).

nextID(R, I, gt) :- nextID(R, I, gt-1), not genProduct(R, _, gt).
nextID(R, I, gt) :- robot(R), MaxID = #max{ J:genProduct(R, J, gt) }, I = MaxID + 1.
