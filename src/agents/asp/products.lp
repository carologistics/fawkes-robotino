% Defines what products there are.

#program transition(gt).
product(R, I) :- genProduct(R, I, _, gt).
base(R, I, B) :- genProduct(R, I, B, gt).

nextID(R, I, gt) :- nextID(R, I, gt-1), not genProduct(R, _, gt).
nextID(R, I, gt) :- robot(R), MaxID = #max{ J:genProduct(R, J, gt) }, I = MaxID + 1.
