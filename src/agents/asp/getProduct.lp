% Everything concerning the getProduct task.
% The task fetches a product from a machine output.

% In ourTeam the locations are generated.
#program ourTeam(t).

% This means each robot can get 20 products from each machine.
#const maxGetProduct = 20.

task(getProduct(L, R, S))                                :- robot(R), getLocation(_, L), S = 1..maxGetProduct.
points(getProduct(L, R, S), 0)                           :- task(getProduct(L, R, S)).
duration(getProduct(L, R, S), @getProductTaskDuration()) :- task(getProduct(L, R, S)).
location(getProduct(L, R, S), L)                         :- task(getProduct(L, R, S)).
spawn(getProduct(L, R, S), T)                            :- task(getProduct(L, R, S)), deliveryTimes(T, _).

#program transition(gt).
inUse(L, gt) :- begin(R, getProduct(L, R, S), gt), not end(R, getProduct(L, R, S), gt).
free(L, gt)  :- end(R, getProduct(L, R, _), gt).

possible(R, getProduct(L, R, S), gt) :- task(getProduct(L, R, S)), holding(R, gt, none), getLocation(M, L),
										not storing(M, gt, none), not inUse(L, gt).

{ pickUp(R, P, M, gt)
  } = 1 :- end(R, getProduct(L, R, S), gt), not failure(getProduct(L, R, S), gt), getLocation(M, L), storing(M, gt, P).

