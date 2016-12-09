% Everything concerning the prepareCS task.
% The task fetches a product from the CS shelf, the robot decides which position it will use, sends the prepare unmount
% message and feeds the fetched product into the machine.

% In ourTeam the locations are generated.
#program ourTeam(t).

% This means each robot can prepare each CS up to 3 times, even when one robot is doing all the preparations we can
% produce at least 3 products.
#const maxPrepCS = 3.

task(prepareCS(L, R, S))                               :- robot(R), capLocation(_, L), S = 1..maxPrepCS.
points(prepareCS(L, R, S), 2)                          :- task(prepareCS(L, R, S)).
duration(prepareCS(L, R, S), @prepareCSTaskDuration()) :- task(prepareCS(L, R, S)).
location(prepareCS(L, R, S), L)	                       :- task(prepareCS(L, R, S)).
spawn(prepareCS(L, R, S), T)                           :- task(prepareCS(L, R, S)), deliveryTimes(T, _).

#program transition(gt).
inUse(L, gt) :- begin(R, prepareCS(L, R, S), gt), not end(R, prepareCS(L, R, S), gt).
free(L, gt)  :- end(R, prepareCS(L, R, _), gt).

possible(R, prepareCS(L, R, S), gt) :- task(prepareCS(L, R, S)), holding(R, gt, none), capLocation(M, L),
									   not csPrepared(M, gt), storing(M, gt, none), not inUse(L, gt).

{ csPrepared(M, gt);
  genProduct(R, I, "TRANSPARENT", gt);
  produced(M, gt, product(R, I))
  } = 3 :- end(R, prepareCS(L, R, S), gt), not failure(prepareCS(L, R, S), gt), nextID(R, I, gt), capLocation(M, L).

