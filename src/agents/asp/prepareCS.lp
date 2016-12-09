% Everythin concerning the prepareCS task.
% The task is not associated with an order and every robot can perform it a limited amount per CS.

% In outTeam the locations are generated.
#program ourTeam(t).

% This means each robot can prepare each CS up to 3 times, even when one robot is doing all the preparations we can
% produce at least 3 products.
{ task(prepareCS(L, R, S));
  points(prepareCS(L, R, S), 0);
  duration(prepareCS(L, R, S), @prepareCSTaskDuration());
  location(prepareCS(L, R, S), L)
  } = 4 :- robot(R), capLocation(L), S = 1..3.

#program transition(gt).
inUse(L, gt) :- begin(_, prepareCS(L, O, Q, S), gt), not end(_, prepareCS(L, O, Q, S), gt).
free(L, gt) :- end(_, prepareCS(L, _, _, _), gt).

%possible(R, prepareCS(_, _, _, _), gt) :- holding(R, gt, none).
canSkip(prepareCS(L, O, Q, S), gt) :- toBeDone(prepaceCS(L, O, Q, S), gt), cap(O, C), capStationAssignment(M, C),
									  csPrepared(M, gt).
done(prepareCS(L, O, Q, S), gt) :- toBeDone(prepaceCS(L, O, Q, S), gt), nextTask(prepaceCS(L, O, Q, S), T),
								   not toBeDone(T, gt), toBeDone(T, gt-1).

{ csPrepared(M, gt)
  } = 1 :- end(R, prepareCS(L, O, Q, S), gt), not failure(prepareCS(L, O, Q, S), gt), cap(O, C),
		   capStationAssignment(M, C).

% Spawn the task on a new order.
#program newOrder(order, quantity, base, cap, deliveryBegin, deliveryEnd).
%{ spawnTask(getBase(L, order, Q, S));
%  nextTask(T, getBase(L, order, Q, S))
%  } = 2 :- capStationAssignment(M, cap), capLocation(M, L), Q = 1..quantity, not spawnTask(.

