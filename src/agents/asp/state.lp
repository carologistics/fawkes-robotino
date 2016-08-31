#external horizon(h).

#program updateGametime(gt, t).
gameTime(gt, t).

#program transition(gt).

{ do(R, T, gt) : task(T) } 1 :- robot(R, gt).
:- do(R, T, gt), not possible(R, T, gt).

