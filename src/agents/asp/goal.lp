#program transition(gt).
% Our goal.
#maximize{ P@2,T,GT : begin(_, T, GT), points(T, P) }.

%#heuristic begin(_,T,GT). [GT,false]
%#heuristic begin(_,T,GT). [-1,sign]

%#minimize{ GT@1,T : begin(_, T, GT), task(T) }.

