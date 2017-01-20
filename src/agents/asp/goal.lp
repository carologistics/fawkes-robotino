% Contains the optimization goal.

#program ourTeam(t).
% Our goal.
#maximize{ P@2,T : end(_, T, GT), points(T, P) }.

% --opt-strategy=usc

%#heuristic begin(_,T,GT). [GT,false]
%#heuristic begin(_,T,GT). [-1,sign]

%#minimize{ GT@1,T : begin(_, T, GT), task(T) }.

