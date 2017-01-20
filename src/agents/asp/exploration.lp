% In this file defined predicates:
% location(z(Z))                 The location of a zone, only needed if we have to explore the zone.
% points(expore(Z), P)           The points for exploring zone Z, basicly a huge value to ensure machines are explored,
%                                even if we have orders to fullfil.
% possible(R, explore(Z), GT)    If robot R is capable of exploring zone Z in GT. (Yes he is. ;))
% task(explore(Z))               The task to explore zone Z.
% taskDuration(explore(Z), D)    The needed time to explore zone Z.
% taskLocation(explore(Z), L)    The location to explore zone Z.
% toBeDone(explore(Z), 0)        The zone has to be explored.
%
% Additionally used predicates:
% horizon(H)                     Defined in facts.lp
% robot(R)                       Defined in facts.lp
% zone(Z)                        Defined in facts.lp

task(explore(Z)) :- zone(Z).

#external location(z(Z)) : zone(Z).
#external toBeDone(explore(Z), 0) : zone(Z).

taskLocation(explore(Z), z(Z)) :- zone(Z), location(z(Z)).
taskDuration(explore(Z), @explorationTaskDuration()) :- zone(Z).
points(explore(Z), 100) :- zone(Z).

possible(R, explore(Z), GT) :- robot(R), zone(Z), horizon(H), GT=0..H-1.
