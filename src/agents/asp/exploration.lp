#external location(z(Z)) : zone(Z).
#external task(explore(Z)) : zone(Z).

taskLocation(explore(Z), z(Z)) :- zone(Z).
duration(explore(Z), @explorationTaskDuration()) :- zone(Z).
points(explore(Z), 100) :- zone(Z).

possible(R, explore(Z), GT) :- robot(R), toBeDone(explore(Z), GT).
