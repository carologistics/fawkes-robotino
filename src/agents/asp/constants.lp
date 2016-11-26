% File to collect all constants.

% Task durations (avg.) in seconds. Update if robot and/or skiller improve.
#const explorationTaskDuration=10.

% Not the real points for exploring a zone, but a large number, so that the planer tries to explore zones, even if there are production orders.
#const explorationPoints=100.

% How many ticks (i.e. new events) we expect at maximum.
#const maxTicks = 30000.

% The map coordinates in centimeters.
#const mapMinX = -600.
#const mapMaxX =  600.
#const mapMinY = -100.
#const mapMaxY =  600.

% The longest task duration possible, in time units.
#const maxDuration = 50.
