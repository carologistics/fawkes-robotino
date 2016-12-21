#program ourTeam(t).

setDriveDuration(L, L, 0) :- location(L).

driveDuration(L1, L2, D) :- setDriveDuration(L1, L2, D).
driveDuration(L1, L2, @clingoToASP(5)) :- location(L1), location(L2), not setDriveDuration(L1, L2, _).

#script (python)

def releaseDriveDuration(ctl):
	print("Release driveDurations.", flush = True)

	locations = [clingo.String("ins-out")]

	for machine in ["BS", "CS1", "CS2", "RS1", "RS2", "DS"]:
		for side in ["I", "O"]:
			locations.append(clingo.Function("m", ["C", machine, side]))

	for l1 in locations:
		for l2 in locations:
			for dur in range(0, maxDriveDuration() + 1):
				ctl.release_external(clingo.Function("driveDuration", [l1, l2, dur]))
#end.
