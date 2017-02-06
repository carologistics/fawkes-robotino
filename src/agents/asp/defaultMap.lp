% Used by script.lp to provide the default map and its distances.

#program ourTeam(t).

% setDriveDuration gives the duration in (rounded) real time seconds.
setDriveDuration(m("C", "BS",  "O"), m("C", "BS",  "I"),  9).

setDriveDuration(m("C", "CS1", "I"), m("C", "BS",  "I"), 15).
setDriveDuration(m("C", "CS1", "I"), m("C", "BS",  "O"), 14).

setDriveDuration(m("C", "CS1", "O"), m("C", "BS",  "I"), 17).
setDriveDuration(m("C", "CS1", "O"), m("C", "BS",  "O"), 36).
setDriveDuration(m("C", "CS1", "O"), m("C", "CS1", "I"), 10).

setDriveDuration(m("C", "CS2", "I"), m("C", "BS",  "I"), 13).
setDriveDuration(m("C", "CS2", "I"), m("C", "BS",  "O"), 23).
setDriveDuration(m("C", "CS2", "I"), m("C", "CS1", "I"),  8).
setDriveDuration(m("C", "CS2", "I"), m("C", "CS1", "O"),  8).

setDriveDuration(m("C", "CS2", "O"), m("C", "BS",  "I"),  9).
setDriveDuration(m("C", "CS2", "O"), m("C", "BS",  "O"), 21).
setDriveDuration(m("C", "CS2", "O"), m("C", "CS1", "I"),  5).
setDriveDuration(m("C", "CS2", "O"), m("C", "CS1", "O"),  8).
setDriveDuration(m("C", "CS2", "O"), m("C", "CS2", "I"),  7).

setDriveDuration(m("C", "DS",  "I"), m("C", "BS",  "I"), 11).
setDriveDuration(m("C", "DS",  "I"), m("C", "BS",  "O"), 32).
setDriveDuration(m("C", "DS",  "I"), m("C", "CS1", "I"), 21).
setDriveDuration(m("C", "DS",  "I"), m("C", "CS1", "O"), 20).
setDriveDuration(m("C", "DS",  "I"), m("C", "CS2", "I"), 18).
setDriveDuration(m("C", "DS",  "I"), m("C", "CS2", "O"),  6).

setDriveDuration(m("C", "DS",  "O"), m("C", "BS",  "I"),  8).
setDriveDuration(m("C", "DS",  "O"), m("C", "BS",  "O"), 12).
setDriveDuration(m("C", "DS",  "O"), m("C", "CS1", "I"), 22).
setDriveDuration(m("C", "DS",  "O"), m("C", "CS1", "O"), 44).
setDriveDuration(m("C", "DS",  "O"), m("C", "CS2", "I"), 25).
setDriveDuration(m("C", "DS",  "O"), m("C", "CS2", "O"), 15).
setDriveDuration(m("C", "DS",  "O"), m("C", "DS",  "I"),  8).

setDriveDuration(m("C", "RS1", "I"), m("C", "BS",  "I"),  6).
setDriveDuration(m("C", "RS1", "I"), m("C", "BS",  "O"), 15).
setDriveDuration(m("C", "RS1", "I"), m("C", "CS1", "I"), 12).
setDriveDuration(m("C", "RS1", "I"), m("C", "CS1", "O"), 17).
setDriveDuration(m("C", "RS1", "I"), m("C", "CS2", "I"),  8).
setDriveDuration(m("C", "RS1", "I"), m("C", "CS2", "O"),  6).
setDriveDuration(m("C", "RS1", "I"), m("C", "DS",  "I"),  9).
setDriveDuration(m("C", "RS1", "I"), m("C", "DS",  "O"), 10).

setDriveDuration(m("C", "RS1", "O"), m("C", "BS",  "I"),  8).
setDriveDuration(m("C", "RS1", "O"), m("C", "BS",  "O"), 21).
setDriveDuration(m("C", "RS1", "O"), m("C", "CS1", "I"),  9).
setDriveDuration(m("C", "RS1", "O"), m("C", "CS1", "O"), 11).
setDriveDuration(m("C", "RS1", "O"), m("C", "CS2", "I"),  9).
setDriveDuration(m("C", "RS1", "O"), m("C", "CS2", "O"),  8).
setDriveDuration(m("C", "RS1", "O"), m("C", "DS",  "I"),  3).
setDriveDuration(m("C", "RS1", "O"), m("C", "DS",  "O"),  7).
setDriveDuration(m("C", "RS1", "O"), m("C", "RS1", "I"),  8).

setDriveDuration(m("C", "RS2", "I"), m("C", "BS",  "I"),  9).
setDriveDuration(m("C", "RS2", "I"), m("C", "BS",  "O"), 13).
setDriveDuration(m("C", "RS2", "I"), m("C", "CS1", "I"), 13).
setDriveDuration(m("C", "RS2", "I"), m("C", "CS1", "O"), 15).
setDriveDuration(m("C", "RS2", "I"), m("C", "CS2", "I"), 14).
setDriveDuration(m("C", "RS2", "I"), m("C", "CS2", "O"), 11).
setDriveDuration(m("C", "RS2", "I"), m("C", "DS",  "I"), 12).
setDriveDuration(m("C", "RS2", "I"), m("C", "DS",  "O"),  5).
setDriveDuration(m("C", "RS2", "I"), m("C", "RS1", "I"),  9).
setDriveDuration(m("C", "RS2", "I"), m("C", "RS1", "O"),  7).

setDriveDuration(m("C", "RS2", "O"), m("C", "BS",  "I"),  6).
setDriveDuration(m("C", "RS2", "O"), m("C", "BS",  "O"), 10).
setDriveDuration(m("C", "RS2", "O"), m("C", "CS1", "I"), 17).
setDriveDuration(m("C", "RS2", "O"), m("C", "CS1", "O"), 19).
setDriveDuration(m("C", "RS2", "O"), m("C", "CS2", "I"), 17).
setDriveDuration(m("C", "RS2", "O"), m("C", "CS2", "O"), 13).
setDriveDuration(m("C", "RS2", "O"), m("C", "DS",  "I"), 22).
setDriveDuration(m("C", "RS2", "O"), m("C", "DS",  "O"),  8).
setDriveDuration(m("C", "RS2", "O"), m("C", "RS1", "I"),  9).
setDriveDuration(m("C", "RS2", "O"), m("C", "RS1", "O"),  9).
setDriveDuration(m("C", "RS2", "O"), m("C", "RS2", "I"),  9).

setDriveDuration(L, L, 0) :- location(L).
setDriveDuration(L1, L2, D) :- setDriveDuration(L2, L1, D).

% For driveDuration we use the conversion.
driveDuration(L1, L2, @clingoToASP(D)) :- setDriveDuration(L1, L2, D).
driveDuration(L1, L2, @clingoToASP(5)) :- location(L1), location(L2), not setDriveDuration(L1, L2, _).

#script (python)

def releaseDriveDuration(ctl):
	myPrint("Release driveDurations.", flush = True)

	locations = [clingo.String("ins-out")]

	for machine in ["BS", "CS1", "CS2", "RS1", "RS2", "DS"]:
		for side in ["I", "O"]:
			locations.append(clingo.Function("m", ["C", machine, side]))

	for l1 in locations:
		for l2 in locations:
			for dur in range(0, maxDriveDuration() + 1):
				ctl.release_external(clingo.Function("driveDuration", [l1, l2, dur]))
#end.
