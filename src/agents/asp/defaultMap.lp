#program ourTeam(t).

setDriveDuration("ins-out", "ins-out", 0).
driveDuration(L1, L2, D) :- setDriveDuration(L1, L2, D).
driveDuration(L1, L2, @clingoToASP(5)) :- location(L1), location(L2), not setDriveDuration(L1, L2, _).

