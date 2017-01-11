#include "exploration.lp".
#include "script.lp".

#script (python)

MaxOrders = 0
MaxQuantity = 0
Res = 5
Horizon = 120
Start = 0
End = 0

StepSeconds = None

StartDelivery = 240
EndDelivery = 1140

MaxFeedRS = 2
MaxGetBase = 2
MaxGetProduct = 4
MaxPrepCS = 1
MaxProducts = 5
Robots = ["Tuvok", "T'Pol", "Spock"]

def generateInstance():
	return ([1,2,3,4,5,7,8,9,10,11,18,24], [], [], [])

#end.

#program testInstance.

:- not begin("Tuvok", explore(18), 0).
:- not begin("Tuvok", explore(24), DD + TD),
   driveDuration("ins-out", z(18), DD), taskDuration(explore(18), TD).
:- not begin("Tuvok", explore(4), DD1 + DD2 + TD1 + TD2),
   driveDuration("ins-out", z(18), DD1), taskDuration(explore(18), TD1),
   driveDuration(z(18), z(24), DD2), taskDuration(explore(24), TD2).
:- not begin("Tuvok", explore(3), DD1 + DD2 + DD3 + TD1 + TD2 + TD3),
   driveDuration("ins-out", z(18), DD1), taskDuration(explore(18), TD1),
   driveDuration(z(18), z(24), DD2), taskDuration(explore(24), TD2),
   driveDuration(z(24), z(4), DD3), taskDuration(explore(4), TD3).

:- not begin("T'Pol", explore(9), 0).
:- not begin("T'Pol", explore(10), DD + TD),
   driveDuration("ins-out", z(9), DD), taskDuration(explore(9), TD).
:- not begin("T'Pol", explore(11), DD1 + DD2 + TD1 + TD2),
   driveDuration("ins-out", z(9), DD1), taskDuration(explore(9), TD1),
   driveDuration(z(9), z(10), DD2), taskDuration(explore(10), TD2).
:- not begin("T'Pol", explore(8), DD1 + DD2 + DD3 + TD1 + TD2 + TD3),
   driveDuration("ins-out", z(9), DD1), taskDuration(explore(9), TD1),
   driveDuration(z(9), z(10), DD2), taskDuration(explore(10), TD2),
   driveDuration(z(10), z(11), DD3), taskDuration(explore(11), TD3).

:- not begin("Spock", explore(5), 0).
:- not begin("Spock", explore(1), DD + TD),
   driveDuration("ins-out", z(5), DD), taskDuration(explore(5), TD).
:- not begin("Spock", explore(2), DD1 + DD2 + TD1 + TD2),
   driveDuration("ins-out", z(5), DD1), taskDuration(explore(5), TD1),
   driveDuration(z(5), z(1), DD2), taskDuration(explore(1), TD2).
:- not begin("Spock", explore(7), DD1 + DD2 + DD3 + TD1 + TD2 + TD3),
   driveDuration("ins-out", z(5), DD1), taskDuration(explore(5), TD1),
   driveDuration(z(5), z(1), DD2), taskDuration(explore(1), TD2),
   driveDuration(z(1), z(2), DD3), taskDuration(explore(2), TD3).
