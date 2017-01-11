#include "defaultMap.lp".
#include "script.lp".

#script (python)

MaxOrders = 1
MaxQuantity = 1
Res = 5
Horizon = 300
Start = 240
End = 240

StepSeconds = None

StartDelivery = 240
EndDelivery = 1140

MaxFeedRS = 1
MaxGetBase = 1
MaxGetProduct = 2
MaxPrepCS = 1
MaxProducts = 10
Robots = ["Tuvok", "T'Pol"]

def generateInstance():
	return ([],
		[RingInfo("YELLOW", "RS2", 1), RingInfo("GREEN", "RS2", 0),
		 RingInfo("BLUE", "RS1", 2), RingInfo("ORANGE", "RS1", 0)],
		[OrderInfo(1, 1, "BLACK", "BLACK", "GREEN", "ORANGE", "GREEN", 240, 1140, 240)],
		[])

#end.

#program testInstance.

:- not begin("T'Pol", prepareCS(m("C", "CS2", "I"), 1), 0).
:- not begin("T'Pol", getProduct(m("C", "CS2", "O"), 1), D1),
   fullDuration(getProduct(m("C", "CS2", "0"), 1), "ins-out", D1).
:- not begin("T'Pol", feedRS(m("C", "RS1", "I"), 1), D1 + D2),
   fullDuration(prepareCS(m("C", "CS2", "I"), 1), "ins-out", D1),
   fullDuration(getProduct(m("C", "CS2", "O"), 1), m("C", "CS2", "I"), D2).
:- not begin("T'Pol", prepareCS(m("C", "CS1", "I"), 1), D1 + D2 + D3),
   fullDuration(prepareCS(m("C", "CS2", "I"), 1), "ins-out", D1),
   fullDuration(getProduct(m("C", "CS2", "O"), 1), m("C", "CS2", "I"), D2),
   fullDuration(freedRS(m("C", "RS1", "I"), 1), m("C", "CS2", "O"), D3).
:- not begin("T'Pol", getProduct(m("C", "CS1", "O"), 1), D1 + D2 + D3 + D4),
   fullDuration(prepareCS(m("C", "CS2", "I"), 1), "ins-out", D1),
   fullDuration(getProduct(m("C", "CS2", "O"), 1), m("C", "CS2", "I"), D2),
   fullDuration(freedRS(m("C", "RS1", "I"), 1), m("C", "CS2", "O"), D3),
   fullDuration(prepareCS(m("C", "CS1", "I"), 1), m("C", "RS1", "I"), D4).
:- not begin("T'Pol", feedRS(m("C", "RS2", "I"), 1), D1 + D2 + D3 + D4 + D5),
   fullDuration(prepareCS(m("C", "CS2", "I"), 1), "ins-out", D1),
   fullDuration(getProduct(m("C", "CS2", "O"), 1), m("C", "CS2", "I"), D2),
   fullDuration(freedRS(m("C", "RS1", "I"), 1), m("C", "CS2", "O"), D3),
   fullDuration(prepareCS(m("C", "CS1", "I"), 1), m("C", "RS1", "I"), D4),
   fullDuration(getProduct(m("C", "CS1", "O"), 1), m("C", "CS1", "I"), D5).

:- not begin("Tuvok", getBase(m("C", "BS", "O"), "BLACK", 1), 0).
