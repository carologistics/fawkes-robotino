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
MaxProducts = 5
Robots = ["Tuvok"]

def generateInstance():
	return ([],
		[RingInfo("YELLOW", "RS2", 1), RingInfo("GREEN", "RS2", 0),
		 RingInfo("BLUE", "RS1", 2), RingInfo("ORANGE", "RS1", 0)],
		[OrderInfo(1, 1, "BLACK", "BLACK", "none", "none", "none", 240, 1140, 240)],
		[])

#end.

#program testInstance.
:- not begin("Tuvok", prepareCS(m("C", "CS2", "I"), 1), 0).
:- not begin("Tuvok", getProduct(m("C", "CS2", "O"), 1), D1),
   fullDuration(prepareCS(m("C", "CS2", "I"), 1), "ins-out", D1).
:- not begin("Tuvok", feedRS(m("C", "RS1", "I"), 1), D1 + D2),
   fullDuration(prepareCS(m("C", "CS2", "I"), 1), "ins-out", D1),
   fullDuration(getProduct(m("C", "CS2", "O"), 1), m("C", "CS2", "I"), D2).
:- not begin("Tuvok", getBase(m("C", "BS", "O"), "BLACK", 1), D1 + D2 + D3),
   fullDuration(prepareCS(m("C", "CS2", "I"), 1), "ins-out", D1),
   fullDuration(getProduct(m("C", "CS2", "O"), 1), m("C", "CS2", "I"), D2),
   fullDuration(feedRS(m("C", "RS1", "I"), 1), m("C", "CS2", "O"), D3).
:- not begin("Tuvok", mountCap(m("C", "CS2", "I"), 1, 1), D1 + D2 + D3 + D4),
   fullDuration(prepareCS(m("C", "CS2", "I"), 1), "ins-out", D1),
   fullDuration(getProduct(m("C", "CS2", "O"), 1), m("C", "CS2", "I"), D2),
   fullDuration(feedRS(m("C", "RS1", "I"), 1), m("C", "CS2", "O"), D3),
   fullDuration(getBase(m("C", "BS", "O"), "BLACK", 1), m("C", "RS1", "I"), D4).
:- not begin("Tuvok", getProduct(m("C", "CS2", "O"), 2), D1 + D2 + D3 + D4 + D5),
   fullDuration(prepareCS(m("C", "CS2", "I"), 1), "ins-out", D1),
   fullDuration(getProduct(m("C", "CS2", "O"), 1), m("C", "CS2", "I"), D2),
   fullDuration(feedRS(m("C", "RS1", "I"), 1), m("C", "CS2", "O"), D3),
   fullDuration(getBase(m("C", "BS", "O"), "BLACK", 1), m("C", "RS1", "I"), D4),
   fullDuration(mountCap(m("C", "CS2", "I"), 1, 1), m("C", "BS", "O"), D5).
:- not begin("Tuvok", deliver(m("C", "DS", "I"), 1, 1), D1 + D2 + D3 + D4 + D5 + D6),
   fullDuration(prepareCS(m("C", "CS2", "I"), 1), "ins-out", D1),
   fullDuration(getProduct(m("C", "CS2", "O"), 1), m("C", "CS2", "I"), D2),
   fullDuration(feedRS(m("C", "RS1", "I"), 1), m("C", "CS2", "O"), D3),
   fullDuration(getBase(m("C", "BS", "O"), "BLACK", 1), m("C", "RS1", "I"), D4),
   fullDuration(mountCap(m("C", "CS2", "I"), 1, 1), m("C", "BS", "O"), D5),
   fullDuration(getProduct(m("C", "CS2", "O"), 2), m("C", "CS2", "I"), D6).
