#include "defaultMap.lp".
#include "script.lp".

#script (python)

MaxOrders = 1
MaxQuantity = 1
Res = 5
Horizon = 300
Start = 240
End = 240

CancelSolveTime = 3

StartDelivery = 240
EndDelivery = 1140

MaxFeedRS = 3
MaxGetBase = 4
MaxGetProduct = 4
MaxPrepCS = 2
Robots = ["Tuvok"] #, "T'Pol", "Spock"]

def generateInstance():
	return ([],
		[RingInfo("YELLOW", "RS2", 1), RingInfo("GREEN", "RS2", 0),
		 RingInfo("BLUE", "RS1", 2), RingInfo("ORANGE", "RS1", 0)],
		[OrderInfo(1, 1, "BLACK", "BLACK", "none",   "none",   "none",   240, 1140, 240),
		 OrderInfo(7, 1, "RED",   "BLACK", "GREEN",  "YELLOW", "ORANGE", 860, 1030, 240),
		 OrderInfo(6, 1, "BLACK", "GREY",  "ORANGE", "BLUE",   "none",   940, 1060, 360),
		 OrderInfo(2, 1, "BLACK", "GREY",  "none",   "none",   "none",   550,  715, 408),
		 OrderInfo(5, 1, "RED",   "BLACK", "YELLOW", "none",   "none",   855,  970, 510),
		 OrderInfo(3, 2, "BLACK", "GREY",  "none",   "none",   "none",   840,  935, 706),
		 OrderInfo(4, 1, "RED",   "BLACK", "none",   "none",   "none",   985, 1140, 789)],
		[])

#end.

#program testInstance.
%:- not begin("Tuvok", prepareCS(m("C", "CS1", "I"), "Tuvok", 1), 0).
%:- not begin("Tuvok", prepareCS(m("C", "CS2", "I"), "Tuvok", 1), 8).
