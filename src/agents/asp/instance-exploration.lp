#include "exploration.lp".
#include "script.lp".

#script (python)

MaxOrders = 0
MaxQuantity = 0
Res = 5
Horizon = 120
End = 0

StartDelivery = 240
EndDelivery = 1140

MaxFeedRS = 2
MaxGetBase = 2
MaxGetProduct = 4
MaxPrepCS = 1
Robots = ["Tuvok"] #, "T'Pol", "Spock"]

def generateInstance():
	list = []
	for gt in gameRange():
		assign = []
		release = []

		list.append((assign, release))

	for z in [1,2,4,5,6,7,8,9,10,11,14,24]:
		list[0][0].append((clingo.Function("location", [clingo.Function("z", [z])]), True))
		list[0][0].append((clingo.Function("toBeDone", [clingo.Function("explore", [z]), 0]), True))
	return (True, list)

#end.

%:- not begin("Tuvok", explore(5), 0).
