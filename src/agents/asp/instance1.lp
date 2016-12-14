#include "defaultMap.lp".
#include "script.lp".

#script (python)

MaxOrders = 1
MaxQuantity = 1
MaxTicks = 7

MaxFeedRS = 2
MaxGetBase = 2
MaxGetProduct = 4
MaxPrepCS = 1
Robots = ["Spock"] #, "T'Pol", "Tuvok"]

def generateInstance():
	list = []
	for gt in gameRange():
		list.append([])

	list[0].extend(defaultMap())
	list[0].extend([
		("ourTeam", ["C"]),
		#("addRobot", ["Tuvok", 0, 0]),
		#("setRobotLocation", ["Tuvok", "ins-out", 0, 1]),
		#("addRobot", ["T'Pol", 0, 2]),
		#("setRobotLocation", ["T'Pol", "ins-out", 0, 3]),
		("addRobot", ["Spock", 0, 4]),
		("setRobotLocation", ["Spock", "ins-out", 0, 5])])


	list[minDeliveryTime()].extend([
		("setRingInfo", ["YELLOW", 1, "RS2"]),
		("setRingInfo", ["GREEN", 0, "RS2"]),
		("setRingInfo", ["BLUE", 2, "RS1"]),
		("setRingInfo", ["ORANGE", 0, "RS1"])])

	list[minDeliveryTime()].extend([
		#("newOrder", [7, 1, "RED", "BLACK", "GREEN", "YELLOW", "ORANGE", 172, 206, minDeliveryTime()]),
		("newOrder", [1, 1, "BLACK", "BLACK", "none", "none", "none", 48, 228, minDeliveryTime()])])

	gt = toASP(120 + 240)
	#list[gt].append(("newOrder", [6, 1, "BLACK", "GREY", "ORANGE", "BLUE", "none", 188, 212, gt]))

	gt = toASP(168 + 240)
	#list[gt].append(("newOrder", [2, 1, "BLACK", "GREY", "none", "none", "none", 110, 143, gt]))

	gt = toASP(270 + 240)
	#list[gt].append(("newOrder", [5, 1, "RED", "BLACK", "YELLOW", "none", "none", 171, 194, gt]))

	gt = toASP(466 + 240)
	#list[gt].append(("newOrder", [3, 2, "BLACK", "GREY", "none", "none", "none", 168, 187, gt]))

	gt = toASP(549 + 240)
	#list[gt].append(("newOrder", [4, 1, "RED", "BLACK", "none", "none", "none", 197, 228, gt]))

	return (False, list)

#end.
