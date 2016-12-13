% For the external functions which are implemented in C++.

#script (python)

def toASP(time):
	res = 5
	ret = time // res
	if (time % res) * 2 >= res:
		ret = ret + 1
	return ret

def robots():
	return ["Tuvok", "T'Pol", "Spock"]

def maxOrders():
	return MaxOrders

def maxQuantity():
	return MaxQuantity

def minDeliveryTime():
	return toASP(240)

def maxDeliveryTime():
	return toASP(19 * 60)

def capColor(m):
	if m == 1:
		return "GREY"
	return "BLACK"

def maxDriveDuration():
	return toASP(30)

def explorationTaskDuration():
	return toASP(5)

def maxTaskDuration():
	return toASP(65)

def maxTicks():
	return 25

def prepareCSTaskDuration():
	return toASP(35)

def mountCapTaskDuration():
	return toASP(15)

def mountRingTaskDuration():
	return toASP(15)

def feedRSTaskDuration():
	return toASP(15)

def deliverTaskDuration():
	return toASP(15)

def getProductTaskDuration():
	return toASP(10)

def getTaskDuration():
	return toASP(5)

def gameRange():
	return range(0, maxDeliveryTime() + 1)

def maxFeedRS():
	return MaxFeedRS

def maxGetBase():
	return MaxGetBase

def maxGetProduct():
	return MaxGetProduct

def maxPrepCS():
	return MaxPrepCS

import clingo

def main(ctl):
	(withExploration, programParts) = generateInstance()
	ctl.configuration.solver.heuristic = "Domain"

	for file in ["delivery.lp", "facts.lp", "feedRS.lp", "getBase.lp", "getProduct.lp", "goal.lp", "machines.lp", "mountCap.lp", "mountRing.lp", "orders.lp", "prepareCS.lp", "products.lp", "robots.lp", "state.lp"]:
		ctl.load(file)

	if withExploration:
		ctl.load("exploration.lp")

	ctl.ground([("base", [])])

	for gt in gameRange():
		print("Start ground " + str(gt) + ": " + str(programParts[gt]))
		ctl.ground(programParts[gt])
		ctl.ground([("transition", [gt])])
		if ctl.solve().unsatisfiable:
			return

	print("Start solve.")
	ctl.solve()
#end.
