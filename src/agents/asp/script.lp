#script (python)

import clingo

def toASP(time):
	res = 5
	ret = time // res
	if (time % res) * 2 >= res:
		ret = ret + 1
	return ret

def robots():
	return Robots

def maxOrders():
	return MaxOrders

def maxQuantity():
	return MaxQuantity

def minDeliveryTime():
	return toASP(240)

def maxDeliveryTime():
	return toASP(19 * 60)

def capColor(m):
	if m == clingo.Number(1):
		return "GREY"
	return "BLACK"

def maxDriveDuration():
	return toASP(30)

def explorationTaskDuration():
	return toASP(5)

def maxTaskDuration():
	return toASP(35)

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

def main(ctl):
	(withExploration, programParts) = generateInstance()
	ctl.configuration.solver.heuristic = "Domain"

	for file in ["delivery.lp", "facts.lp", "feedRS.lp", "getBase.lp", "getProduct.lp", "goal.lp", "machines.lp", "mountCap.lp", "mountRing.lp", "orders.lp", "prepareCS.lp", "products.lp", "robots.lp", "state.lp"]:
		ctl.load(file)

	if withExploration:
		ctl.load("exploration.lp")

	print("Ground base.", flush = True)
	ctl.ground([("base", [])])

	for gt in gameRange():
		print("Start ground " + str(gt) + ": " + str(programParts[gt]), flush = True)
		ctl.ground(programParts[gt])
		print("Start ground " + str(gt) + ": (transition, [" + str(gt) + "])", flush = True)
		ctl.ground([("transition", [gt])])
		#programParts[gt].append(("transition", [gt]))
		#for part in programParts[gt]:
		#	print("Start ground " + str(gt) + ": " + str(part), flush = True)
		#	ctl.ground([part])
		print("Start solve " + str(gt) + ".", flush = True)
		if ctl.solve().unsatisfiable:
			return

	print("Start solve.", flush = True)
	ctl.solve()
#end.
