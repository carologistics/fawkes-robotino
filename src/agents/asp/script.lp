#script (python)

import clingo

def clingoToASP(timeSymbol):
	return toASP(timeSymbol.number)

def toASP(time):
	ret = time // Res
	if (time % Res) * 2 >= Res:
		ret = ret + 1
	return ret

def robots():
	return Robots

def maxOrders():
	return MaxOrders

def maxQuantity():
	return MaxQuantity

def minDeliveryTime():
	return toASP(StartDelivery)

def maxDeliveryTime():
	return toASP(EndDelivery)

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
	return range(0, toASP(End) + 1)

def horizon():
	return toASP(Horizon)

def maxFeedRS():
	return MaxFeedRS

def maxGetBase():
	return MaxGetBase

def maxGetProduct():
	return MaxGetProduct

def maxPrepCS():
	return MaxPrepCS

class Foo:
	def __init__(self, ctl):
		self.ctl = ctl

	def onModel(self, model):
		print("Model")

	def solve(self, gt):
		self.ctl.solve(self.onModel)

def main(ctl):
	(withExploration, instance) = generateInstance()
	#ctl.configuration.solver.heuristic = "Domain"

	for file in ["defaultMap.lp", "delivery.lp", "facts.lp", "feedRS.lp", "getBase.lp", "getProduct.lp", "goal.lp", "machines.lp", "mountCap.lp", "mountRing.lp", "orders.lp", "prepareCS.lp", "products.lp", "robots.lp", "state.lp"]:
		ctl.load(file)

	print("Ground base.", flush = True)
	ctl.ground([("base", [])])
	print("Ground team.", flush = True)
	ctl.ground([("ourTeam", ["C"])])

	for robot in robots():
		ctl.assign_external(clingo.Function("availableRobot", [robot]), True)
		ctl.assign_external(clingo.Function("robotLocation", [robot, "ins-out", 0]), True)

	print("Solve after init.", flush = True)
	ctl.solve()

	print("Begin with instance.", flush = True)
	f = Foo(ctl)
	for gt in gameRange():
		print("Prepare " + str(gt), flush = True)
		(assign, release) = instance[gt]
		for a in assign:
			ctl.assign_external(a[0], a[1])
		for r in release:
			ctl.release_external(r)
		print("Solve " + str(gt), flush = True)
		f.solve(gt)
#end.
