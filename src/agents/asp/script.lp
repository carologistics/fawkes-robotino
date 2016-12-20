#script (python)

import clingo
import time

from collections import namedtuple

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
	return range(toASP(Start), toASP(End) + 1)

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

RingInfo = namedtuple("RingInfo", "Color Machine Cost")
OrderInfo = namedtuple("OrderInfo", "Number Quantity Base Cap Ring1 Ring2 Ring3 DelBegin DelEnd Spawn")

class Foo:
	def __init__(self, ctl):
		self.ctl = ctl
		self.LastModel = None

	def onModel(self, model):
		self.LastModel = model

	def solve(self, gt):
		result = self.ctl.solve_async(self.onModel)
		if not result.wait(CancelSolveTime):
			print("Interrupt solving.", flush = True)
			self.ctl.interrupt()

		result.get()

def zoneLocationExternal(zone):
	return clingo.Function("location", [clingo.Function("z", [zone])])

def taskExternal(task):
	return clingo.Function("toBeDone", [task, 0])

def zoneTaskExternal(zone):
	return taskExternal(clingo.Function("explore", [zone]))

def spawnZone(ctl, zone):
	print("Add zone " + str(zone) + " to explore.", flush = True)
	ctl.assign_external(zoneLocationExternal(zone), True)
	ctl.assign_external(zoneTaskExternal(zone), True)

def releaseZone(ctl, zone, array = None):
	ctl.release_external(zoneLocationExternal(zone))
	ctl.release_external(zoneTaskExternal(zone))

	if array != None:
		print("Remove zone " + str(zone) + " from the zones to explore.", flush = True)
		array.remove(zone)

def setRingColor(ctl, ring):
	print("Set Ring-Color: " + str(ring))
	ctl.ground([("setRingInfo", [ring.Color, ring.Cost, ring.Machine])])

	for machine in ["RS1", "RS2"]:
		ctl.release_external(clingo.Function("ringStationAssignment", [machine, ring.Color]))
	for cost in [0,1,2]:
		ctl.release_external(clingo.Function("ringColorCost", [ring.Color, cost]));

def addOrder(ctl, order):
	print("Add Order: " + str(order))
	ctl.ground([("newOrder", [order.Number, order.Quantity, order.Base, order.Cap, order.Ring1, order.Ring2, order.Ring3, toASP(order.DelBegin), toASP(order.DelEnd)])])

	for color in ["BLACK", "RED", "SILVER"]:
		ctl.release_external(clingo.Function("base", [order.Number, color]))

	for color in ["BLACK", "GREY"]:
		ctl.release_external(clingo.Function("cap", [order.Number, color]))

	for color in ["BLUE", "YELLOW", "ORANGE", "GREEN"]:
		for ring in [1,2,3]:
			ctl.release_external(clingo.Function("ring", [order.Number, ring, color]))

	for gt in range(minDeliveryTime(), maxDeliveryTime() + 1):
		ctl.release_external(clingo.Function("orderBegin", [order.Number, gt]))
		ctl.release_external(clingo.Function("orderDeadline", [order.Number, gt]))


def currentTimeExternal(time):
	return clingo.Function("currentTime", [time])

def ringStationExternal(machine, fillState):
	return clingo.Function("rsFillState", [machine, fillState, 0])

def capStationExternal(machine):
	return clingo.Function("csPrepared", [machine, 0])

def main(ctl):
	(zonesToExplore, ringColors, orders, machineFailures) = generateInstance()

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
	for zone in range(1, 25):
		if zone in zonesToExplore:
			spawnZone(ctl, zone)
		else:
			releaseZone(ctl, zone)

	ringStations = {"RS1" : 0, "RS2" : 0}
	capStations = {"CS1" : False, "CS2" : False}

	ctl.assign_external(ringStationExternal("RS1", 0), True)
	ctl.assign_external(ringStationExternal("RS2", 0), True)
	ctl.assign_external(capStationExternal("CS1"), False)
	ctl.assign_external(capStationExternal("CS2"), False)

	f = Foo(ctl)
	first = True
	nextOrder = 0
	lastGT = 0
	for gt in gameRange():
		if gt == minDeliveryTime():
			for ring in ringColors:
				setRingColor(ctl, ring)
			ctl.ground([("startProduction", [])])
			ctl.release_external(clingo.Function("productionStarted", []))

		if nextOrder < maxOrders() and gt == toASP(orders[nextOrder].Spawn):
			addOrder(ctl, orders[nextOrder])
			nextOrder = nextOrder + 1

		if first:
			first = False
			print("Add test instance.", flush = True)
			ctl.ground([("testInstance", [])])

		for t in range(lastGT, gt):
			ctl.release_external(currentTimeExternal(t))
		ctl.assign_external(currentTimeExternal(gt), True)
		print("Solve " + str(gt), flush = True)
		f.solve(gt)
#end.
