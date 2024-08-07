%---------------------------------------------------------------------------
%  Copyright  2016-2017  Bjoern Schaepers
%  Licensed under GPLv2+ license, cf. LICENSE file
%---------------------------------------------------------------------------
#script (python)

import clingo
import datetime
import sys

from collections import namedtuple

try:
	import psutil
	HAS_PSUTIL = True
except ImportError:
	HAS_PSUTIL = False

FlushPrint = True

def myPrint(string, flush = False):
	print(string)
	if flush:
		sys.stdout.flush()

RingInfo = namedtuple("RingInfo", "Color Machine Cost")
OrderInfo = namedtuple("OrderInfo", "Number Quantity Base Cap Ring1 Ring2 Ring3 DelBegin DelEnd Spawn")

def zoneLocationExternal(zone):
	return clingo.Function("location", [clingo.Function("z", [zone])])

def taskExternal(task):
	return clingo.Function("toBeDone", [task, 0])

def zoneTaskExternal(zone):
	return taskExternal(clingo.Function("explore", [zone]))

def currentTimeExternal(time):
	return clingo.Function("currentTime", [time])

def ringStationExternal(machine, fillState):
	return clingo.Function("rsFillState", [machine, fillState, 0])

def capStationExternal(machine):
	return clingo.Function("csPrepared", [machine, 0])

def taskLocation(machine):
	return clingo.Function("m", ["C", machine, "I"])

def deliveryTaskExternal(number, qty):
	return taskExternal(clingo.Function("deliver", [taskLocation("DS"), number, qty]))

def lateDeliveryTaskExternal(number, qty):
	return taskExternal(clingo.Function("lateDeliver", [taskLocation("DS"), number, qty]))

def splitParameters(string):
	temp = []
	loop = True

	while loop:
		comma = string.find(',')
		paranthesis = string.find('(')
		if comma != -1 and comma < paranthesis or paranthesis == -1 and comma != -1:
			temp.append(string[:comma])
			string = string[comma+1:]
		elif paranthesis != -1:
			count = 1
			index = paranthesis + 1
			while count != 0:
				if string[index] == ')':
					count = count - 1
				elif string[index] == '(':
					count = count + 1
				index = index + 1
			temp.append((string[:paranthesis], splitParameters(string[paranthesis+1:index-1])))
			string = string[index+1:]

		loop = comma != -1 or paranthesis != -1

	if len(string) != 0:
		temp.append(string)

	ret = []
	for s in temp:
		if s[0] == '"':
			ret.append(s[1:-1])
		else:
			ret.append(s)
	return ret

def memToString(mem):
	kib = 1024
	mib = 1024 ** 2
	gib = 1024 ** 3
	if mem >= gib:
		div = gib
		suffix = "GiB"
	elif mem >= mib:
		div = mib
		suffix = "MiB"
	elif mem >= kib:
		div = kib
		suffix = "KiB"
	else:
		div = 1
		suffix = "B"
	return "%.3f %s" % (float(mem) / div, suffix)

class Order:
	def __init__(self, info):
		self.Info = info
		self.CapTask = None
		self.RingTask = [None, None, None, None] # One more for index based access beginning at 1.
		self.DeliveryTasks = [None, None]

	def __repr__(self):
		return "Order(Order=(%d,%d), CapTask=%s, RingTask=%s, DeliveryTasks=%s)" % (self.Info.Number, self.Info.Quantity, str(self.CapTask), str(self.RingTask[1:]), str(self.DeliveryTasks))

class RobotState:
	def __init__(self, name):
		self.Name = name
		self.Location = "ins-out"
		self.Task = None
		self.Holding = None
		self.LastStep = -1

	def __repr__(self):
		return "RobotState(Name=%s, Location=%s, Task=%s, Holding=%s)" % (self.Name, str(self.Location), str(self.Task), str(self.Holding))

class Product:
	def __init__(self, base):
		self.Base = base
		self.Cap = None
		self.Rings = [None, None, None]

	def __repr__(self):
		return "Product(Base=%-11s, Rings=[%-6s, %-6s, %-6s], Cap=%-5s)" % (self.Base, str(self.Rings[0]), str(self.Rings[1]), str(self.Rings[2]), self.Cap)

class Machine:
	def __init__(self, name):
		self.Name = name
		self.Storing = None
		self.Working = None
		self.Broken = None

	def __repr__(self):
		return "Machine(Name=%s, Storing=%s, Working=%s, Broken=%s)" % (self.Name, str(self.Storing), str(self.Working), str(self.Broken))

class PlanEntry:
	def __init__(self,task,begin):
		self.Task = task
		self.Begin = begin
		self.End = None
		self.Score = 0

	def __repr__(self):
		return "PlanEntry(Begin = %3d, End = %3s, Score = %3d Task = %s)" % (self.Begin, str(self.End), self.Score, self.Task)

class Plan:
	def __init__(self,robot):
		self.Robot = robot
		self.Score = 0
		self.Entries = []

class Control:
	def __init__(self, ctl, zonesToExplore, ringColors, orders, machineFailures):
		self.CapStations       = {"CS1" : False, "CS2" : False}
		self.CTL               = ctl
		self.FirstSolve        = True
		self.GroundTime        = None
		self.InitialSolveTime  = None
		self.LastGameTime      = 0
		self.LastModel         = None
		self.Machines          = {}
		self.MachineFailures   = machineFailures
		self.MaxProductCount   = 0
		self.ModifyCTL         = True
		self.NextOrder         = 0
		self.Orders            = orders
		self.OrdersWithTasks   = {}
		self.Plan              = {}
		self.ProductionStarted = False
		self.Products          = []
		self.RingColors        = ringColors
		self.RingStations      = {"RS1" : 0, "RS2" : 0}
		self.Robots            = {}
		self.StartUpTime       = None
		self.ZonesToExplore    = zonesToExplore

		for file in ["defaultMap.lp", "facts.lp", "functions.lp", "goal.lp", "machines.lp", "orders.lp", "products.lp", "robots.lp", "state.lp", "taskDeliver.lp", "taskGoto.lp", "taskFeedRS.lp", "taskGetBase.lp", "taskGetProduct.lp", "taskMountCap.lp", "taskMountRing.lp", "taskPrepareCS.lp"]:
			self.CTL.load(file)

		self.CapColors         = {capColor(clingo.Number(1)) : "CS1", capColor(clingo.Number(2)) : "CS2"}

		groundBegin = datetime.datetime.now()
		myPrint("Ground base.", flush = FlushPrint)
		self.CTL.ground([("base", [])])
		myPrint("Ground team.", flush = FlushPrint)
		self.CTL.ground([("ourTeam", ["C"])])
		groundEnd = datetime.datetime.now()
		self.GroundTime = groundEnd - groundBegin

		for robot in robots():
			self.CTL.assign_external(clingo.Function("availableRobot", [robot]), True)
			self.Robots[robot] = RobotState(robot)
			self.Robots[robot].Location = "ins-out"
			self.Plan[robot] = Plan(robot)

		for machine in ["BS", "DS", "RS1", "RS2", "CS1", "CS2"]:
			self.Machines[machine] = Machine(machine)

		solveBegin = datetime.datetime.now()
		myPrint("Solve after init.", flush = FlushPrint)
		self.CTL.solve()
		solveEnd = datetime.datetime.now()
		self.InitialSolveTime = solveEnd - solveBegin

		self.StartUpTime = self.GroundTime + self.InitialSolveTime

	def run(self):
		myPrint("Begin with instance.", flush = FlushPrint)
		for zone in range(1, 25):
			if zone in self.ZonesToExplore:
				self.spawnZone(zone)
			else:
				self.releaseZone(zone)

		self.CTL.assign_external(ringStationExternal("RS1", 0), True)
		self.CTL.assign_external(ringStationExternal("RS2", 0), True)
		self.CTL.assign_external(capStationExternal("CS1"), False)
		self.CTL.assign_external(capStationExternal("CS2"), False)

		releaseDriveDuration(self.CTL)

		externals = []

		gameTimes = gameRange()
		gameTimesCount = len(gameTimes)
		for index in range(0, gameTimesCount):
			gt = gameTimes[index]
			if gt >= minDeliveryTime() and not self.ProductionStarted:
				self.ProductionStarted = True
				for ring in self.RingColors:
					self.setRingColor(ring)
				self.CTL.ground([("startProduction", [])])
				self.CTL.release_external(clingo.Function("productionStarted", []))

			while self.NextOrder < len(self.Orders) and gt >= toASP(self.Orders[self.NextOrder].Spawn):
				order = self.Orders[self.NextOrder]
				self.addOrder(order)
				self.NextOrder = self.NextOrder + 1

			for t in range(self.LastGameTime, gt):
				self.CTL.release_external(currentTimeExternal(t))
			self.CTL.assign_external(currentTimeExternal(gt), True)
			self.LastGameTime = gt

			if self.FirstSolve:
				self.FirstSolve = False
				myPrint("Add test instance.", flush = FlushPrint)
				self.CTL.ground([("testInstance", [])])

			for external in externals:
				#myPrint("Unset external: " + str(external), flush = FlushPrint)
				self.CTL.assign_external(external, False)
			externals = []

			for robot in self.Robots:
				info = self.Robots[robot]
				setLocation = True

				if info.Task is not None:
					remaining = info.Task[1] - gt
					if remaining < 0:
						myPrint("Calculated remaining time is " + str(remaining) + "! This should not happen.", flush = FlushPrint)
						self.endRobotTaskWithoutCheck(robot, info.Task[0], gt)
						info.Task = None
					elif remaining == 0:
						self.endRobotTaskWithoutCheck(robot, info.Task[0], gt)
						info.Task = None
					else:
						setLocation = False
						external = clingo.Function("robotDoing", [robot, info.Task[0], remaining])
						externals.append(external)
						myPrint("Set robot task:     " + str(external), flush = FlushPrint)

				if setLocation:
					external = clingo.Function("robotLocation", [robot, info.Location, 0])
					externals.append(external)
					myPrint("Set robot location: " + str(external), flush = FlushPrint)

				if info.Holding is not None:
					external = clingo.Function("holding", [robot, self.productExternal(info.Holding), 0])
					externals.append(external)
					myPrint("Set robot holding:  " + str(external), flush = FlushPrint)

				info.LastStep = -1

			for productIndex in range(0, len(self.Products)):
				product = self.Products[productIndex]
				productFunction = self.productExternal(productIndex)

				externals.append(clingo.Function("product", [productFunction]))
				externals.append(clingo.Function("productBase", [productFunction, product.Base]))
				myPrint("Add product %d: %s" % (productIndex, product), flush = FlushPrint)

				for ring in range(0, 3):
					if product.Rings[ring] is not None:
						externals.append(clingo.Function("productRing", [productFunction, ring+1, product.Rings[ring], 0]))
					else:
						break

				if product.Cap is not None:
					externals.append(clingo.Function("productCap", [productFunction, product.Cap, 0]))

			for name in self.Machines:
				machine = self.Machines[name]
				if machine.Broken is not None:
					externals.append(clingo.Function("broken", [name, machine.Broken, 0]))
					myPrint("%s is for %d steps broken." % (name, machine.Broken), flush = FlushPrint)
				if machine.Working is not None:
					externals.append(clingo.Function("processing", [name, self.productExternal(machine.Storing), machine.Working, 0]))
					myPrint("%s is working on %d for %d steps." % (name, machine.Storing, machine.Working), flush = FlushPrint)
				elif machine.Storing is not None:
					externals.append(clingo.Function("storing", [name, self.productExternal(machine.Storing), 0]))
					myPrint("%s is storing %d." % (name, machine.Storing), flush = FlushPrint)

			for external in externals:
				self.CTL.assign_external(external, True)

			myPrint("Solve, starting with time step %d. (%d of %d)" % (gt, index+1, gameTimesCount), flush = FlushPrint)
			self.solve()
			if index < gameTimesCount - 1:
				nextGT = gameTimes[index+1]
				for step in range(0, nextGT - gt):
					self.update(step, gt)

		if self.LastModel is not None:
			steps = horizonSteps()
			self.ModifyCTL = False
			for step in range(steps[0], steps[1] + 1):
				self.update(step, gt)
		else:
			step = 0

		score = 0
		myPrint("Plan:", flush = FlushPrint)
		for robot in self.Plan:
			plan = self.Plan[robot]
			score = score + plan.Score
			myPrint("  For Robot " + robot + " (" + str(plan.Score) + " Score):", flush = FlushPrint)
			i = 1
			for entry in plan.Entries:
				myPrint("    %2d: %s" % (i, str(entry)), flush = FlushPrint)
				i = i + 1

		myPrint("Solved for time steps %d to %d, additionally evaluated until %d. One time step stands for %d real time seconds." % (gameTimes[0], gameTimes[-1], gt + step, Res), flush = FlushPrint)
		myPrint("Overall score: %d" % (score), flush = FlushPrint)
		myPrint("Products:", flush = FlushPrint)
		for index in range(0, len(self.Products)):
			machine = None
			robot = None

			for name in self.Machines:
				if self.Machines[name].Storing == index:
					machine = name
					break

			for name in self.Robots:
				if self.Robots[name].Holding == index:
					robot = name
					break

			if machine is None and robot is None:
				myPrint("Product %d: %s is neither hold nor stored, this is an error!" % (index, self.Products[index]), flush = FlushPrint)
			elif machine is not None and robot is not None:
				myPrint("Product %d: %s is hold by %s and stored by %s, this is an error!" % (index, self.Products[index], robot, machine), flush = FlushPrint)
			elif machine is not None:
				myPrint("Product %d: %s stored by %s." % (index, self.Products[index], machine), flush = FlushPrint)
			else:
				myPrint("Product %d: %s hold   by %s." % (index, self.Products[index], robot), flush = FlushPrint)
		myPrint("CS prepare state: CS1: %5s CS2: %5s" % (str(self.CapStations["CS1"]), str(self.CapStations["CS2"])), flush = FlushPrint)
		myPrint("RS fill state:    RS1: %5d RS2: %5d" % (self.RingStations["RS1"], self.RingStations["RS2"]), flush = FlushPrint)
		myPrint("Set MaxProducts: %d Used: %d" % (maxProducts(), self.MaxProductCount), flush = FlushPrint)
		myPrint("Time for grounding base & team: %s" % str(self.GroundTime), flush = FlushPrint)
		myPrint("Time for solving base & team:   %s" % str(self.InitialSolveTime), flush = FlushPrint)
		myPrint("Overall startup time:           %s" % str(self.StartUpTime), flush = FlushPrint)
		if HAS_PSUTIL:
			p = psutil.Process()
			mem = p.memory_info()
			myPrint("Consumed memory at the end of the execution: Virtual %s Physical %s" % (memToString(mem.vms), memToString(mem.rss)), flush = FlushPrint)
		else:
			myPrint("psutil is not installed, no information abour memory consumption. See README.lp for more information.", flush = FlushPrint)

	def update(self, step, gt):
		if step + gt > maxDeliveryTime():
			return
		myPrint("Update on basis of step: " + str(step), flush = FlushPrint)
		for symbol in self.LastModel:
			if symbol.name == "doing":
				self.setRobotTask(symbol, step, gt)
			elif symbol.name == "end":
				self.endRobotTask(symbol, step, gt)
			elif symbol.name == "begin":
				self.beginRobotTask(symbol, step, gt)

		for name in self.Machines:
			machine = self.Machines[name]
			if machine.Broken is not None:
				machine.Broken = machine.Broken - 1
				if machine.Broken == 0:
					machine.Broken = None
			elif machine.Working is not None:
				machine.Working = machine.Working - 1
				if machine.Working == 0:
					machine.Working = None

	def updateFillState(self, machine, modify):
		oldFillState = self.RingStations[machine]
		newFillState = oldFillState + modify
		self.RingStations[machine] = newFillState
		if newFillState < 0 or newFillState > 3:
			myPrint("Invalid fill state for %s: %d!" % (machine, newFillState), flush = FlushPrint)
			sys.exit()
		if oldFillState == newFillState:
			return
		myPrint("Change fill state of " + str(machine) + " from " + str(oldFillState) + " to " + str(newFillState) + ".", flush = FlushPrint)
		if self.ModifyCTL:
			self.CTL.assign_external(ringStationExternal(machine, oldFillState), False)
			self.CTL.assign_external(ringStationExternal(machine, newFillState), True)

	def updatePrepareState(self, machine, state):
		oldState = self.CapStations[machine]
		if state != oldState:
			myPrint("Change " + str(machine) + " prepare state from " + str(oldState) + " to " + str(state) + ".", flush = FlushPrint)
			self.CapStations[machine] = state
			if self.ModifyCTL:
				self.CTL.assign_external(capStationExternal(machine), state)
		else:
			myPrint("Update prepare state, without update!", flush = FlushPrint)

	def beginRobotTask(self, symbol, step, gt):
		if symbol.arguments[2].number != step:
			return
		self.Plan[symbol.arguments[0].string].Entries.append(PlanEntry(str(symbol.arguments[1]), gt + step))

	def setRobotTask(self, symbol, step, gt):
		if symbol.arguments[3].number != step:
			return
		info = self.Robots[symbol.arguments[0].string]
		info.Task = (symbol.arguments[1],symbol.arguments[2].number + gt + step)
		info.LastStep = step

	def endRobotTask(self, symbol, step, gt):
		if symbol.arguments[2].number != step:
			return
		robot = symbol.arguments[0].string
		info = self.Robots[robot]
		if info.LastStep != step:
			info.Task = None
		self.endRobotTaskWithoutCheck(robot, symbol.arguments[1], gt + step)

	def endRobotTaskWithoutCheck(self, robot, symbol, time):
		plan = self.Plan[robot]
		for entry in plan.Entries:
			if entry.End is None:
				entry.End = time
				break

		info = self.Robots[robot]
		string = str(symbol)
		pos = string.find('(')
		task = string[:pos]
		parameters = splitParameters(string[pos+1:-1])
		score = 0
		if task == "explore":
			self.releaseZone(int(parameters[0]), True)
			# Little hack, so it will have a location which is not released.
			info.Location = "ins-out"
			score = 100
		elif task == "prepareCS":
			machine = parameters[0][1][1]
			self.updatePrepareState(machine, True)
			productIndex = self.spawnProduct("TRANSPARENT")
			self.machineProcess(machine, productIndex)
			info.Location = clingo.Function(parameters[0][0], parameters[0][1])
			score = 3
		elif task == "getBase":
			productIndex = self.spawnProduct(parameters[1])
			info.Holding = productIndex
			info.Location = clingo.Function(parameters[0][0], parameters[0][1])
		elif task == "feedRS":
			machine = parameters[0][1][1]
			productIndex = info.Holding
			info.Holding = None
			self.updateFillState(machine, 1)
			product = self.Products[productIndex]
			if product.Cap is not None or product.Rings[1] is not None:
				myPrint("Fed a non trivial product to RS! %s" % product, flush = FlushPrint)
			self.destroyProduct(productIndex)
			info.Location = clingo.Function(parameters[0][0], parameters[0][1])
			score = 2
		elif task == "getProduct":
			machineName = parameters[0][1][1]
			machine = self.Machines[machineName]
			productIndex = machine.Storing
			machine.Storing = None
			info.Holding = productIndex
			info.Location = clingo.Function(parameters[0][0], parameters[0][1])
		elif task == "mountRing":
			machine = parameters[0][1][1]
			order = int(parameters[1])
			qty = int(parameters[2])
			ring = int(parameters[3])
			productIndex = info.Holding
			info.Holding = None
			self.mountRing(machine, productIndex, order, qty, ring)
			info.Location = clingo.Function(parameters[0][0], parameters[0][1])
			if ring == 1:
				score = 10
			elif ring == 2:
				score = 20
			elif ring == 3:
				score = 50
		elif task == "mountCap":
			machine = parameters[0][1][1]
			order = int(parameters[1])
			qty = int(parameters[2])
			productIndex = info.Holding
			info.Holding = None
			self.mountCap(machine, productIndex, order, qty)
			info.Location = clingo.Function(parameters[0][0], parameters[0][1])
			score = 10
		elif task == "deliver" or task == "lateDeliver":
			machine = parameters[0][1][1]
			order = int(parameters[1])
			qty = int(parameters[2])
			productIndex = info.Holding
			info.Holding = None
			self.deliver(machine, productIndex, order, qty)
			info.Location = clingo.Function(parameters[0][0], parameters[0][1])
			if task == "lateDeliver":
				score = 5
			else:
				score = 20
		elif task == "goto":
			if type(parameters[0]) is tuple:
				#The normal case: Tuple ("m", ["CS", <machine>, <side>])
				info.Location = clingo.Function(parameters[0][0], parameters[0][1])
			else:
				#A goto to "ins-out" doesn't make sense, but this does not hinder the solver to try so.
				info.Location = clingo.Function(parameters[0])
		entry.Score = score
		plan.Score += score

	def spawnProduct(self, base):
		if len(self.Products) == maxProducts():
			myPrint("Trying to spawn a new product, but there already the maximum of products in use, increase MaxProducts!", flush = FlushPrint)
			sys.exit()
		self.Products.append(Product(base))
		self.MaxProductCount = max(self.MaxProductCount, len(self.Products))
		return len(self.Products) - 1

	def destroyProduct(self, index):
		self.Products.pop(index)

		for robot in self.Robots:
			info = self.Robots[robot]
			if info.Holding is not None and info.Holding >= index:
				info.Holding = info.Holding - 1

		for name in self.Machines:
			machine = self.Machines[name]
			if machine.Storing is not None and machine.Storing >= index:
				machine.Storing = machine.Storing - 1

	def mountRing(self, machineName, productIndex, order, qty, ring):
		self.machineProcess(machineName, productIndex)
		if ring == 1:
			ringColor = self.OrdersWithTasks[(order, 1)].Info.Ring1
		elif ring == 2:
			ringColor = self.OrdersWithTasks[(order, 1)].Info.Ring2
		elif ring == 3:
			ringColor = self.OrdersWithTasks[(order, 1)].Info.Ring3
		for info in self.RingColors:
			if info.Color == ringColor:
				cost = info.Cost
				break
		self.updateFillState(machineName, -cost)
		if self.Products[productIndex].Rings[ring-1] is not None:
			myPrint("Overwriting a ring on a product! This should not happen! Product: %s Ring: %d New Color: %s" % (self.Products[productIndex], ring-1, str(ringColor)), flush = FlushPrint)
		self.Products[productIndex].Rings[ring-1] = ringColor
		if self.ModifyCTL:
			self.CTL.release_external(self.OrdersWithTasks[(order, qty)].RingTask[ring])

	def mountCap(self, machineName, productIndex, order, qty):
		self.machineProcess(machineName, productIndex)
		self.updatePrepareState(machineName, False)
		self.Products[productIndex].Cap = self.OrdersWithTasks[(order, 1)].Info.Cap
		if self.ModifyCTL:
			self.CTL.release_external(self.OrdersWithTasks[(order, qty)].CapTask)

	def deliver(self, machineName, productIndex, order, qty):
		if self.ModifyCTL:
			for external in self.OrdersWithTasks[(order, qty)].DeliveryTasks:
				self.CTL.release_external(external)
		self.destroyProduct(productIndex)

	def machineProcess(self, machineName, productIndex):
		machine = self.Machines[machineName]
		machine.Storing = productIndex
		machine.Working = machineWorkingDuration(machineName)

	def onModel(self, model):
		self.LastModel = model.symbols(True)

	def solve(self):
		if StepSeconds is None or StepSeconds < 0:
			self.CTL.solve(self.onModel)
		else:
			future = self.CTL.solve_async(self.onModel)
			if not future.wait(StepSeconds):
				future.cancel()
				myPrint("Interrupt solving.", flush = FlushPrint)
				future.get()
			else:
				myPrint("Solving finished.", flush = FlushPrint)

	def spawnZone(self, zone):
		myPrint("Add zone " + str(zone) + " to explore.", flush = FlushPrint)
		if self.ModifyCTL:
			self.CTL.assign_external(zoneLocationExternal(zone), True)
			self.CTL.assign_external(zoneTaskExternal(zone), True)

	def releaseZone(self, zone, remove = False):
		if self.ModifyCTL:
			self.CTL.release_external(zoneLocationExternal(zone))
			self.CTL.release_external(zoneTaskExternal(zone))

		if remove:
			myPrint("Remove zone " + str(zone) + " from the zones to explore.", flush = FlushPrint)
			self.ZonesToExplore.remove(zone)

	def setRingColor(self, ring):
		myPrint("Set Ring-Color: " + str(ring), flush = FlushPrint)
		self.CTL.ground([("setRingInfo", [ring.Color, ring.Cost, ring.Machine])])

		for machine in ["RS1", "RS2"]:
			self.CTL.release_external(clingo.Function("ringStationAssignment", [machine, ring.Color]))
		for cost in [0,1,2]:
			self.CTL.release_external(clingo.Function("ringColorCost", [ring.Color, cost]));

	def addOrder(self, info):
		myPrint("Add Order: " + str(info), flush = FlushPrint)
		self.CTL.ground([("newOrder", [info.Number, info.Quantity, info.Base, info.Cap, info.Ring1, info.Ring2, info.Ring3, toASP(info.DelBegin), toASP(info.DelEnd)])])

		tasks = []
		for qty in range(1, info.Quantity + 1):
			order = Order(info)
			self.OrdersWithTasks[(info.Number, qty)] = order
			#myPrint("Qty " + str(qty) + ": " + str(order) + " | " + str(self.OrdersWithTasks), flush = FlushPrint)
			order.CapTask = self.capTaskExternal(info.Number, qty, info.Cap)
			tasks.append(order.CapTask)
			if info.Ring1 != "none":
				order.RingTask[1] = self.ringTaskExternal(info.Number, qty, 1, info.Ring1)
				tasks.append(order.RingTask[1])
			if info.Ring2 != "none":
				order.RingTask[2] = self.ringTaskExternal(info.Number, qty, 2, info.Ring2)
				tasks.append(order.RingTask[2])
			if info.Ring3 != "none":
				order.RingTask[3] = self.ringTaskExternal(info.Number, qty, 3, info.Ring3)
				tasks.append(order.RingTask[3])
			order.DeliveryTasks[0] = deliveryTaskExternal(info.Number, qty)
			order.DeliveryTasks[1] = lateDeliveryTaskExternal(info.Number, qty)
			tasks.extend(order.DeliveryTasks)

			#myPrint("Qty " + str(qty) + ": " + str(order) + " | " + str(self.OrdersWithTasks), flush = FlushPrint)

		#myPrint("Tasks " + str(tasks), flush = FlushPrint)
		for task in tasks:
			#myPrint("Task " + str(task), flush = FlushPrint)
			self.CTL.assign_external(task, True)

		for color in ["BLACK", "RED", "SILVER"]:
			self.CTL.release_external(clingo.Function("base", [info.Number, color]))

		qtyRange = range(1, maxQuantity() + 1)

		for color in ["BLACK", "GREY"]:
			self.CTL.release_external(clingo.Function("cap", [info.Number, color]))
			for qty in qtyRange:
				task = self.capTaskExternal(info.Number, qty, color)
				if task not in tasks:
					self.CTL.release_external(task)

		for color in ["BLUE", "YELLOW", "ORANGE", "GREEN"]:
			for ring in [1,2,3]:
				self.CTL.release_external(clingo.Function("ring", [info.Number, ring, color]))
				for qty in qtyRange:
					task = self.ringTaskExternal(info.Number, qty, ring, color)
					if task not in tasks:
						self.CTL.release_external(task)

		for qty in qtyRange:
			task = deliveryTaskExternal(info.Number, qty)
			if task not in tasks:
				self.CTL.release_external(task)
			task = lateDeliveryTaskExternal(info.Number, qty)
			if task not in tasks:
				self.CTL.release_external(task)

		for gt in range(minDeliveryTime(), maxDeliveryTime() + 1):
			self.CTL.release_external(clingo.Function("orderBegin", [info.Number, gt]))
			self.CTL.release_external(clingo.Function("orderDeadline", [info.Number, gt]))

	def capTaskExternal(self, order, qty, color):
		return taskExternal(clingo.Function("mountCap", [taskLocation(self.CapColors[color]), order, qty]))

	def ringTaskExternal(self, order, qty, ring, color):
		for ringInfo in self.RingColors:
			if ringInfo.Color == color:
				machine = ringInfo.Machine
				break
		return taskExternal(clingo.Function("mountRing", [taskLocation(machine), order, qty, ring]))

	def productExternal(self, productIndex):
		return clingo.Function("product", [productIndex])

def main(ctl):
	(zonesToExplore, ringColors, orders, machineFailures) = generateInstance()

	control = Control(ctl, zonesToExplore, ringColors, orders, machineFailures)
	control.run()
	sys.stdout.flush()

#end.
