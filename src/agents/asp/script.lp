#script (python)

import clingo
import datetime
import sys

from collections import namedtuple

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

	temp.append(string)

	ret = []
	for s in temp:
		if s[0] == '"':
			ret.append(s[1:-1])
		else:
			ret.append(s)
	return ret

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
		return "Product(Base=%s, Rings=%s, Cap=%s)" % (self.Base, str(self.Rings), self.Cap)

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

		for file in ["defaultMap.lp", "delivery.lp", "facts.lp", "feedRS.lp", "functions.lp", "getBase.lp", "getProduct.lp", "goal.lp", "machines.lp", "mountCap.lp", "mountRing.lp", "orders.lp", "prepareCS.lp", "products.lp", "robots.lp", "state.lp"]:
			self.CTL.load(file)

		self.CapColors         = {capColor(clingo.Number(1)) : "CS1", capColor(clingo.Number(2)) : "CS2"}

		groundBegin = datetime.datetime.now()
		print("Ground base.", flush = True)
		self.CTL.ground([("base", [])])
		print("Ground team.", flush = True)
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
		print("Solve after init.", flush = True)
		self.CTL.solve()
		solveEnd = datetime.datetime.now()
		self.InitialSolveTime = solveEnd - solveBegin

		self.StartUpTime = self.GroundTime + self.InitialSolveTime

	def run(self):
		print("Begin with instance.", flush = True)
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

			while self.NextOrder < maxOrders() and gt == toASP(self.Orders[self.NextOrder].Spawn):
				order = self.Orders[self.NextOrder]
				self.addOrder(order)
				self.NextOrder = self.NextOrder + 1

			for t in range(self.LastGameTime, gt):
				self.CTL.release_external(currentTimeExternal(t))
			self.CTL.assign_external(currentTimeExternal(gt), True)
			self.LastGameTime = gt

			if self.FirstSolve:
				self.FirstSolve = False
				print("Add test instance.", flush = True)
				self.CTL.ground([("testInstance", [])])

			for external in externals:
				print("Unset external: " + str(external), flush = True)
				self.CTL.assign_external(external, False)
			externals = []

			for robot in self.Robots:
				info = self.Robots[robot]
				setLocation = True

				if info.Task is not None:
					remaining = info.Task[1] - gt
					if remaining < 0:
						print("Calculated remaining time is " + str(remaining) + "! This should not happen.", flush = True)
						self.endRobotTaskWithoutCheck(robot, info.Task[0], gt)
						info.Task = None
					elif remaining == 0:
						self.endRobotTaskWithoutCheck(robot, info.Task[0], gt)
						info.Task = None
					else:
						setLocation = False
						external = clingo.Function("robotDoing", [robot, info.Task[0], remaining])
						externals.append(external)
						print("Set robot task: " + str(external), flush = True)

				if setLocation:
					external = clingo.Function("robotLocation", [robot, info.Location, 0])
					externals.append(external)
					print("Set robot location: " + str(external), flush = True)

				if info.Holding is not None:
					external = clingo.Function("holding", [robot, self.productExternal(info.Holding), 0])
					externals.append(external)
					print("Set robot holding: " + str(external), flush = True)

				info.LastStep = -1

			for productIndex in range(0, len(self.Products)):
				product = self.Products[productIndex]
				productFunction = self.productExternal(productIndex)

				externals.append(clingo.Function("product", [productFunction]))
				externals.append(clingo.Function("productBase", [productFunction, product.Base]))
				print("Add product %d: %s" % (productIndex, product), flush = True)

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
					print("%s is for %d steps broken." % (name, machine.Broken), flush = True)
				if machine.Working is not None:
					externals.append(clingo.Function("processing", [name, self.productExternal(machine.Storing), machine.Working, 0]))
					print("%s is working on %d for %d steps." % (name, machine.Storing, machine.Working), flush = True)
				elif machine.Storing is not None:
					externals.append(clingo.Function("storing", [name, self.productExternal(machine.Storing), 0]))
					print("%s is storing %d." % (name, machine.Storing), flush = True)

			for external in externals:
				self.CTL.assign_external(external, True)

			print("Solve " + str(gt), flush = True)
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

		score = 0
		print("Plan:", flush = True)
		for robot in self.Plan:
			plan = self.Plan[robot]
			score = score + plan.Score
			print("  For Robot " + robot + " (" + str(plan.Score) + " Score):", flush = True)
			i = 1
			for entry in plan.Entries:
				print("    %2d: %s" % (i, str(entry)), flush = True)
				i = i + 1

		print("Overall score: " + str(score), flush = True)
		print("CS prepare state: CS1: %5s CS2: %5s" % (str(self.CapStations["CS1"]), str(self.CapStations["CS2"])), flush = True)
		print("RS fill state:    RS1: %5d RS2: %5d" % (self.RingStations["RS1"], self.RingStations["RS2"]), flush = True)
		print("Time for grounding base & team: %s" % str(self.GroundTime), flush = True)
		print("Time for solving base & team:   %s" % str(self.InitialSolveTime), flush = True)
		print("Overall startup time:           %s" % str(self.StartUpTime), flush = True)

	def update(self, step, gt):
		print("Update on basis of step: " + str(step), flush = True)
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
			print("Invalid fill state for %s: %d!" % (machine, newFillState), flush = True)
			sys.exit()
		print("Change fill state of " + str(machine) + " from " + str(oldFillState) + " to " + str(newFillState) + ".", flush = True)
		if self.ModifyCTL:
			self.CTL.assign_external(ringStationExternal(machine, oldFillState), False)
			self.CTL.assign_external(ringStationExternal(machine, newFillState), True)

	def updatePrepareState(self, machine, state):
		oldState = self.CapStations[machine]
		if state != oldState:
			print("Change " + str(machine) + " prepare state from " + str(oldState) + " to " + str(state) + ".", flush = True)
			self.CapStations[machine] = state
			if self.ModifyCTL:
				self.CTL.assign_external(capStationExternal(machine), state)
		else:
			print("Update prepare state, without update!", flush = True)

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
		entry.Score = score
		plan.Score += score

	def spawnProduct(self, base):
		if len(self.Products) == maxProducts():
			print("Trying to spawn a new product, but there already the maximum of products in use, increase MaxProducts!", flush = True)
			sys.exit()
		self.Products.append(Product(base))
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
		if StepSeconds is None:
			self.CTL.solve(self.onModel)
		else:
			future = self.CTL.solve_async(self.onModel)
			if not future.wait(StepSeconds):
				future.cancel()
				print("Interrupt solving.", flush = True)
				future.get()
			else:
				print("Solving finished.", flush = True)

	def spawnZone(self, zone):
		print("Add zone " + str(zone) + " to explore.", flush = True)
		if self.ModifyCTL:
			self.CTL.assign_external(zoneLocationExternal(zone), True)
			self.CTL.assign_external(zoneTaskExternal(zone), True)

	def releaseZone(self, zone, remove = False):
		if self.ModifyCTL:
			self.CTL.release_external(zoneLocationExternal(zone))
			self.CTL.release_external(zoneTaskExternal(zone))

		if remove:
			print("Remove zone " + str(zone) + " from the zones to explore.", flush = True)
			self.ZonesToExplore.remove(zone)

	def setRingColor(self, ring):
		print("Set Ring-Color: " + str(ring), flush = True)
		self.CTL.ground([("setRingInfo", [ring.Color, ring.Cost, ring.Machine])])

		for machine in ["RS1", "RS2"]:
			self.CTL.release_external(clingo.Function("ringStationAssignment", [machine, ring.Color]))
		for cost in [0,1,2]:
			self.CTL.release_external(clingo.Function("ringColorCost", [ring.Color, cost]));

	def addOrder(self, info):
		print("Add Order: " + str(info), flush = True)
		self.CTL.ground([("newOrder", [info.Number, info.Quantity, info.Base, info.Cap, info.Ring1, info.Ring2, info.Ring3, toASP(info.DelBegin), toASP(info.DelEnd)])])

		tasks = []
		for qty in range(1, info.Quantity + 1):
			order = Order(info)
			self.OrdersWithTasks[(info.Number, qty)] = order
			#print("Qty " + str(qty) + ": " + str(order) + " | " + str(self.OrdersWithTasks), flush = True)
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

			#print("Qty " + str(qty) + ": " + str(order) + " | " + str(self.OrdersWithTasks), flush = True)

		#print("Tasks " + str(tasks), flush = True)
		for task in tasks:
			#print("Task " + str(task), flush = True)
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

#end.
