#script (python)

# The functions for the instances, which are normally provided by the C++ part.

def clingoToASP(timeSymbol):
	return toASP(timeSymbol.number)

def toASP(time):
	if time == 0:
		return 0
	ret = time // Res
	if (time % Res) * 2 >= Res:
		ret = ret + 1
	return max(ret, 1)

def robots():
	return Robots

def machineWorkingDuration(machine):
	if isinstance(machine, clingo.Symbol):
		machine = str(machine)[1:-1]
	if machine[:2] == "CS":
		return toASP(20)
	elif machine[:2] == "RS":
		return toASP(50)
	elif machine == "DS":
		return toASP(30)
	elif machine == "BS":
		return toASP(5)
	return toASP(1)

def maxOrders():
	return MaxOrders

def maxProducts():
	return MaxProducts

def maxQuantity():
	return MaxQuantity

def maxWorkingDuration():
	ret = 0
	for machine in ["BS", "DS", "RS1", "RS2", "CS1", "CS2"]:
		ret = max(ret, machineWorkingDuration(machine))
	return ret

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

def gameSteps():
	if StepSeconds is None:
		return 1
	elif StepSeconds < 0:
		return -StepSeconds
	return StepSeconds

def gameRange():
	ret = []
	for second in range(Start, End + 1, gameSteps()):
		step = toASP(second)
		if step not in ret:
			ret.append(step)
	return ret

def horizonSteps():
	return [0, toASP(Horizon) - 1]

def horizon():
	return toASP(Horizon)

#end.
