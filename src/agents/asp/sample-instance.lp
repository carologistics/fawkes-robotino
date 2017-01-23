#include "defaultMap.lp".
#include "script.lp".

#script (python)

# Defines the orders, which are considered, that are [1,MaxOrders].
MaxOrders = 1
# Upper bound for the quantitiy of the orders.
MaxQuantity = 1
# The time resolution, how many real time seconds are one time step.
Res = 5
# How many real time seconds should be looked into the future.
Horizon = 300
# Starting point for the solving, given in real time seconds. Exploration phase is [0,240) and production [240,1140].
Start = 240
# End point for the solven, given in real time seconds.
End = 240

# How many seconds the solving is allowed to run (None = no time out) and also the step for real time seconds. The
# latter is set to 1 if StepSeconds is None. Use None for check instances and a multiple of Res for all other cases.
# While all values are valid, the resulting program run may not.
# Also possible are negative Values, their absolute value is used as real time seconds for the steps, but it does not
# cause a timeout for the solver.
StepSeconds = None

# The time frame for deliveries, i.e. the production phase. Only change this, when the rules are changed.
StartDelivery = 240
EndDelivery = 1140

# An upper bound for the products, that are alive at any given point in time, if there are more products than this value
# the test script will terminate and in a real scenario we have to work with a wrong world model.
# But here also, keep the value as low as possible, higher values result in more externals
MaxProducts = 5

# The names of the robots available. In the test script a robot is available from the beginning and there is no way to
# remove it.
Robots = ["Tuvok", "T'Pol"]

def generateInstance():
	# The returned instance is a tuple (Zones, RingInfo, Orders, MachineDownTime)
	# Zones is a list of zones to explore.
	# RingInfo is a list which defines the assignment of ring color to a machine and its cost.
	# Orders is a list of the orders, see OrderInfo in script.lp for more information on its parameters.
	#        Only orders with a number <= MaxOrders (see above) will be considered by the program.
	#        Also quantities > MaxQuantity are ignored.
	# MachineDownTime Is not implemented yet.
	return ([],
		[RingInfo("YELLOW", "RS2", 1), RingInfo("GREEN", "RS2", 0),
		 RingInfo("BLUE", "RS1", 2), RingInfo("ORANGE", "RS1", 0)],
		[OrderInfo(1, 1, "BLACK", "BLACK", "GREEN", "ORANGE", "none", 240, 1140, 240)],
		[])

#end.

% For check instances add the program testInstance and encode the solution to check in it.
% The encoding below looks a bit more complicated, because right now (Jan 2017) the default map is not yet encoded
% properly and also the task duration has to be measured, the values are guessed. And so we calculate the times for the
% task begin.
#program testInstance.
testTask("Tuvok", 1, prepareCS(L, 1),        L, 0) :- L = m("C", "CS2", "I").
testTask("Tuvok", 2, getProduct(L, 1),       L, W) :- L = m("C", "CS2", "O"), W = @machineWorkingDuration("CS2").

testTask("T'Pol", 1, getBase(L, "BLACK", 1), L, 0) :- L = m("C", "BS",  "O").
testTask("T'Pol", 2, mountRing(L, 1, 1, 1),  L, 0) :- L = m("C", "RS2", "I").
testTask("T'Pol", 3, getProduct(L, 1),       L, W) :- L = m("C", "RS2", "O"), W = @machineWorkingDuration("RS2").
testTask("T'Pol", 4, mountRing(L, 1, 1, 2),  L, 0) :- L = m("C", "RS1", "I").
testTask("T'Pol", 5, getProduct(L, 1),       L, W) :- L = m("C", "RS1", "O"), W = @machineWorkingDuration("RS1").
testTask("T'Pol", 6, mountCap(L, 1, 1),      L, 0) :- L = m("C", "CS2", "I").
testTask("T'Pol", 7, getProduct(L, 2),       L, W) :- L = m("C", "CS2", "O"), W = @machineWorkingDuration("CS2").
testTask("T'Pol", 8, deliver(L, 1, 1),       L, 0) :- L = m("C", "DS",  "I").

startTestTask(R, 1, 0, D) :- testTask(R, 1, Task, _, _), fullDuration(Task, "ins-out", D).
startTestTask(R, Number, Time, Duration) :- testTask(R, Number, Task, Location, Wait),
											testTask(R, Number-1, PrevTask, PrevLocation, _),
											startTestTask(R, Number-1, PrevStart, PrevDuration),
											fullDuration(Task, PrevLocation, Duration),
											Time = PrevStart + PrevDuration + Wait.

:- not begin(R, Task, Time), startTestTask(R, Number, Time, _), testTask(R, Number, Task, _, _).
