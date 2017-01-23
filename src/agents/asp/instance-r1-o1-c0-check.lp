#include "defaultMap.lp".
#include "script.lp".

#script (python)

MaxOrders = 1
MaxQuantity = 1
Res = 5
Horizon = 300
Start = 240
End = 240

StepSeconds = None

StartDelivery = 240
EndDelivery = 1140

MaxProducts = 5
Robots = ["Tuvok"]

def generateInstance():
	return ([],
		[RingInfo("YELLOW", "RS2", 1), RingInfo("GREEN", "RS2", 0),
		 RingInfo("BLUE", "RS1", 2), RingInfo("ORANGE", "RS1", 0)],
		[OrderInfo(1, 1, "BLACK", "BLACK", "none", "none", "none", 240, 1140, 240)],
		[])

#end.

#program testInstance.
testTask("Tuvok", 1, prepareCS(L),        L, 0) :- L = m("C", "CS2", "I").
testTask("Tuvok", 2, getProduct(L),       L, W) :- L = m("C", "CS2", "O"), W = @machineWorkingDuration("CS2").
testTask("Tuvok", 3, feedRS(L),           L, 0) :- L = m("C", "RS1", "I").
testTask("Tuvok", 4, getBase(L, "BLACK"), L, 0) :- L = m("C", "BS",  "O").
testTask("Tuvok", 5, mountCap(L, 1, 1),   L, 0) :- L = m("C", "CS2", "I").
testTask("Tuvok", 6, getProduct(L),       L, W) :- L = m("C", "CS2", "O"), W = @machineWorkingDuration("CS2").
testTask("Tuvok", 7, deliver(L, 1, 1),    L, 0) :- L = m("C", "DS",  "I").

startTestTask(R, 1, 0, D) :- testTask(R, 1, Task, _, _), fullDuration(Task, "ins-out", D).
startTestTask(R, Number, Time, Duration) :- testTask(R, Number, Task, Location, Wait),
											testTask(R, Number-1, PrevTask, PrevLocation, _),
											startTestTask(R, Number-1, PrevStart, PrevDuration),
											fullDuration(Task, PrevLocation, Duration),
											Time = PrevStart + PrevDuration + Wait.

:- not begin(R, Task, Time), startTestTask(R, Number, Time, _), testTask(R, Number, Task, _, _).
