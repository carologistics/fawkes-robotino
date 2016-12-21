#include "exploration.lp".
#include "script.lp".

#script (python)

MaxOrders = 0
MaxQuantity = 0
Res = 5
Horizon = 120
Start = 0
End = 0

StepSeconds = None

StartDelivery = 240
EndDelivery = 1140

MaxFeedRS = 2
MaxGetBase = 2
MaxGetProduct = 4
MaxPrepCS = 1
Robots = ["Tuvok", "T'Pol", "Spock"]

def generateInstance():
	return ([1,2,3,4,5,7,8,9,10,11,18,24], [], [], [])

#end.
