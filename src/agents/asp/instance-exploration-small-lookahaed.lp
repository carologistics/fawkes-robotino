#include "exploration.lp".
#include "script.lp".

#script (python)

MaxOrders = 0
MaxQuantity = 0
Res = 5
Horizon = 40
Start = 0
End = 180

StepSeconds = 7

StartDelivery = 240
EndDelivery = 1140

MaxFeedRS = 0
MaxGetBase = 0
MaxGetProduct = 0
MaxPrepCS = 0
Robots = ["Tuvok", "T'Pol", "Spock"]

def generateInstance():
	return ([1,2,3,4,5,7,8,9,10,11,18,24], [], [], [])

#end.
