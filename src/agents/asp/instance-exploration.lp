#include "exploration.lp".
#include "script.lp".

#script (python)

MaxOrders = 0
MaxQuantity = 0
Res = 5
Horizon = 120
Start = 0
End = 0

CancelSolveTime = 3

StartDelivery = 240
EndDelivery = 1140

MaxFeedRS = 2
MaxGetBase = 2
MaxGetProduct = 4
MaxPrepCS = 1
Robots = ["Tuvok"] #, "T'Pol", "Spock"]

def generateInstance():
	return ([1,2,4,5,6,7,8,9,10,11,14,24], [], [], [])

#end.

#program testInstance.

%:- not begin("Tuvok", explore( 4),  0).
%:- not begin("Tuvok", explore(11),  2).
%:- not begin("Tuvok", explore( 8),  4).
%:- not begin("Tuvok", explore( 5),  6).
%:- not begin("Tuvok", explore( 2),  8).
%:- not begin("Tuvok", explore( 6), 10).
%:- not begin("Tuvok", explore( 1), 12).
%:- not begin("Tuvok", explore(10), 14).
%:- not begin("Tuvok", explore(24), 16).
%:- not begin("Tuvok", explore( 9), 18).
%:- not begin("Tuvok", explore(14), 20).
%:- not begin("Tuvok", explore( 7), 22).
