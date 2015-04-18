
----------------------------------------------------------------------------
--  goto.lua - generic global goto
--
--  Created: Thu Aug 14 14:32:47 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
--
----------------------------------------------------------------------------

--  This program is free software; you can redistribute it and/or modify
--  it under the terms of the GNU General Public License as published by
--  the Free Software Foundation; either version 2 of the License, or
--  (at your option) any later version.
--
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU Library General Public License for more details.
--
--  Read the full text in the LICENSE.GPL file in the doc directory.

-- Initialize module
module(..., skillenv.module_init)

-- Crucial skill information
name               = "production_agent"
fsm                = SkillHSM:new{name=name, start="DELIVER_P", debug=false}
depends_skills     = {"deliver_p", "produce_p", "produce_s2", "produce_s1", "get_s0", "recycle"}
depends_interfaces = nil

[[
lua 1x1 :D

t = {{pos =1}, {pos=2},{pos=3}}
for i,e in ipairs(t) do print(i,e.pos) end

table.insert (t, {pos=4})

t[2].pos =5


r={}
r[2] = {}
r[2].pos = 5

for k,v in pairs(r) do print(k,tostring(v)) end


r["foo"] = {pos=2}

]]

documentation      = [==[Basic Agent to decide what action to persue next as a Robot trying to deliver P
needed skills & spezification:

	- transport_puck:
		fetch puck at location fsm.vars.args[1] and transport it to location fsm.vars.args[2], which is some kind of machine, 
				fsm.vars.args[3] true if puck is already grabed, false if needed to drive to first location 
				
		behavoiur expectations:
			- if args[3] = false (true means puck is already grabed)
				- drive to location fsm.vars.args[1]
				- fetch_puck
			 
			- drive carefully to fsm.vars.args[2] 
			- put puck under rfid-reader and stop
			- 
			

	- exit_machine:
		move backwards ~30cm without moving the puck

	-wait_4_green:
		wait until green light starts --> success
		if red light starts (OutOfOrder) --> fail
		if more than 60 sec pass by -->fail

	-green_yellow:
		
		if green and yellow both are activated 	--> success
		if yellow is flashing 			--> fail
		else 					--> success


	- move_puck_from_machine_to_machineField


variables: OUT OF DATE
	-machine (1-10):
		fsm.vars.machine["m1"].possibleType  		€ {"M1"}, {"M2"}, {"M3"}, {"M1","M2","M3"},"M1 or M2"}
		fsm.vars.machine["m1"].processing 		€ {false, "puck1"-"puck25"}
		fsm.vars.machine["m1"].processingSince		€ {nil, 0:00-15:00  (min)} 
		fsm.vars.machine["m1"].outOfOrder		€ {true, false}
		fsm.vars.machine["m1"].processingTime		€ {nil, 3-60 (sec)}
		fsm.vars.machine["m1"].alreadyConsumed		€ {nil, S0,S1,S2}
		fsm.vars.machine["m1"].neededGoods		€ {unknown, S0,S1,S2}
	-puck (1-25):
		fsm.vars.material["puck1"].location 		€ {loc[1-20] & loc[24]& loc[29]} loc[1-10]= m1-m10 = machine, loc[11-20]= fm1-fm10 = machineField, 
												 loc[24]= is = Input store, loc[29] = moving
		fsm.vars.material["puck1"].materialType		€ {"S0","S1","S2","P","consumed","junk","notingame"}


conventions:
-machine
--possibleTypeConvention:
	-> 1 = M1
	-> 2 = M2
	-> 3 = M3
	-> 4 = M1 or M2
	-> 5 = M1 or M3
	-> 6 = M2 or M3
	-> 7 = M1 or M2 or M3
--alreadConsumed/neededGoodsConvention
	-> 1 = S0
	-> 2 = S1
	-> 3 = S2
	-> 4 = S0 and S1
	-> 5 = S0 and S2
	-> 6 = S1 and S2
	-> 7 = S0 and S1 and S2 (only needed goods)
	-> nil = nil	(only consumend)
	-> -1 = unknown (only needed goods)

-puck
--locationConvention:
	-> 1-10  = m1-m10
	-> 11-20 = fm1-fm10
	-> 21    = InputStore




]==]

-- Constants
loc ={m1="m1",m2="m2",m3="m3",m4="m4",m5="m5",m6="m6",m7="m7",m8="m8",m9="m9",m10="m10",fm1="fm1",fm2="fm2",fm3="fm3",fm4="fm4",fm5="fm5",fm6="fm6",fm7="fm7",fm8="fm8",fm9="fm9",fm10="fm10",recycle1="recycle1",recycle2="recycle2",
teststation="teststation",inputstore="inputstore",deliverstation1="deliverstation1",deliverstation2="deliverstation2",deliverstation3="deliverstation3",deliverarea="deliverarea",moving="moving"}  
[[ 

loc[1-10] 	-> m1-m10	= machine 1-10 
loc[11-20] 	-> fm1-10 	= machineField
loc[21-22] 	-> rc1-2	= recycling unit
loc[23] 	-> ts		= test station
loc[24] 	-> is		= "blue" area (input store)
loc[25-27] 	-> ds1-ds3 	= delivery slot (specified)
loc[28] 	-> da 		= delivery area
loc[29] 	-> mo		= moving

]]

todo = {deliverP="deliverP", produceP="produceP",produceS2="procudeS2",produceS1="produceS1",getS0="getS0" ,getS1="getS1",getS2="getS2",checkTrafficLight="checkTrafficLight"}


retry = 2

-- Initialize as skill module
skillenv.skill_module(...)

-- stackfunctions
function stackPush(item)
	fsm.vars.stackSize = fsm.vars.stackSize +1
	fsm.vars.stack[fsm.vars.stackSize] = item
end

function stackPop()
 	if (fsm.vars.stackSize <= 0) then return nil end
   	local item = fsm.vars.stack[fsm.vars.stackSize]
   	fsm.vars.stackSize = fsm.vars.stackSize - 1
    	return item

end

function stackPeek()
    if (fsm.vars.stackSize <= 0) then return nil end
    return fsm.vars.stack[fsm.vars.stackSize]

end

-- usefull functions
function findpattern(text, pattern, start)
 return string.sub(text, string.find(text, pattern, start))
end

function table_contains(table, element)
  for _, value in pairs(table) do
    if value == element then
      return true
    end
  end
  return false
end

-- functions

function checklogical(item)
	local destination = item["destination"]

	if (item["todo"]==todo["getS2"]) then
		if table_contains(fsm.vars.machine[destination].possibleType , "M3") then
			return true
		else
			return false		
		end
	end 

	if (item["todo"]==todo["getS1"]) then
		if table_contains(fsm.vars.machine[destination].possibleType , "M2") then
			return true
		else
			return false		
		end
	end

	if (item["todo"]==todo["getS0"]) then
		if table_contains(fsm.vars.machine[destination].possibleType , "M1") then
			return true
		else
			return false		
		end
	end
	return true


end


function next_todo_is_deliverP()
	local item = stackPeek()
	if (todo["deliverP"] == item["todo"]) then
		return true
	end
	return false
end

function next_todo_is_produceP()
	local item = stackPeek()

	if (todo["procudeP"] == item["todo"]) then
		return true
	end
	return false
end





function p_exists()
--check if any puck flagged as p and not moving, if so, prepare for moving p to delivery gate 
	for i =1,25,1 do 
		local helpstring = "puck"..i
		if (( fsm.vars.material[helpstring].materialType == "P") and (fsm.vars.material[helpstring].location ~= loc["moving"])) then			
			fsm.vars.args[1] = fsm.vars.material[helpstring].location
			fsm.vars.args[2] = loc["deliverarea"]
			fsm.vars.args[3] = false
			fsm.vars.material[helpstring].location = loc["moving"]
			fsm.vars.currentPuck = helpstring
			return true
		end	
	end	
end

function p_doesnt_exist()
--check if there is no knowen p which is not moving, return true
	for i =1,25,1 do 
		local helpstring = "puck"..i
		if ( fsm.vars.material[helpstring].materialType == "P") then
			if (fsm.vars.material[helpstring].location ~= loc["moving"]) then
				return false
			end
		end
	end
	stackPush({todo=todo["procudeP"],from=nil,destination=nil,puckInRobot=false})
	return true

end

function deliver_failure_counter()
--counts how often deliver has failed, returns true as long as smaller or equal as constant retry
	if (fsm.vars.deliver_failure > (retry+1)) then
		return false
	end
	fsm.vars.deliver_failure = fsm.vars.deliver_failure+1
	return true
		
end

function deliver_failed()
--returns true, if deliver has failed too often, updates world knowledge
	if (fsm.vars.deliver_failure <= (retry+1)) then
		return false
	end
	fsm.vars.deliver_failure = 1
	fsm.vars.material[fsm.vars.currentPuck].materialType="notingame"
	fsm.vars.currentPuck=nil
	return true	
	
end


function p_delivered()
-- update world model and return true
	fsm.vars.deliver_failure = 1
	fsm.vars.material[fsm.vars.currentPuck].materialType="notingame"
	fsm.vars.currentPuck=nil

	return true
end

function exists_puck(materialtype, machineNr)
--if puck of specified typ exists, prepare args for transport_puck skill (bring puck to specified machine)
	for i =1,25,1 do 
		local helpstring = "puck"..i
		if (fsm.vars.material[helpstring].location ~= loc["moving"]) then
			if ( fsm.vars.material[helpstring].materialType == materialtype) then
				fsm.vars.currentPuck = helpstring
				fsm.vars.currentMachine = machineNr
				fsm.vars.args[1] = fsm.vars.material[helpstring].location
				fsm.vars.args[2] = loc[machineNr]
				fsm.vars.args[3] = false
				fsm.vars.material[helpstring].location = loc["moving"]
				return true
			end
		end
	end
	return false
end
[[
function p_producable()
--if only one transport to a machine is needed to produce P, then return true and transport_puck skill is prepared

	for i = 1,10,1 do
		helpstring = "m"..i
		if (not table_contains(fsm.vars.machine[helpstring].alreadyConsumed,"S2")) then
			if exists_puck("S2",helpstring) then --exists 'S2'?
				return true
			end
		end

		if (not table_contains(fsm.vars.machine[helpstring].alreadyConsumed,"S1")) then
			if exists_puck("S1",helpstring) then --exists 'S1'?
				return true
			end
		end

		if (not table_contains(fsm.vars.machine[helpstring].alreadyConsumed,"S0")) then
			if exists_puck("S0",helpstring) then --exists 'S0'?
				return true
			end
		end
	
	end
	return false
end
]]

function p_in_production()

	--if yellow light is falshing goto reading device
	stackPop()
	fsm.vars.material[fsm.vars.currentPuck].materialType = "P"
	fsm.vars.machine[fsm.vars.currentMachine].processing = fsm.vars.currentPuck	

return true
end

function prepare_deliver_p()

fsm.vars.args[1] = loc[fsm.vars.currentMachine]
fsm.vars.args[2] = loc["deliverarea"] 
fsm.vars.args[3] = true

fsm.vars.machine[fsm.vars.currentMachine].possibleType  	= {"M3"}
fsm.vars.machine[fsm.vars.currentMachine].processing 	= false
fsm.vars.machine[fsm.vars.currentMachine].alreadyConsumed 	= {}
fsm.vars.machine[fsm.vars.currentMachine].neededGoods	= {"S0", "S1", "S2"}	

return true

end
[[
function p_not_directly_produceable()
	for i = 1,10,1 do
		local helpstring = "m"..i
		if (fsm.vars.machine[helpstring].possibleType[1] == "M3") then -- machine is definitly M3
			if (table_contains(fsm.vars.machine[helpstring].neededGoods, "S2")) then
				stackPush({todo=todo["getS2"],from=nil,destination=loc[helpstring],puckInRobot=false})
			end
			if (table_contains(fsm.vars.machine[helpstring].neededGoods, "S1")) then
				stackPush({todo=todo["getS1"],from=nil,destination=loc[helpstring],puckInRobot=false})
			end
			if (table_contains(fsm.vars.machine[helpstring].neededGoods, "S0")) then
				stackPush({todo=todo["getS0"],from=nil,destination=loc[helpstring],puckInRobot=false})
			end
			return true
			
		end
	end

	for i = 1,10,1 do
		local helpstring = "m"..i
		if (table_contains(fsm.vars.machine[helpstring].possibleType, "M3" ) ) then --Machine could be M3
			if (#fsm.vars.machine[helpstring].alreadyConsumed == 0) then
				stackPush({todo=todo["getS2"],from=nil,destination=loc[helpstring],puckInRobot=false}) 
				stackPush({todo=todo["getS1"],from=nil,destination=loc[helpstring],puckInRobot=false})
				stackPush({todo=todo["getS0"],from=nil,destination=loc[helpstring],puckInRobot=false})
				return true
			end
			if (not table_contains(fsm.vars.machine[helpstring].alreadyConsumed, "S2")) then
				stackPush({todo=todo["getS2"],from=nil,destination=loc[helpstring],puckInRobot=false})
			end
			if (not table_contains(fsm.vars.machine[helpstring].alreadyConsumed, "S1")) then
				stackPush({todo=todo["getS1"],from=nil,destination=loc[helpstring],puckInRobot=false})
			end
			if (not table_contains(fsm.vars.machine[helpstring].alreadyConsumed, "S0")) then
				stackPush({todo=todo["getS0"],from=nil,destination=loc[helpstring],puckInRobot=false})
			end
			return true
	
		end
	end

	

	return false
end
]]

fsm:add_transitions{
	closure={motor=motor},
   	{"START","NEXT_TODO",cond = true},
	{"NEXT_TODO","P_EXISTS",cond=next_todo_is_deliverP},
	{"NEXT_TODO","PRODUCE_P",cond=next_todo_is_produceP},
	{"NEXT_TODO", "S0_EXISTS", cond=next_todo_is_get_S0}, --condition to be implemented
	{"NEXT_TODO", "CHECK_TRAFFIC_LIGHT", cond=next_todo_is_check_traffic_light} -- condition to be implemented
	{"NEXT_TODO", "S1_EXISTS", cond=next_todo_is_get_S1}, --condition to be implemented
	{"NEXT_TODO", "S2_EXISTS", cond=next_todo_is_get_S2}, --condition to be implemented

	--Possible to deliver P? If so, do it! Else push todo produce p to stack
   	{"P_EXISTS", "DELIVER_P", cond=p_exists},
	{"P_EXISTS", "NEXT_TODO", cond=p_doesnt_exist},
	{"DELIVER_P", "P_DELIVERED", skill=transport_puck, fail_to="DELIVER_P_FAILED"},
	{"DELIVER_P_FAILED", "DELIVER_P", cond=deliver_failure_counter},
   	{"DELIVER_P_FAILED", "P_EXISTS", cond=deliver_failed},
	{"P_DELIVERED", "EXIT_GATE", cond=p_delivered},
	{"EXIT_GATE", "P_EXISTS", skill=exit_machine, fail_to="MOTOR_FAIL"},
	
	--add everything needed to produce P to stack
	{"PRODUCE_P", "NEXT_TODO", cond=true},	

	--needs to be fixed
	--{"P_PRODUCABLE", "PRODUCE_P",cond=p_producable},
	--{"PRODUCE_P","CHECK_GREEN_YELLOW", skill=transport_puck, fail_to="TRANSPORT_FAIL"},
	--{"CHECK_GREEN_YELLOW","P_IN_PRODUCTION", skill = green_yellow ,fail_to = "GOTO_READING"},
	--{"P_IN_PRODUCTION","WAIT_4_P",cond=p_in_production},
	{"WAIT_4_P","PREPARE_DELIVER_P", skill = wait_4_green, fail_to="OUT_OF_ORDER"},  
	{"PREPARE_DELIVER_P","DELIVER_P", cond=prepare_deliver_p},

	--{"P_PRODUCABLE", "NEXT_TODO",cond=p_not_directly_produceable}, 

	{"S0_EXISTS", "TRANSPORT_PUCK", cond=S0_exists}, --condition to be implemented: set args, update underlying todo (if checkTrafficLight)
	{"S0_EXISTS", "NEXT_TODO", cond=S0_doesnt_exist}, --condition to be implemented: update todos

	{"S1_EXISTS", "TRANSPORT_PUCK", cond=S1_exists}, --condition to be implemented: set args, update underlying todo (if checkTrafficLight)
	{"S1_EXISTS", "NEXT_TODO", cond=S1_doesnt_exist}, --condition to be implemented: update todos

	{"S2_EXISTS", "TRANSPORT_PUCK", cond=S2_exists}, --condition to be implemented: set args, update underlying todo (if checkTrafficLight)
	{"S2_EXISTS", "NEXT_TODO", cond=S2_doesnt_exist}, --condition to be implemented: update todos



	{"TRANSPORT_PUCK", "NEXT_TODO", skill=transport_puck, fail_to="TRANSPORT_FAIL"},--

	{"CHECK_TRAFFIC_LIGHT","CHECK_RED",cond=true}, --dummy call for readability and order of what is checked first
	{"CHECK_RED", "OUT_OF_ORDER", skill=red_light, fail_to="CHECK_YELLOW_FLASHING"},
	{"CHECK_YELLOW_FLASHING", "YELLOW_FLASHING",skill=yellow_flashing, fail_to="CHECK_YELLOW_GREEN"},
	{"CHECK_YELLOW_GREEN", "YELLOW_GREEN",skill=yellow_green, fail_to="CHECK_GREEN"},
	{"CHECK_GREEN", "GREEN",skill=green, fail_to="CHECK_YELLOW"},
	{"CHECK_YELLOW", "YELLOW",skill=yellow, fail_to="TRAFFIC_LIGHT_FAIL"},

	{"YELLOW_FLASHING","GOTO_READING",cond=true}, --something is wrong with our world model, check what material you have there
	{"YELLOW_GREEN","CHECK_YELLOW_GREEN", wait_sec=1.0}, --always wait until product is finished
	{"GREEN","NEXT_TODO",cond=true}, --GREEN:init needs to update world knowledge and todos
	{"YELLOW","NEXT_TODO",cond=true}, --YELLOW:init needs to update world knowledge and todos
	 
}



function START:init()
--stack init

	fsm.vars.stack={}
	fsm.vars.stackSize=0
	stackPush({todo=todo["deliverP"],from=nil,destination=loc["deliverarea"],puckInRobot=false})
	

--machine init

	fsm.vars.machine={}


	for i =1,10,1 do 
		local	helpstring  = "m" .. i
		fsm.vars.machine[helpstring]={}
		fsm.vars.machine[helpstring].possibleType = {"M1", "M2", "M3"}
		fsm.vars.machine[helpstring].processing = false
		fsm.vars.machine[helpstring].processingSince = nil
		fsm.vars.machine[helpstring].outOfOrder = false
		fsm.vars.machine[helpstring].processingTime = nil
		fsm.vars.machine[helpstring].alreadyConsumed = {}
		fsm.vars.machine[helpstring].neededGoods = {"unknown"}
	end


--puck init
	fsm.vars.material={}

	for i =1,25,1 do 
		local	helpstring = "puck" .. i
		fsm.vars.material[helpstring].location = loc["inputstore"]
		fsm.vars.material[helpstring].materialType = "S0"

	end
--args init

	fsm.vars.args ={}
	fsm.vars.args[1]=nil
	fsm.vars.args[2]=nil
	fsm.vars.args[3]=false

--counter init
	fsm.vars.deliver_failure = 1
--variable init
	fsm.vars.currentPuck = nil
	fsm.vars.currentMachine = nil

	

end

function NEXT_TODO:init()
	-- check if todos are still plausible with current world knowledge
	local item = stackPeek()

	while not (checklogical(item)) do
			
		stackPop()
		--delete traffic light check underneath, too
		item = stackPeek()
		if (item["goal"]=="checkTrafficLight") then
			stackPop()
		end	
		
		item = stackPeek()
	end
end

function PRODUCE_P:init()
	--check what is needed to produce p and put it on stack
	stackPop() --remove todo produce p from stack, as when every todo we will now put on stack is fullfilled, we will have a p at hands
	
	--check if there is a machine known as exactly M3, then produce P with this machine 
	for i = 1,10,1 do
		local helpstring = "m"..i
		if (fsm.vars.machine[helpstring].possibleType[1] == "M3") then -- machine is definitly M3
			if (table_contains(fsm.vars.machine[helpstring].neededGoods, "S2")) then
				stackPush({todo=todo["checkLight"], machine=helpstring, material="update", expectation="green"})
				stackPush({todo=todo["getS2"],from=nil,destination=loc[helpstring],puckInRobot=false})
			end
			if (table_contains(fsm.vars.machine[helpstring].neededGoods, "S1")) then
				stackPush({todo=todo["checkLight"], machine=helpstring, material="update", expectation="yellow"})
				stackPush({todo=todo["getS1"],from=nil,destination=loc[helpstring],puckInRobot=false})
			end
			if (table_contains(fsm.vars.machine[helpstring].neededGoods, "S0")) then
				stackPush({todo=todo["checkLight"], machine=helpstring, material="update", expectation="yellow"})
				stackPush({todo=todo["getS0"],from=nil,destination=loc[helpstring],puckInRobot=false})
			end
			return true
			
		end
	end

	--if M3 is not exactly identified, try to produce a P with the first machine that could be M3
	-- (we will check in NEXT_TODO:init, if the todo on the stack is still plausible with the current knowledge before 
	-- trying to reach that todo)
	for i = 1,10,1 do
		local helpstring = "m"..i
		if (table_contains(fsm.vars.machine[helpstring].possibleType, "M3" ) ) then --Machine could be M3
			if (#fsm.vars.machine[helpstring].alreadyConsumed == 0) then
				stackPush({todo=todo["checkLight"], machine=helpstring, material="update", expectation="green"})
				stackPush({todo=todo["getS2"],from=nil,destination=loc[helpstring],puckInRobot=false})

				stackPush({todo=todo["checkLight"], machine=helpstring, material="update", expectation="yellow"}) 
				stackPush({todo=todo["getS1"],from=nil,destination=loc[helpstring],puckInRobot=false})

				stackPush({todo=todo["checkLight"], machine=helpstring, material="update", expectation="yellow"})
				stackPush({todo=todo["getS0"],from=nil,destination=loc[helpstring],puckInRobot=false})
				return true
			end
			if (not table_contains(fsm.vars.machine[helpstring].alreadyConsumed, "S2")) then
				stackPush({todo=todo["checkLight"], machine=helpstring, material="update", expectation="green"})
				stackPush({todo=todo["getS2"],from=nil,destination=loc[helpstring],puckInRobot=false})
			end
			if (not table_contains(fsm.vars.machine[helpstring].alreadyConsumed, "S1")) then
				stackPush({todo=todo["checkLight"], machine=helpstring, material="update", expectation="yellow"}) 
				stackPush({todo=todo["getS1"],from=nil,destination=loc[helpstring],puckInRobot=false})
			end
			if (not table_contains(fsm.vars.machine[helpstring].alreadyConsumed, "S0")) then
				stackPush({todo=todo["checkLight"], machine=helpstring, material="update", expectation="yellow"}) 
				stackPush({todo=todo["getS0"],from=nil,destination=loc[helpstring],puckInRobot=false})
			end
			return true
	
		end
	end

	

	return false
end

[[
	TODO: wenn todo vom stack geholt wird, müssen wir gucken, ob die destination noch sinn macht, wenn nicht löschen 
]]



