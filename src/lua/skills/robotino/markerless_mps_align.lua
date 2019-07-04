----------------------------------------------------------------------------
--  markerless_mps_align.lua
--
--  Created: Thu Aug 14 14:32:47 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
--             2015  Tobias Neumann
--                   Johannes Rothe
--                   Nicolas Limpert
--             2016  Victor Mataré
--             2019  Morian Sonnet
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
name               = "markerless_mps_align"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = { "motor_move" ,"gripper_commands"}
depends_interfaces = {
   {v = "line1", type="LaserLineInterface", id="/laser-lines/1"},
   {v = "line2", type="LaserLineInterface", id="/laser-lines/2"},
   {v = "line3", type="LaserLineInterface", id="/laser-lines/3"},
   {v = "line4", type="LaserLineInterface", id="/laser-lines/4"},
   {v = "line5", type="LaserLineInterface", id="/laser-lines/5"},
   {v = "line6", type="LaserLineInterface", id="/laser-lines/6"},
   {v = "line7", type="LaserLineInterface", id="/laser-lines/7"},
   {v = "line8", type="LaserLineInterface", id="/laser-lines/8"},
   {v = "line1_avg", type="LaserLineInterface", id="/laser-lines/1/moving_avg"},
   {v = "line2_avg", type="LaserLineInterface", id="/laser-lines/2/moving_avg"},
   {v = "line3_avg", type="LaserLineInterface", id="/laser-lines/3/moving_avg"},
   {v = "line4_avg", type="LaserLineInterface", id="/laser-lines/4/moving_avg"},
   {v = "line5_avg", type="LaserLineInterface", id="/laser-lines/5/moving_avg"},
   {v = "line6_avg", type="LaserLineInterface", id="/laser-lines/6/moving_avg"},
   {v = "line7_avg", type="LaserLineInterface", id="/laser-lines/7/moving_avg"},
   {v = "line8_avg", type="LaserLineInterface", id="/laser-lines/8/moving_avg"},
   {v = "laserline_switch", type = "SwitchInterface", id="laser-lines"},
}

documentation      = [==[Align precisely at the given coordinates, relative to the center of an MPS
@param x X offset, i.e. distance from the MPS in meters.
@param y (optional) Y offset from the center of the MPS. Defaults to 0.
]==]

local pre_conveyor_pose = { x = 0.05, y = 0.0, z = 0.045 }

-- Initialize as skill module
skillenv.skill_module(_M)

local tfm = require("fawkes.tfutils")
local llutils = require("fawkes.laser-lines_utils")

-- Tunables
local MIN_VIS_HIST_LINE=5 --15
local MAX_ORI=15
local MAX_TRIES=3

local LINE_LENGTH_MIN=0.64
local LINE_LENGTH_MAX=0.71

local EXTRA_X_FAST_OFFSET=0.1

local TURN_MOVES={
   { ori = math.pi},
   { ori = -math.pi/2},
   { ori = -math.pi}
}

function find_line(lines, prealigned)
  if prealigned == nil then
    prealigned = false
  end
  local closest_ll = nil
  local min_distance = math.huge
  for line_id,line in pairs(lines) do
    local center = llutils.center(line)
    local distance = math.sqrt(math.pow(center.x,2),math.pow(center.y,2))
    local ori = math.deg(math.atan2(center.y, center.x))
    if prealigned then
       ori = ori +math.deg(math.atan2(fsm.vars.y, fsm.vars.x))
    end
    
    if math.abs(ori) < MAX_ORI then -- approximately in front of us
      if line:visibility_history() >= MIN_VIS_HIST_LINE then -- this laser line is not stale
        if distance < min_distance then
          if line:length() > LINE_LENGTH_MIN and line:length() < LINE_LENGTH_MAX then
            closest_ll = line
            min_distance = distance
          end
        end
      end
    end
  end
  return closest_ll
end





fsm:define_states{ export_to=_M, closure={
      pose_error=pose_error, tag_visible=tag_visible, MIN_VIS_HIST_TAG=MIN_VIS_HIST_TAG, MAX_TRIES=MAX_TRIES,
      want_search=want_search, line_visible=line_visible },
   {"INIT",                   JumpState},
   {"GRIPPER_PRE_CONVEYOR",   SkillJumpState, skills={{"gripper_commands"}}, final_to="FIND_LINE", fail_to="FIND_LINE"},
   {"FIND_LINE",             JumpState},
   {"NO_LINE",                JumpState},
   {"SEARCH_LINE",        SkillJumpState, skills={{"motor_move"}}, final_to="FIND_LINE", fail_to="FIND_LINE"},
   {"ALIGN_FAST",             SkillJumpState, skills={{"motor_move"}}, final_to="FIND_AVG_LINE", fail_to="FAILED"},
   {"FIND_AVG_LINE",         JumpState},
   {"ALIGN_PRECISE",          SkillJumpState, skills={{"motor_move"}}, final_to="ALIGN_TURN", fail_to="ALIGN_FAST"},
   {"ALIGN_TURN",             SkillJumpState, skills={{"motor_move"}}, final_to="FINAL", fail_to="FAILED"}
}

fsm:add_transitions{
   {"INIT",          "FAILED",          precond="not vars.x", desc="x argument missing"},
   {"INIT",          "GRIPPER_PRE_CONVEYOR",       cond=true},

   {"FIND_LINE",   "ALIGN_FAST",       cond="self.fsm.vars.found_line"},
   {"FIND_LINE",   "NO_LINE",          timeout=2, desc="cannot find line"},

   {"FIND_AVG_LINE", "ALIGN_PRECISE",  timeout=0.5},
   {"FIND_AVG_LINE", "FIND_LINE", cond="not self.fsm.vars.found_line"},

   {"NO_LINE",       "FAILED",          cond="vars.tried_searching > MAX_TRIES"},
   {"NO_LINE",       "SEARCH_LINE", cond=true},

   {"ALIGN_FAST",    "FAILED",          cond="vars.align_attempts >= 3"}
}

function INIT:init()
   laserline_switch:msgq_enqueue(laserline_switch.EnableSwitchMessage:new())

   self.fsm.vars.y   = self.fsm.vars.y   or 0
   self.fsm.vars.ori = self.fsm.vars.ori or 0

   self.fsm.vars.lines = {}
   self.fsm.vars.lines[line1:id()] = line1
   self.fsm.vars.lines[line2:id()] = line2
   self.fsm.vars.lines[line3:id()] = line3
   self.fsm.vars.lines[line4:id()] = line4
   self.fsm.vars.lines[line5:id()] = line5
   self.fsm.vars.lines[line6:id()] = line6
   self.fsm.vars.lines[line7:id()] = line7
   self.fsm.vars.lines[line8:id()] = line8

   self.fsm.vars.lines_avg = {}
   self.fsm.vars.lines_avg[line1_avg:id()] = line1_avg
   self.fsm.vars.lines_avg[line2_avg:id()] = line2_avg
   self.fsm.vars.lines_avg[line3_avg:id()] = line3_avg
   self.fsm.vars.lines_avg[line4_avg:id()] = line4_avg
   self.fsm.vars.lines_avg[line5_avg:id()] = line5_avg
   self.fsm.vars.lines_avg[line6_avg:id()] = line6_avg
   self.fsm.vars.lines_avg[line7_avg:id()] = line7_avg
   self.fsm.vars.lines_avg[line8_avg:id()] = line8_avg


   self.fsm.vars.tried_searching = 0
end

function FIND_LINE:init()
   self.fsm.vars.align_attempts = 0
   self.fsm.vars.tried_searching = self.fsm.vars.tried_searching + 1
end


function FIND_LINE:loop()
   self.fsm.vars.found_line = find_line(self.fsm.vars.lines, self.fsm.vars.prealigned)
end


--function SEARCH_LINE:init()
--end

function GRIPPER_PRE_CONVEYOR:init()
  self.args["gripper_commands"] = pre_conveyor_pose
  self.args["gripper_commands"].command = "MOVEABS"
  self.args["gripper_commands"].wait = false
end

function ALIGN_FAST:init()
   local center = llutils.center(self.fsm.vars.found_line)
   printf("center l: %f, %f, %f", center.x, center.y, center.ori)
   local center_bl = tfm.transform(center, "/base_laser", "/base_link")
   local p = llutils.point_in_front(center_bl, self.fsm.vars.x+EXTRA_X_FAST_OFFSET)
   
   printf("p    : %f %f %f", p.x, p.y, p.ori)

   self.args["motor_move"] = {
      x = p.x+math.sin(p.ori)*self.fsm.vars.y,
      y = p.y+math.cos(p.ori)*self.fsm.vars.y,
      ori = p.ori,
      tolerance = { x=0.05, y=0.03, ori=0.03 }
   }
   self.fsm.vars.prealigned = true
end


function FIND_AVG_LINE:loop()
  self.fsm.vars.found_line = find_line(self.fsm.vars.lines, self.fsm.vars.prealigned)
end


function ALIGN_PRECISE:init()
   local center = llutils.center(self.fsm.vars.found_line)
   printf("center l avg: %f, %f, %f", center.x, center.y, center.ori)
   local center_bl = tfm.transform(center, "/base_laser", "/base_link")
   local p = llutils.point_in_front(center_bl, self.fsm.vars.x)
   
   printf("p    avg : %f %f %f", p.x, p.y, p.ori)

   self.args["motor_move"] = {
      x = p.x+math.sin(p.ori)*self.fsm.vars.y,
      y = p.y+math.cos(p.ori)*self.fsm.vars.y,
      ori = p.ori,
   }
end


function ALIGN_TURN:init()
   self.args["motor_move"] = {
      ori = self.fsm.vars.ori
   }
end


