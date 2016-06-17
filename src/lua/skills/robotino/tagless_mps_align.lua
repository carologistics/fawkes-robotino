
----------------------------------------------------------------------------
--  tagless_mps_align.lua - Align to a line detected by laserlines plugin
--
--  Created: Mon Jun 6 2016
--  Copyright  2016  David Schmidt
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
name               = "tagless_mps_align"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"motor_move"}
depends_interfaces = {
   {v = "line1", type="LaserLineInterface", id="/laser-lines/1"},
   {v = "line2", type="LaserLineInterface", id="/laser-lines/2"},
   {v = "line3", type="LaserLineInterface", id="/laser-lines/3"},
   {v = "line4", type="LaserLineInterface", id="/laser-lines/4"},
   {v = "line5", type="LaserLineInterface", id="/laser-lines/5"},
   {v = "line6", type="LaserLineInterface", id="/laser-lines/6"},
   {v = "line7", type="LaserLineInterface", id="/laser-lines/7"},
   {v = "line8", type="LaserLineInterface", id="/laser-lines/8"}
}

documentation      = [==[Align in front of a mps using laserlines instead of tags.

@param ori    angle we want to pose related to the mps, default is facing mps, ori=math.pi/2
@param x1     four coordinates specifying a rectangle in which we want to look for the mps. Default is whole playing field
@param x2
@param y1
@param y2

]==]

-- Initialize as skill module
skillenv.skill_module(_M)

-- Constants
MIN_VIS_HIST = -50	-- negative value, because noisy lines can be approached and we hope for better laserline visibility after approach
DIST_TO_MPS = 0.15+0.46/2
X1=-6.
X2=6.
Y1=0.
Y2=6.

-- Variables
local tfm = require("tf_module")

local map_frame = "/map"

local lines = {
   line1,
   line2,
   line3,
   line4,
   line5,
   line6,
   line7,
   line8
}

local candidates = {}

-- Functions
function visible_and_writer(self)
   for _,o in ipairs(lines) do
      if o:visibility_history() >= MIN_VIS_HIST and o:has_writer() then
         if inside_rectangle(o,self) then
            return true
         end
      end
   end
   return false
end

function get_best_line(line_list)
   local best_line
   local min_dist=100
   for _,o in ipairs(line_list) do
      x,y,ori=get_position_infront_line(o)
      if math.sqrt(x*x+y*y)<min_dist then
         best_line=o
         min_dist=math.sqrt(x*x+y*y)
      end
   end
   return best_line
end

function get_position_infront_line(line)   -- calculate a point infront of the detected laserline
   local a={line:end_point_1(0),line:end_point_1(1)}
   local b={line:end_point_2(0),line:end_point_2(1)}
   local m={(a[1]+b[1])/2,(a[2]+b[2])/2}
   normal_ori=math.pi+line:bearing()
   target={m[1]+math.cos(normal_ori)*DIST_TO_MPS,m[2]+math.sin(normal_ori)*DIST_TO_MPS}
   target_in_base_link = tfm.transform({x=target[1], y=target[2], ori=normal_ori-math.pi}, line:frame_id(), "/base_link")

   return target_in_base_link.x, target_in_base_link.y, target_in_base_link.ori
end

function inside_rectangle(line,self)   --check if the midpoint of the line is inside the given rectangle
   local a={line:end_point_1(0),line:end_point_1(1)}
   local b={line:end_point_2(0),line:end_point_2(1)}
   local m={(a[1]+b[1])/2,(a[2]+b[2])/2}
   local m_in_base_link = tfm.transform({x=m[1], y=m[2], ori=0.}, line:frame_id(), map_frame)
   m={m_in_base_link.x,m_in_base_link.y}
   if self.fsm.vars.x1 > m[1] then
      return false
   end
   if self.fsm.vars.x2 < m[1] then
      return false
   end
   if self.fsm.vars.y1 > m[2] then
      return false
   end
   if self.fsm.vars.y2 < m[2] then
      return false
   end
   return true
end

-- States
fsm:define_states{
   export_to=_M,
   closure={visible_and_writer=visible_and_writer},
   {"INIT", JumpState},
   {"DETECT_LINE", JumpState},
   {"ALIGN", SkillJumpState, skills={{"motor_move"}}, final_to="DETECT_LINE", fail_to="FAILED"},
}

-- Transitions
fsm:add_transitions {
   {"INIT", "DETECT_LINE", cond=visible_and_writer, desc="initialized"},
   {"INIT", "FAILED", cond=true, desc="no writer or vis_hist too low"},
   {"DETECT_LINE", "ALIGN", cond="vars.line_detected"},
   {"DETECT_LINE", "DETECT_LINE", cond=visible_and_writer, desc="there still might be a laserline with low visibility"},
   {"DETECT_LINE", "FAILED", cond="true"},
   {"ALIGN", "FINAL", cond="vars.tries>=3"},
}

function INIT:init()
   self.fsm.vars.tries = 0
   self.fsm.vars.detect_tries = 0
   self.fsm.vars.ori = self.fsm.vars.ori or math.pi/2
   self.fsm.vars.x1 = self.fsm.vars.x1 or X1
   self.fsm.vars.x2 = self.fsm.vars.x2 or X2
   self.fsm.vars.y1 = self.fsm.vars.y1 or Y1
   self.fsm.vars.y2 = self.fsm.vars.y2 or Y2
end

function DETECT_LINE:init()
   local global_line_pos = {}
   local candidates = {}
   self.fsm.vars.line_detected=false

   -- Gather all lines with high visiblity history
   for _,o in ipairs(lines) do
      if o:visibility_history() >= math.max(10-2*self.fsm.vars.detect_tries,1) then
         if inside_rectangle(o,self) then
            table.insert(candidates, o)
            self.fsm.vars.line_detected=true
         end
      end
   end

   if self.fsm.vars.line_detected then
      local best_line = get_best_line(candidates)
      self.fsm.vars.x_to_drive, self.fsm.vars.y_to_drive, self.fsm.vars.ori_to_drive = get_position_infront_line(best_line)
   else
      printf("no line detected with min. visibility:%i",math.max(10-2*self.fsm.vars.detect_tries,1))
   end
   self.fsm.vars.detect_tries=self.fsm.vars.detect_tries+1
end

function ALIGN:init()
   self.args["motor_move"] = {x = self.fsm.vars.x_to_drive, y = self.fsm.vars.y_to_drive, ori = self.fsm.vars.ori_to_drive,
				vel_trans = 0.2,
				tolerance = { x=0.002, y=0.002, ori=0.01 }}
   self.fsm.vars.tries = self.fsm.vars.tries + 1
   self.fsm.vars.detect_tries = 0
end
