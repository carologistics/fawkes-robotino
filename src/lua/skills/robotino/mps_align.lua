
----------------------------------------------------------------------------
--  align_mps.lua
--
--  Created: Thu Aug 14 14:32:47 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
--             2015  Tobias Neumann
--                   Johannes Rothe
--                   Nicolas Limpert
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
name               = "mps_align"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = { "align_tag", "motor_move", "check_tag" }
depends_interfaces = {
  --[[
  {v = "line1avg", type="LaserLineInterface", id="/laser-lines/1/moving_avg"},
  {v = "line2avg", type="LaserLineInterface", id="/laser-lines/2/moving_avg"},
  {v = "line3avg", type="LaserLineInterface", id="/laser-lines/3/moving_avg"},
  {v = "line4avg", type="LaserLineInterface", id="/laser-lines/4/moving_avg"},
  {v = "line5avg", type="LaserLineInterface", id="/laser-lines/5/moving_avg"},
  {v = "line6avg", type="LaserLineInterface", id="/laser-lines/6/moving_avg"},
  {v = "line7avg", type="LaserLineInterface", id="/laser-lines/7/moving_avg"},
  {v = "line8avg", type="LaserLineInterface", id="/laser-lines/8/moving_avg"},
  --]]
  ---[[
  {v = "line1", type="LaserLineInterface", id="/laser-lines/1"},
  {v = "line2", type="LaserLineInterface", id="/laser-lines/2"},
  {v = "line3", type="LaserLineInterface", id="/laser-lines/3"},
  {v = "line4", type="LaserLineInterface", id="/laser-lines/4"},
  {v = "line5", type="LaserLineInterface", id="/laser-lines/5"},
  {v = "line6", type="LaserLineInterface", id="/laser-lines/6"},
  {v = "line7", type="LaserLineInterface", id="/laser-lines/7"},
  {v = "line8", type="LaserLineInterface", id="/laser-lines/8"},
  --]]
}

documentation      = [==[ align_mps

                          This skill does:
                          - aligns to the machine via sensor information AND a optional offsets which are given as parameters 

                          @param tag_id     int     optional the tag_id for align_tag
                          @param x          float   the x offset after the alignmend
                          @param y          float   optional the y offset after the alignmend; TODO this just works within the AREA_LINE_ERR_Y
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

local tfm = require("tf_module")
local llutils = require("fawkes.laser-lines_utils")

-- Tunables
local TAG_X_ERR=0.02
local MIN_VIS_HIST=10
local TIMEOUT=4
local AREA_LINE_ERR_X=0.1
local AREA_LINE_ERR_Y=0.2
local AREA_LINE_ERR_ORI=0.2 --0.17
local LINE_TRIES=3

function see_line(self)
  self.fsm.vars.line_chosen = nil
  -- visible
  local lines_vis = {}
  for k,v in pairs( self.fsm.vars.lines ) do
    if v:visibility_history() >= MIN_VIS_HIST then
      local center = llutils.laser_lines_center({ x=v:end_point_1(0),
                                                  y=v:end_point_1(1)},
                                                { x=v:end_point_2(0),
                                                  y=v:end_point_2(1)}, v:bearing())

      local trans = tfm.transform({x=center.x, y=center.y, ori=center.ori}, v:frame_id(), "/base_link")
      local x   = trans.x
      local y   = trans.y
      local ori = trans.ori
      printf("Got line: (%f, %f, %f)", x, y, ori)
      printf("Errors are: (%f, %f, %f)", math.abs(x - self.fsm.vars.x), math.abs(y - self.fsm.vars.y), math.angle_distance(ori, self.fsm.vars.ori))

      if math.abs(x - self.fsm.vars.x)                <= AREA_LINE_ERR_X and
         math.abs(y - self.fsm.vars.y)                <= AREA_LINE_ERR_Y and
         math.angle_distance(ori, self.fsm.vars.ori)  <= AREA_LINE_ERR_ORI then
         printf("OK  line: (%f, %f, %f)", math.abs(x - self.fsm.vars.x), math.abs(y - self.fsm.vars.y), math.angle_distance(ori, self.fsm.vars.ori))
        table.insert( lines_vis, {x=x, y=y, ori=ori, bearing_err = math.abs(math.angle_distance(ori, self.fsm.vars.ori)), frame_id="/base_link"} )
      end
    end
  end
  if table.getn( lines_vis ) <= 0 then
    return false
  else
    -- get best line (by bearing? or what would be better)
    self.fsm.vars.line_best = lines_vis[1]
    for k,v in pairs( lines_vis ) do
      if self.fsm.vars.line_best.bearing_err > v.bearing_err then
        self.fsm.vars.line_best = v
      end
    end
    return true
  end
end

fsm:define_states{ export_to=_M, closure={see_line = see_line, LINE_TRIES=LINE_TRIES},
   {"INIT",                   JumpState},
   {"ALIGN_TAG",              SkillJumpState, skills={{align_tag}}, final_to="DECIDE_TRY", fail_to="DECIDE_TRY"}, -- TODO if we can't find it with the tag, we can at least try with the line (correct tag is later checked
   {"DECIDE_TRY",             JumpState},
   {"ALIGN",                  SkillJumpState, skills={{motor_move}}, final_to="CHECK_TAG", fail_to="FAILED"},  --TODO check if tag is right if given
   {"SEARCH_LINE",            JumpState}, --TODO check visibility_history
   {"CHECK_TAG",              SkillJumpState, skills={{check_tag}}, final_to="FINAL", fail_to="FINAL"},  --TODO go final even when failed because we have no solution right now
}

fsm:add_transitions{
   {"INIT",           "ALIGN_TAG",          cond=true },
   {"INIT",           "FAILED",             precond="not vars.x", desc="x argument missing"},
   {"DECIDE_TRY",     "SEARCH_LINE",        cond="vars.try <= LINE_TRIES", desc="try again to find a line" },
   {"DECIDE_TRY",     "FINAL",              cond=true, desc="tryed often enough, going final now :(" },
   {"SEARCH_LINE",    "DECIDE_TRY",         timeout=TIMEOUT,      desc="timeout"},
   {"SEARCH_LINE",    "ALIGN",              cond=see_line,        desc="see line"},
}

function INIT:init()
  self.fsm.vars.y   = self.fsm.vars.y   or 0
  self.fsm.vars.ori = self.fsm.vars.ori or 0
  self.fsm.vars.lines = {}
  ---[[
  self.fsm.vars.lines[1]  = line1
  self.fsm.vars.lines[2]  = line2
  self.fsm.vars.lines[3]  = line3
  self.fsm.vars.lines[4]  = line4
  self.fsm.vars.lines[5]  = line5
  self.fsm.vars.lines[6]  = line6
  self.fsm.vars.lines[7]  = line7
  self.fsm.vars.lines[8]  = line8
  --]]
  --[[
  self.fsm.vars.lines[1]  = line1avg
  self.fsm.vars.lines[2]  = line2avg
  self.fsm.vars.lines[3]  = line3avg
  self.fsm.vars.lines[4]  = line4avg
  self.fsm.vars.lines[5]  = line5avg
  self.fsm.vars.lines[6]  = line6avg
  self.fsm.vars.lines[7]  = line7avg
  self.fsm.vars.lines[8]  = line8avg
  --]]
  
  self.fsm.vars.try = 0
  self.fsm.vars.retry_to_recover_by_line_times = 0
end

function ALIGN_TAG:init()
   -- use defaults for optional args
   -- self.fsm.vars.y = self.fsm.vars.y or 0 -- just important for laser-lines? should we provice a different param for that? or automatic from navgraph?
   self.fsm.vars.ori = self.fsm.vars.ori or 0
   -- align by ALIGN_DISTANCE from tag to base_link with align_tag
   local tag_transformed = tfm.transform({x=self.fsm.vars.x, y=self.fsm.vars.y, ori=self.fsm.vars.ori}, "/base_link", "/cam_tag")
   -- give align_tag the id if we have one
   self.skills[1].tag_id = self.fsm.vars.tag_id
   self.skills[1].x = self.fsm.vars.x + TAG_X_ERR
--   self.skills[1].y = -self.fsm.vars.y -- see comment above
   self.skills[1].y = 0
   self.skills[1].ori = self.fsm.vars.ori
end

function DECIDE_TRY:init()
  self.fsm.vars.try = self.fsm.vars.try + 1
end

function ALIGN:init()
   local pp = llutils.point_in_front(self.fsm.vars.line_best, self.fsm.vars.x)
   self.skills[1].x         = pp.x
   self.skills[1].y         = pp.y
   self.skills[1].ori       = pp.ori
   --self.skills[1].tolerance = {x=0.05, y=0.03, ori=0.1} -- this does not exists
end

function CHECK_TAG:init()
  self.skills[1].tag_id = self.fsm.vars.tag_id
end
