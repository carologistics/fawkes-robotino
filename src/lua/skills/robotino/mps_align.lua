
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
depends_skills     = { "align_tag", "motor_move" }
depends_interfaces = {
  ---[[
  {v = "line1avg", type="LaserLineInterface", id="/laser-lines/1/moving_avg"},
  {v = "line2avg", type="LaserLineInterface", id="/laser-lines/2/moving_avg"},
  {v = "line3avg", type="LaserLineInterface", id="/laser-lines/3/moving_avg"},
  {v = "line4avg", type="LaserLineInterface", id="/laser-lines/4/moving_avg"},
  {v = "line5avg", type="LaserLineInterface", id="/laser-lines/5/moving_avg"},
  {v = "line6avg", type="LaserLineInterface", id="/laser-lines/6/moving_avg"},
  {v = "line7avg", type="LaserLineInterface", id="/laser-lines/7/moving_avg"},
  {v = "line8avg", type="LaserLineInterface", id="/laser-lines/8/moving_avg"},
  --]]
  --[[
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
                          @param y          float   optional the y offset after the alignmend
                          @param ori        float   optional the ori offset after the alignmend
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

local tfm = require("tf_module")
local llutils = require("fawkes.laser-lines_utils")

-- Tunables
local TAG_X_ERR=0.02
local MIN_VIS_HIST=20
local TIMEOUT=4
local AREA_LINE_ERR_X=0.05
local AREA_LINE_ERR_Y=0.1
local AREA_LINE_ERR_ORI=0.17
local LINE_TRYS=3
local LINE_FOR_TAG_TRYS=2

function see_line_for_tag(self)
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
      
      table.insert( lines_vis, {x=x, y=y, ori=ori, bearing_err = math.abs(math.angle_distance(ori, self.fsm.vars.ori)), frame_id="/base_link"} )
    end
  end
  if table.getn( lines_vis ) <= 0 then
    return false
  else
    self.fsm.vars.line_best = lines_vis[1]
    for k,v in pairs( lines_vis ) do
      if self.fsm.vars.line_best.bearing_err > v.bearing_err then
        self.fsm.vars.line_best = v
      end
    end
    return true
  end
end

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

fsm:define_states{ export_to=_M, closure={see_line = see_line, LINE_TRYS=LINE_TRYS, LINE_FOR_TAG_TRYS=LINE_FOR_TAG_TRYS},
   {"INIT",                   JumpState},
   {"DECIDE_RETRY",           JumpState},
   {"SKILL_ALIGN_TAG",        SkillJumpState, skills={{align_tag}},  final_to="SEARCH_LINE", fail_to="SEARCH_LINE_FOR_TAG"},
   {"SEARCH_LINE_FOR_TAG",    JumpState},
   {"ALIGN_TO_SEE_TAG",       SkillJumpState, skills={{motor_move}}, final_to="SKILL_ALIGN_TAG", fail_to="SKILL_ALIGN_TAG"},  -- failed to skill becase just recover (can't be worst)
   {"SEARCH_LINE",            JumpState},
   {"LINE_SETTLE",            JumpState},
   {"ALIGN_WITH_LASERLINES",  SkillJumpState, skills={{motor_move}}, final_to="FINAL",    fail_to="FAILED"},
}

fsm:add_transitions{
   {"INIT",             "SKILL_ALIGN_TAG",        cond=true },
   {"DECIDE_RETRY",     "SKILL_ALIGN_TAG",        cond="vars.try <= LINE_TRYS", desc="try again to find a line" },
   {"DECIDE_RETRY",     "FINAL",                  cond=true,                    desc="tryed often enough, going final now :(" },
   {"SKILL_ALIGN_TAG",  "FAILED",                 precond="not vars.x", desc="x argument missing"},
   {"SEARCH_LINE_FOR_TAG", "FAILED",              cond="vars.retry_to_recover_by_line_times >= LINE_FOR_TAG_TRYS", desc="retry to see tag too often => fail" },
   {"SEARCH_LINE_FOR_TAG", "ALIGN_TO_SEE_TAG",    cond=see_line_for_tag,desc="see line, try to align to see tag afterwards" },
   {"SEARCH_LINE_FOR_TAG", "FAILED",              timeout=TIMEOUT,      desc="can't align by tag and can't recover with line => fail" },
   {"SEARCH_LINE",      "LINE_SETTLE",            cond=see_line,        desc="see line"},
   {"SEARCH_LINE",      "DECIDE_RETRY",           timeout=TIMEOUT,      desc="timeout"},
   {"LINE_SETTLE",      "ALIGN_WITH_LASERLINES",  timeout=1.5,          desc="wait 0.5s"}
}

function INIT:init()
  self.fsm.vars.y   = self.fsm.vars.y   or 0
  self.fsm.vars.ori = self.fsm.vars.ori or 0
  self.fsm.vars.lines = {}
  --[[
  self.fsm.vars.lines[1]  = line1
  self.fsm.vars.lines[2]  = line2
  self.fsm.vars.lines[3]  = line3
  self.fsm.vars.lines[4]  = line4
  self.fsm.vars.lines[5]  = line5
  self.fsm.vars.lines[6]  = line6
  self.fsm.vars.lines[7]  = line7
  self.fsm.vars.lines[8]  = line8
  --]]
  ---[[
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

function DECIDE_RETRY:init()
  self.fsm.vars.try = self.fsm.vars.try + 1
end

function SKILL_ALIGN_TAG:init()
   -- use defaults for optional args
   self.fsm.vars.y = self.fsm.vars.y or 0
   self.fsm.vars.ori = self.fsm.vars.ori or 0
   -- align by ALIGN_DISTANCE from tag to base_link with align_tag
   local tag_transformed = tfm.transform({x=self.fsm.vars.x, y=self.fsm.vars.y, ori=self.fsm.vars.ori}, "/base_link", "/cam_tag")
   -- give align_tag the id if we have one
   self.skills[1].tag_id = self.fsm.vars.tag_id
   self.skills[1].x = self.fsm.vars.x + TAG_X_ERR
   self.skills[1].y = -self.fsm.vars.y
   self.skills[1].ori = self.fsm.vars.ori
end

function SEARCH_LINE_FOR_TAG:init()
  self.fsm.vars.retry_to_recover_by_line_times = self.fsm.vars.retry_to_recover_by_line_times + 1
end

function ALIGN_TO_SEE_TAG:init()
   local pp = llutils.point_in_front(self.fsm.vars.line_best, 0.5)
   self.skills[1].x         = pp.x
   self.skills[1].y         = pp.y
   self.skills[1].ori       = pp.ori
   self.skills[1].tolerance = {x=0.01, y=0.01, ori=0.02}
end

function ALIGN_WITH_LASERLINES:init()
   -- align by ALIGN_DISTANCE from tag to base_link with the lase_line
   printf("line choosen: (%f, %f, %f)", self.fsm.vars.line_best.x, self.fsm.vars.line_best.y, self.fsm.vars.line_best.ori)
   local pp = llutils.point_in_front(self.fsm.vars.line_best, self.fsm.vars.x)
   self.skills[1].x         = pp.x
   self.skills[1].y         = 0 -- can't improve y coordinate with laserlines so leave it
   self.skills[1].ori       = pp.ori --self.fsm.vars.line_best.ori + self.fsm.vars.ori
   self.skills[1].tolerance = {x=0.01, y=0.01, ori=0.02}
end
