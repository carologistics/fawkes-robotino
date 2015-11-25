
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
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
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
local MIN_VIS_HIST=10
local TIMEOUT=5
local AREA_LINE_ERR_X=0.1
local AREA_LINE_ERR_Y=0.2
local AREA_LINE_ERR_ORI=0.2 --0.17
local LINE_TRIES=3
local POSES_ORI_TAG={0.3, 0.3, -0.9, -0.3}

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

function pose_ok(self)
  local pp = llutils.point_in_front(self.fsm.vars.line_best, self.fsm.vars.x)
  if  pp.x - self.fsm.vars.x <= AREA_LINE_ERR_X and
      pp.y - self.fsm.vars.y <= AREA_LINE_ERR_Y and
      pp.ori - self.fsm.vars.ori <= AREA_LINE_ERR_ORI then
    return true
  end
  return false
end

fsm:define_states{ export_to=_M, closure={see_line = see_line, LINE_TRIES=LINE_TRIES, pose_ok=pose_ok,},
   {"INIT",                   JumpState},
   {"TURN_TO_SEARCH_FOR_TAG", SkillJumpState, skills={{motor_move}}, final_to="CHECK_TAG", fail_to="CHECK_TAG"}, -- if this failes, we still should keep looking, the exit is in GET_NEXT_POSE_FOR_TAG
   {"CHECK_TAG",              SkillJumpState, skills={{check_tag}}, final_to="ALIGN_TAG", fail_to="GET_NEXT_POSE_FOR_TAG"},
   {"GET_NEXT_POSE_FOR_TAG",  JumpState},
   {"ALIGN_TAG",              SkillJumpState, skills={{align_tag}}, final_to="DECIDE_TRY", fail_to="FAILED"}, -- if this failes, the subskill or the tag vision needs to be fixed (we checked if we see the tag before)
   {"DECIDE_TRY",             JumpState},
   {"SETTLE_LINE",            JumpState},
   {"ALIGN",                  SkillJumpState, skills={{motor_move}}, final_to="CHECK_POSE", fail_to="FAILED"},
   {"CHECK_POSE",             JumpState},
}

fsm:add_transitions{
   {"INIT",                   "FAILED",                 precond="not vars.x", desc="x argument missing"},
   {"INIT",                   "CHECK_TAG",              cond=true },
   {"GET_NEXT_POSE_FOR_TAG",  "TURN_TO_SEARCH_FOR_TAG", cond="table.getn(vars.poses_ori_tag) >= 1" }, -- if there is one in the list, the init function will set the value and remove it
   {"GET_NEXT_POSE_FOR_TAG",  "FAILED",                 cond=true, desc="Can't find tag by turning"},
   {"DECIDE_TRY",             "SETTLE_LINE",            cond="vars.try_line <= LINE_TRIES", desc="try again to find a line" },
   {"DECIDE_TRY",             "FAILED",                 cond=true, desc="tryed often enough, we are failed now :(" },
   {"SETTLE_LINE",            "ALIGN",                  cond=see_line },
   {"SETTLE_LINE",            "FAILED",                 timeout=TIMEOUT },
   {"CHECK_POSE",             "FINAL",                  cond=pose_ok,  desc="pose is ok"},
   {"CHECK_POSE",             "DECIDE_TRY",             cond=true,        desc="pose is not ok, retry"},
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
  
  self.fsm.vars.try_line = 0
  self.fsm.vars.poses_ori_tag = {}
  for k,v in pairs(POSES_ORI_TAG) do
    self.fsm.vars.poses_ori_tag[k] = v
  end
end

function TURN_TO_SEARCH_FOR_TAG:init()
  self.args["motor_move"] = { x=nil, y=nil, ori=self.fsm.vars.next_pose_ori}
end

function CHECK_TAG:init()
  self.args["check_tag"] = { tag_id=self.fsm.vars.tag_id }
end

function GET_NEXT_POSE_FOR_TAG:init()
  self.fsm.vars.next_pose_ori = table.remove(self.fsm.vars.poses_ori_tag, 1)
end

function ALIGN_TAG:init()
  -- align by ALIGN_DISTANCE from tag to base_link with align_tag
  local tag_transformed = tfm.transform({x=self.fsm.vars.x, y=self.fsm.vars.y, ori=self.fsm.vars.ori}, "/base_link", "/cam_tag")
  -- give align_tag the id if we have one
  self.args["align_tag"] =
		 { tag_id = self.fsm.vars.tag_id,
			 x = self.fsm.vars.x,
			 --  y = -self.fsm.vars.y -- this should be the y minus the error tag error to the middle from the navgraph
			 y = 0,
			 ori = self.fsm.vars.ori
		 }
end

function DECIDE_TRY:init()
  self.fsm.vars.try_line = self.fsm.vars.try_line + 1
end

function ALIGN:init()
  local pp = llutils.point_in_front(self.fsm.vars.line_best, self.fsm.vars.x)
  self.args["motor_move"] = {x = pp.x, y = pp.y, ori = pp.ori}
  --self.args["motor_move"].tolerance = {x=0.05, y=0.03, ori=0.1} -- this does not exists
end
