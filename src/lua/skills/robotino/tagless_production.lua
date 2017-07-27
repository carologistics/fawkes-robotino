-- Initialize module
module(..., skillenv.module_init)

-- Crucial skill information
name               = "tagless_production"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"tagless_mps_align","goto","mps_recog_side","shelf_pick","product_pick","product_put","approach_test"}
depends_interfaces = {
}

documentation      = [==[
aligns to a machine and picks a product from the conveyor.
It will get the offsets and the align distance for the machine
from the navgraph

Parameters:

      @ zone

]==]
-- Initialize as skill module
skillenv.skill_module(_M)
-- Constants
MPS_TYPES = {
'No Station',
'cap-input',
'cap-output',
}


function already_at_conveyor(self)
   return (self.fsm.vars.atmps == "CONVEYOR")
end


function calc_xy_coordinates(self)

  -- zone argument is of the form  M-Z21
  self.fsm.vars.xZone = tonumber(string.sub(self.fsm.vars.zone, 4, 4)) - 0.5
  self.fsm.vars.yZone = tonumber(string.sub(self.fsm.vars.zone, 5, 5)) - 0.5
  if string.sub(self.fsm.vars.zone, 1, 1) == "M" then
   self.fsm.vars.xZone = 0 - self.fsm.vars.xZone
  end

  self.fsm.vars.xLeft = self.fsm.vars.xZone-1
  self.fsm.vars.xRight = self.fsm.vars.xZone+1

  self.fsm.vars.yUp = self.fsm.vars.yZone+1
  self.fsm.vars.yDown = self.fsm.vars.yZone-1

  self.fsm.vars.alignX1 = self.fsm.vars.xZone - 0.5
  self.fsm.vars.alignY1 = self.fsm.vars.yZone - 0.5
  self.fsm.vars.alignX2 = self.fsm.vars.xZone + 0.5
  self.fsm.vars.alignY2 = self.fsm.vars.yZone + 0.5


  self.fsm.vars.currShelf = "LEFT"
  self.fsm.vars.pickedShelf = false

  return true
end

function shelf_decide(self)

  if self.fsm.vars.pickedShelf == false then

    printf("picked shelf false")
    if (self.fsm.vars.currShelf == "LEFT" ) then
        printf("curr shelf was LEFT")

        self.fsm.vars.currShelf = "MIDDLE"

    elseif (self.fsm.vars.currShelf == "MIDDLE") then

        self.fsm.vars.currShelf = "RIGHT"

     end
        return false
  else
       return true

  end
end

function shelf_suc(self)
    self.fsm.vars.pickedShelf = true
    return true
end

function side_check_evaluation(self)


    --recognition_result = MPS_TYPES[mps_recognition_if:mpstype()+1];
    --  if(recognition_result == 'cap-input') then
    --      return true
    --  end
    return false
end

fsm:define_states{ export_to=_M, closure={navgraph=navgraph},
   {"INIT", JumpState},
   {"DRIVE_TO_1", SkillJumpState, skills={{goto}}, final_to="MPS_ALIGN_1", fail_to="DRIVE_TO_2"},
   {"DRIVE_TO_2", SkillJumpState, skills={{goto}}, final_to="MPS_ALIGN_2", fail_to="DRIVE_TO_3"},
   {"DRIVE_TO_3", SkillJumpState, skills={{goto}}, final_to="MPS_ALIGN_3", fail_to="DRIVE_TO_4"},
   {"DRIVE_TO_4", SkillJumpState, skills={{goto}}, final_to="MPS_ALIGN_4", fail_to="FAILED"},
   {"MPS_ALIGN_1", SkillJumpState, skills={{tagless_mps_align}}, final_to="CHECK_SIDE", fail_to="DRIVE_TO_2"},
   {"MPS_ALIGN_2", SkillJumpState, skills={{tagless_mps_align}}, final_to="CHECK_SIDE", fail_to="DRIVE_TO_3"},
   {"MPS_ALIGN_3", SkillJumpState, skills={{tagless_mps_align}}, final_to="CHECK_SIDE", fail_to="DRIVE_TO_4"},
   {"MPS_ALIGN_4", SkillJumpState, skills={{tagless_mps_align}}, final_to="CHECK_SIDE", fail_to="FAILED"},
   {"CHECK_SIDE_EVALUATE",JumpState},
   {"CHECK_SIDE", SkillJumpState, skills={{mps_recog_side}}, final_to="CHECK_SIDE_EVALUATE", fail_to="CHECK_SIDE_EVALUATE"},
   {"DRIVE_TO_CORRECT_SIDE", SkillJumpState, skills={{goto}}, final_to="SKILL_TAGLESS_SHELF_PICK",fail_to="FAILED"},
   {"DECIDE_SHELF_PICK", JumpState},
   {"SHELF_PICK_SUC",JumpState},
   {"SKILL_TAGLESS_SHELF_PICK", SkillJumpState, skills={{approach_test}}, final_to="SHELF_PICK_SUC", fail_to="DECIDE_SHELF_PICK"},
   {"SKILL_TAGLESS_PRODUCT_PUT",SkillJumpState, skills={{approach_test}}, final_to="DRIVE_TO_OTHER_SIDE", fail_to="FAILED"},
   {"SKILL_TAGLESS_PRODUCT_PICK",SkillJumpState,skills={{approach_test}}, final_to="FINAL", fail_to="FAILED"},
   --{"SKILL_TAGLESS_PRODUCT_PICK", SkillJumpState, skills={{product_pick}}, final_to="FINAL", fail_to="FAILED"},
   {"DRIVE_TO_OTHER_SIDE",SkillJumpState,skills={{goto}}, final_to="MPS_ALIGN_OUTPUT", fail_to="FAILED"},
   {"MPS_ALIGN_OUTPUT", SkillJumpState, skills={{tagless_mps_align}}, final_to="SKILL_TAGLESS_PRODUCT_PICK", fail_to="FAILED"},
   {"MPS_ALIGN_PRODUCT_PUT", SkillJumpState, skills={{tagless_mps_align}}, final_to="SKILL_TAGLESS_PRODUCT_PUT",fail_to="FAILED"},
   {"SKILL_REALIGN_INPUT", SkillJumpState, skills={{tagless_mps_align}}, final_to="SKILL_TAGLESS_SHELF_PICK", fail_to="FAILED"},
}

fsm:add_transitions{
   {"INIT", "DRIVE_TO_1", cond=calc_xy_coordinates},
   {"INIT", "FAILED", cond=true},
--   {"SKILL_REALIGN_INPUT","SKILL_TAGLESS_PRODUCT_PICK", cond="self.fsm.vars.pickedShelf == true" },
--   {"SKILL_REALIGN_INPUT", "SKILL_TAGLESS_SHELF_PICK", cond=true},
   {"DECIDE_SHELF_PICK", "MPS_ALIGN_PRODUCT_PUT", cond=shelf_decide}, --shelf_decide
   {"DECIDE_SHELF_PICK", "SKILL_REALIGN_INPUT", cond=true},
   {"SHELF_PICK_SUC","DECIDE_SHELF_PICK", cond=shelf_suc},
   {"CHECK_SIDE_EVALUATE","SKILL_TAGLESS_SHELF_PICK", cond=side_check_evaluation},
   {"CHECK_SIDE_EVALUATE", "DRIVE_TO_CORRECT_SIDE",cond=true},
}


function DRIVE_TO_1:init()

   self.args["goto"].x = self.fsm.vars.xLeft
   self.args["goto"].y = self.fsm.vars.yZone
   self.args["goto"].ori = 0

   self.fsm.vars.lastX = self.fsm.vars.xLeft
   self.fsm.vars.lastY = self.fsm.vars.yZone
   self.fsm.vars.lasatOri = 0
end

function DRIVE_TO_2:init()
   self.args["goto"].x = self.fsm.vars.xZone
   self.args["goto"].y = self.fsm.vars.yDown
   self.args["goto"].ori = 1.57

   self.fsm.vars.lastX = self.fsm.vars.xZone
   self.fsm.vars.lastY = self.fsm.vars.yDown
   self.fsm.vars.lasatOri = 1.57
end


function DRIVE_TO_3:init()
   self.args["goto"].x = self.fsm.vars.xRight
   self.args["goto"].y = self.fsm.vars.yZone
   self.args["goto"].ori = 3.1415

   self.fsm.vars.lastX = self.fsm.vars.xRight
   self.fsm.vars.lastY = self.fsm.vars.yZone
   self.fsm.vars.lasatOri = 3.1415
end

function DRIVE_TO_4:init()
   self.args["goto"].x = self.fsm.vars.xZone
   self.args["goto"].y = self.fsm.vars.yUp
   self.args["goto"].ori = 4.71

   self.fsm.vars.lastX = self.fsm.vars.xZone
   self.fsm.vars.lastY = self.fsm.vars.yUp
   self.fsm.vars.lasatOri = 4.71
end


function MPS_ALIGN_1:init()
 self.args["tagless_mps_align"].x1 = self.fsm.vars.alignX1
  self.args["tagless_mps_align"].y1 = self.fsm.vars.alignY1
  self.args["tagless_mps_align"].x2 = self.fsm.vars.alignX2
  self.args["tagless_mps_align"].y2 = self.fsm.vars.alignY2
end

function MPS_ALIGN_2:init()

  self.args["tagless_mps_align"].x1 = self.fsm.vars.alignX1
   self.args["tagless_mps_align"].y1 = self.fsm.vars.alignY1
   self.args["tagless_mps_align"].x2 = self.fsm.vars.alignX2
   self.args["tagless_mps_align"].y2 = self.fsm.vars.alignY2


end
function MPS_ALIGN_3:init()
 self.args["tagless_mps_align"].x1 = self.fsm.vars.alignX1
  self.args["tagless_mps_align"].y1 = self.fsm.vars.alignY1
  self.args["tagless_mps_align"].x2 = self.fsm.vars.alignX2
  self.args["tagless_mps_align"].y2 = self.fsm.vars.alignY2

end
function MPS_ALIGN_4:init()
 self.args["tagless_mps_align"].x1 = self.fsm.vars.alignX1
  self.args["tagless_mps_align"].y1 = self.fsm.vars.alignY1
  self.args["tagless_mps_align"].x2 = self.fsm.vars.alignX2
  self.args["tagless_mps_align"].y2 = self.fsm.vars.alignY2

end


function MPS_ALIGN_OUTPUT:init()
  self.args["tagless_mps_align"].x1 = self.fsm.vars.alignX1
   self.args["tagless_mps_align"].y1 = self.fsm.vars.alignY1
   self.args["tagless_mps_align"].x2 = self.fsm.vars.alignX2
   self.args["tagless_mps_align"].y2 = self.fsm.vars.alignY2

end

function MPS_ALIGN_PRODUCT_PUT:init()
  self.args["tagless_mps_align"].x1 = self.fsm.vars.alignX1
   self.args["tagless_mps_align"].y1 = self.fsm.vars.alignY1
   self.args["tagless_mps_align"].x2 = self.fsm.vars.alignX2
   self.args["tagless_mps_align"].y2 = self.fsm.vars.alignY2
end

function SKILL_REALIGN_INPUT:init()
  self.args["tagless_mps_align"].x1 = self.fsm.vars.alignX1
   self.args["tagless_mps_align"].y1 = self.fsm.vars.alignY1
   self.args["tagless_mps_align"].x2 = self.fsm.vars.alignX2
   self.args["tagless_mps_align"].y2 = self.fsm.vars.alignY2
end


function DRIVE_TO_CORRECT_SIDE:init()


    if (self.fsm.vars.lastX > self.fsm.vars.xZone) then
      self.args["goto"].x = self.fsm.vars.lastX - 2
      self.fsm.vars.lastX = self.fsm.vars.lastX - 2
    end

    if (self.fsm.vars.lastX < self.fsm.vars.xZone) then
        self.args["goto"].x = self.fsm.vars.lastX + 2
        self.fsm.vars.lastX = self.fsm.vars.lastX + 2
      end


    if (self.fsm.vars.lastX == self.fsm.vars.xZone) then
       self.args["goto"].x = self.fsm.vars.lastX

    end

    if (self.fsm.vars.lastY > self.fsm.vars.yZone) then
           self.args["goto"].y = self.fsm.vars.lastY -2
           self.fsm.vars.lastY = self.fsm.vars.lastY -2
    end

    if (self.fsm.vars.lastY < self.fsm.vars.yZone) then
             self.args["goto"].y = self.fsm.vars.lastY + 2
             self.fsm.vars.lastY = self.fsm.vars.lastY + 2
    end

    if (self.fsm.vars.lastY == self.fsm.vars.yZone) then
             self.args["goto"].y = self.fsm.vars.lastY

    end

    self.args["goto"].ori = self.fsm.vars.lasatOri + 3.1415
    self.fsm.vars.lasatOri = self.fsm.vars.lasatOri + 3.1415

end

function DRIVE_TO_OTHER_SIDE:init()

      if (self.fsm.vars.lastX > self.fsm.vars.xZone) then
         self.args["goto"].x = self.fsm.vars.lastX - 2
       end

      if (self.fsm.vars.lastX < self.fsm.vars.xZone) then
           self.args["goto"].x = self.fsm.vars.lastX + 2
      end

      if (self.fsm.vars.lastX == self.fsm.vars.xZone) then
          self.args["goto"].x = self.fsm.vars.lastX
      end

      if (self.fsm.vars.lastY > self.fsm.vars.yZone) then
               self.args["goto"].y = self.fsm.vars.lastY -2
      end

      if (self.fsm.vars.lastY < self.fsm.vars.yZone) then
                 self.args["goto"].y = self.fsm.vars.lastY + 2
      end

      if (self.fsm.vars.lastY == self.fsm.vars.yZone) then
                self.args["goto"].y = self.fsm.vars.lastY
      end


      self.args["goto"].ori = self.fsm.vars.lasatOri + 3.1415

end

function SKILL_TAGLESS_SHELF_PICK:init()

    printf("pick und %s",self.fsm.vars.currShelf)
    self.args["approach_test"].option = "pick"
    self.args["approach_test"].shelf = self.fsm.vars.currShelf

end



function SKILL_TAGLESS_PRODUCT_PICK:init()

  self.args["approach_test"].option = "pick"

end

function SKILL_TAGLESS_PRODUCT_PUT:init()

  self.args["approach_test"].option = "put"
end
