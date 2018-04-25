

-- Initialize module
module(..., skillenv.module_init)

-- Crucial skill information
name               = "slide_put_new"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"motor_move", "gripper_commands_new"}
depends_interfaces = {}

documentation      = [==[

]==]

-- Initialize as skill module
skillenv.skill_module(_M)
local slide_distance=-0.275, --TODO measure exact value
local move_back_dist=-0.2

fsm:define_states{ export_to=_M,
   {"INIT", JumpState},
   {"GOTO_SLIDE", SkillJumpState, skills={{motor_move}}, final_to="GRIPPER_MOVE_Z", fail_to="FAILED"},
   {"GRIPPER_MOVE_Z", SkillJumpState, skills={{motor_move}}, final_to="OPEN_GRIPPER", fail_to="FAILED"},
   {"OPEN_GRIPPER", SkillJumpState, skills={{gripper_commands_new}}, final_to="WAIT_FOR_GRIPPER",fail_to="FAILED"},
   {"WAIT_FOR_GRIPPER", JumpState},
   {"MOVE_BACKWARDS", SkillJumpState, skills={{motor_move}}, final_to="CLOSE_GRIPPER", fail_to="FAILED"},
   {"CLOSE_GRIPPER", SkillJumpState, skills={{gripper_commands_new}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"INIT", "GOTO_SLIDE", cond=true},
   {"WAIT_FOR_GRIPPER", "MOVE_BACKWARDS", timeout=1},
}


function GOTO_SLIDE:init()
   self.args["motor_move"] ={ y = slide_distance,	vel_trans = 0.2,tolerance = { x=0.002, y=0.002, ori=0.01 }
			}
end

function GRIPPER_MOVE_Z:init()
   self.args["gripper_commands_new"].z = gripper_z
   self.args["gripper_commands_new"].command = "MOVEABS"
end

function OPEN_GRIPPER:init()
   self.args["gripper_commands_new"].command = "OPEN"
end

function MOVE_BACKWARDS:init()
   self.args["motor_move"].x = move_back_dist
end

function CLOSE_GRIPPER:init()
   self.args["gripper_commands_new"].command = "CLOSE"
end
