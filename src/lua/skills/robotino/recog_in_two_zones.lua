-- Initialize module
module(..., skillenv.module_init)

-- Crucial skill information
name               = "recog_in_two_zones"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"zone_recog"}
depends_interfaces = {}

documentation      = [==[ drive_zones_mps_recognize

                          This skill does:
                              Drives to the zones and recognizes the mps               

]==]


-- Initialize as skill module
skillenv.skill_module(_M)


fsm:define_states{ export_to=_M,
   closure = {is_not_finished=is_not_finished,},
   {"INIT", JumpState},
   {"FAILED_TO_RECOGNIZE_MPS_1",JumpState}, 
   {"FAILED_TO_RECOGNIZE_MPS_2",JumpState}, 
   {"RECOGNIZE_MPS_1", SkillJumpState, skills={{zone_recog}}, final_to="RECOGNIZE_MPS_2", fail_to="FAILED_TO_RECOGNIZE_MPS_1"},
   {"RECOGNIZE_MPS_2", SkillJumpState, skills={{zone_recog}}, final_to="FINAL", fail_to="FAILED_TO_RECOGNIZE_MPS_2"},
}

fsm:add_transitions{
   {"INIT", "RECOGNIZE_MPS_1", cond=true},
   {"FAILED_TO_RECOGNIZE_MPS_1", "RECOGNIZE_MPS_2", cond=true,desc="Failed to recognize MPS_1"},
   {"FAILED_TO_RECOGNIZE_MPS_2", "FAILED", cond=true, desc="Failed to recognize MPS_2"}, 
}


function INIT:init()
end

function RECOGNIZE_MPS_1:init()
	 self.args["zone_recog"].zone  = self.fsm.vars.zone1	
end


function RECOGNIZE_MPS_2:init() 
	self.args["zone_recog"].zone = self.fsm.vars.zone2

end
