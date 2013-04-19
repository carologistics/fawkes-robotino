-- toggle_switch.lua
module(..., skillenv.module_init)
documentation = [==[Toggle the switch interface passed as parameter.
parameters:

iface: Interface to toggle.

Switchable interfaces:
omnivision, ampel]==]


-- Crucial skill information
name               = "enable_switch"
fsm                = SkillHSM:new{name=name, start="SWITCH", debug=true}
depends_skills     = {}
depends_interfaces = {
   {v = "omnivisionSwitch", type="SwitchInterface", id="omnivisionSwitch"},
   {v = "ampelSwitch", type="SwitchInterface", id="ampelswitch"}
}

-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
   {"SWITCH", JumpState}
}

fsm:add_transitions{
   {"SWITCH", "FINAL", cond=true},
}

function SWITCH:init()
   ifmap = {
      omnivision = omnivisionSwitch,
      ampel = ampelSwitch
   }
   iface = ifmap[self.fsm.vars.iface]
   if self.fsm.vars.enable then
      msg = iface.EnableSwitchMessage:new()
   else
      msg = iface.DisableSwitchMessage:new()
   end
   iface:msgq_enqueue_copy(msg)
end
