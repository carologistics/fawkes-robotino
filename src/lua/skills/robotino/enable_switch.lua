-- toggle_switch.lua
module(..., skillenv.module_init)
documentation = [==[Toggle the switch interface passed as parameter.
@param iface: Interface to switch (one of laser, motor, lightFront, machine-signal, delivery)
@param enable: true/false, i.e. on/off
]==]


-- Crucial skill information
name               = "enable_switch"
fsm                = SkillHSM:new{name=name, start="SWITCH", debug=false}
depends_skills     = {}
depends_interfaces = {
   {v = "laserSwitch", type="SwitchInterface", id="/laser-cluster/ampel"},
   {v = "motorSwitch", type="SwitchInterface", id="Robotino Motor"},
   {v = "machineSignalSwitch", type="SwitchInterface", id="/machine-signal"},
   {v = "deliverySwitch", type="SwitchInterface", id="/machine-signal/delivery-mode"}
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
      laser          = laserSwitch,
      motor          = motorSwitch,
      lightFront     = machineSignalSwitch,
      machineSignal  = machineSignalSwitch,
      delivery       = deliverySwitch
   }
   iface = ifmap[self.fsm.vars.iface]
	 for k,v in pairs(ifmap) do
			printf("** %s: %s", k, v:uid())
	 end
	 printf("Called for %s (%s)", self.fsm.vars.iface, iface:uid())
   if self.fsm.vars.enable then
      msg = iface.EnableSwitchMessage:new()
   else
      msg = iface.DisableSwitchMessage:new()
   end
   iface:msgq_enqueue_copy(msg)
end
