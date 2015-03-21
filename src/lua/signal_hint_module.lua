require("fawkes.modinit")
module(..., fawkes.modinit.module_init)

-- Table height above the laser scanner plane
local TABLE_HEIGHT = 1.2

-- Signal positions relative to the top-left-front edge of the MPS table
local SIGNAL_POS = {
   RS = { x=30, y=20, z=10 }
}

depends_interfaces.insert(
   { v="bb_signal_hint", type="SignalHintInterface", id="/machine-signal/position-hint" }
)

function send_hint(machine)
   local msg = bb_signal_hint.SignalPositionMessage:new()
   msg:set_translation(0, SIGNAL_POS[machine].x)
   msg:set_translation(1, SIGNAL_POS[machine].y)
   msg:set_translation(2, SIGNAL_POS[machine].z + TABLE_HEIGHT)
   bb_signal_hint.msgq_enqueue_copy(msg)
end
