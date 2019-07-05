-- recognize_mps.lua

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

module(..., skillenv.module_init)
documentation = [==[Tries to recognize a MPS
]==]

-- Crucial skill information
name               = "recognize_mps"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"markerless_mps_align"}
depends_interfaces = {
   {v = "if_picture_taker", type = "PictureTakerInterface", id="PictureTaker"},
}

-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
   {"INIT", JumpState},
   {"MPS_ALIGN", SkillJumpState, skills={{markerless_mps_align}}, final_to="SHORT_WAIT", fail_to="FAILED"},
   {"TAKE_NEW_PICTURE", JumpState},
   {"RECOGNIZE_PICTURE", JumpState},
   {"SHORT_WAIT", JumpState},
   {"TAKE_PICTURE", JumpState},
   {"DECIDE", JumpState},
}

fsm:add_transitions{
   {"INIT", "MPS_ALIGN", cond=true},
   {"SHORT_WAIT", "TAKE_NEW_PICTURE", timeout=0.3},
   {"TAKE_NEW_PICTURE", "RECOGNIZE_PICTURE", timeout=1},
   {"RECOGNIZE_PICTURE", "MPS_ALIGN", cond="vars.index <= 3"},
   {"RECOGNIZE_PICTURE", "DECIDE", cond=true},
   {"DECIDE", "FINAL", timeout=1}
}

function INIT:init()
  self.fsm.vars.index=1
  self.fsm.vars.pos_y = {-0.35,0.35,0.0}
  self.fsm.vars.pos_x = {0.6, 0.6, 0.6}
  self.fsm.vars.ori = {-math.atan2(self.fsm.vars.pos_y[1],self.fsm.vars.pos_x[1]),
		-	math.atan2(self.fsm.vars.pos_y[2],self.fsm.vars.pos_x[2]),
		-	math.atan2(self.fsm.vars.pos_y[3],self.fsm.vars.pos_x[3])
		      }
  self.fsm.vars.possible_station = {"bs", "cs", "rs", "ss"}
  self.fsm.vars.results = {}
  for _,label in pairs(self.fsm.vars.possible_station) do
    self.fsm.vars.results[label] = 0.0
    print(label)
  end
end

function TAKE_NEW_PICTURE:init()
  if if_picture_taker:has_writer() then
    local msg = if_picture_taker.TakePictureMessage:new("blub", "blib")
    if_picture_taker:msgq_enqueue_copy(msg)
  end
end

function RECOGNIZE_PICTURE:init()
  os.execute("convert -rotate \"180\" /tmp/new_image.jpg /tmp/new_image.jpg")
  os.execute("python $HOME/label_image.py --graph=$HOME/recognition_model/output_graph.pb --labels=$HOME/recognition_model/output_labels.txt --input_layer=Placeholder --output_layer=final_result --image=/tmp/new_image.jpg > bla.txt")
  lines1 = io.lines("bla.txt")
  for line in lines1 do
    label, confidence = string.match(line, "(%a+) (%d+\.?%d+)")
    for _,possible_label in pairs(self.fsm.vars.possible_station) do
     if label == possible_label then
       self.fsm.vars.results[label] = self.fsm.vars.results[label] + confidence
       break
     end
    end
  end

  os.execute("python $HOME/label_image.py --graph=$HOME/recognition_model/output_graph2.pb --labels=$HOME/recognition_model/output_labels2.txt --input_layer=Mul --output_layer=final_result --image=/tmp/new_image.jpg > bla.txt")
  lines2 = io.lines("bla.txt")
  local result
  for line in lines2 do
    label, confidence = string.match(line, "(%a+) (%d+\.?%d+)")
    for _,possible_label in pairs(self.fsm.vars.possible_station) do
     if label == possible_label then
       self.fsm.vars.results[label] = self.fsm.vars.results[label] + confidence
       break
     end
    end
  end

  self.fsm.vars.index=self.fsm.vars.index+1	
end

function MPS_ALIGN:init()
  self.args["markerless_mps_align"].x = self.fsm.vars.pos_x[self.fsm.vars.index] 
  self.args["markerless_mps_align"].y = self.fsm.vars.pos_y[self.fsm.vars.index]
  self.args["markerless_mps_align"].ori = self.fsm.vars.ori[self.fsm.vars.index]
end

function DECIDE:init()
  local max_confidence = 0.0
  local max_label = ""
  for label, confidence in pairs(self.fsm.vars.results) do
    printf("%s %f", label, confidence)
    if confidence > max_confidence then
      max_confidence = confidence
      max_label = label
    end
  end
  printf("This is a %s", max_label)
end
