require("fawkes.modinit")

module(..., fawkes.modinit.module_init)


-- Constants
local margin = 0.605
local fmrgin = 0.3
field_size = 0.6

--0.605 sind sicherheitsabstand etc zusammengerechnet
delivery_goto = {
    M1  = {x = 1.68,        y = 1.68-margin, ori =  math.pi/2, d_skill="move_under_rfid"},
    M2  = {x = 3.9,         y = 1.68-margin, ori =  math.pi/2, d_skill="move_under_rfid"},
    M3  = {x = 0.56,        y = 2.80-margin, ori =  math.pi/2, d_skill="move_under_rfid"},
    M4  = {x = 1.68,        y = 2.80-margin, ori =  math.pi/2, d_skill="move_under_rfid"},
    M5  = {x = 2.80-margin, y = 2.80,        ori =  0, d_skill="move_under_rfid"},
    M6  = {x = 3.92,        y = 2.80+margin, ori =  -math.pi/2, d_skill="move_under_rfid"},
    M7  = {x = 5.04-margin, y = 2.80,        ori =  0, d_skill="move_under_rfid"},
    M8  = {x = 1.68,        y = 3.92-margin, ori =  math.pi/2, d_skill="move_under_rfid"},
    M9  = {x = 3.92,        y = 3.92+margin, ori = -math.pi/2, d_skill="move_under_rfid"},
    M10 = {x = 5.04-margin, y = 3.92,        ori = 0, d_skill="move_under_rfid"},
    M11 = {d_skill="move_under_rfid"},
    M12 = {d_skill="move_under_rfid"},
    M13 = {d_skill="move_under_rfid"},
    M14 = {d_skill="move_under_rfid"},
    M15 = {d_skill="move_under_rfid"},
    M16 = {d_skill="move_under_rfid"},
    M17 = {d_skill="move_under_rfid"},
    M18 = {d_skill="move_under_rfid"},
    M19 = {d_skill="move_under_rfid"},
    M20 = {d_skill="move_under_rfid"},
    M21 = {d_skill="move_under_rfid"},
    M22 = {d_skill="move_under_rfid"},
    M23 = {d_skill="move_under_rfid"},
    M24 = {d_skill="move_under_rfid"},
    R1  = {x = 0.20+margin, y = 0.20+margin, ori = -math.pi*(3/4), d_skill="move_under_rfid"},
    R2  = {x = 5.40-margin, y = 5.40-margin, ori = math.pi/4 , d_skill="move_under_rfid"},
    T   = {x = 5.40+margin, y = 0.20+margin, ori = -math.pi/4, d_skill="move_under_rfid"},
    EGI = {x = 3.60+margin, y = 5.35,        ori = math.pi},
    D1 =  {x = 3.15,        y = 0.26+margin, ori = -math.pi/2, d_skill="deliver_puck"},
    D2 =  {x = 2.80,        y = 0.26+margin, ori = -math.pi/2, d_skill="deliver_puck"},
    D3 =  {x = 2.45,        y = 0.26+margin, ori = -math.pi/2, d_skill="deliver_puck"},
    deliver1 =  {d_skill="deliver_puck"},
    deliver2 =  {d_skill="deliver_puck"},
    Is =  {x = 2.25,	    y = 4.9,	     ori = math.pi/2}, 
}

delivery_goto.m1 = delivery_goto.M1
delivery_goto.m2 = delivery_goto.M2
delivery_goto.m3 = delivery_goto.M3
delivery_goto.m4 = delivery_goto.M4
delivery_goto.m5 = delivery_goto.M5
delivery_goto.m6 = delivery_goto.M6
delivery_goto.m7 = delivery_goto.M7
delivery_goto.m8 = delivery_goto.M8
delivery_goto.m9 = delivery_goto.M9
delivery_goto.m10 = delivery_goto.M10

delivery_goto.TEST = delivery_goto.T
delivery_goto.Test = delivery_goto.T

delivery_goto.ExpressGoodInsertion = delivery_goto.EGI

delivery_goto.Delivery1 = delivery_goto.D1
delivery_goto.Delivery2 = delivery_goto.D2
delivery_goto.Delivery3 = delivery_goto.D3

fields = {
    M1  = {x = 1.68-fmrgin, y = 1.68-fmrgin}, 
    M2  = {x = 3.9-fmrgin,  y = 1.68-fmrgin}, 
    M3  = {x = 0.56-fmrgin, y = 2.80-fmrgin},
    M4  = {x = 1.68-fmrgin, y = 2.80-fmrgin},
    M5  = {x = 2.80-fmrgin, y = 2.80-fmrgin},
    M6  = {x = 3.92-fmrgin, y = 2.80-fmrgin},
    M7  = {x = 5.04-fmrgin, y = 2.80-fmrgin},
    M8  = {x = 1.68-fmrgin, y = 3.92-fmrgin},
    M9  = {x = 3.92-fmrgin, y = 3.92-fmrgin},
    M10 = {x = 5.04-fmrgin, y = 3.92-fmrgin},
    R1  = {x = 0.20-fmrgin, y = 0.20-fmrgin},
    R2  = {x = 5.40-fmrgin, y = 5.40-fmrgin},
    T   = {x = 5.40-fmrgin, y = 0.20-fmrgin},
    EGI = {x = 3.60-fmrgin, y = 5.35-fmrgin},
    D1  = {x = 3.15-fmrgin, y = 0.26-fmrgin},
    D2  = {x = 2.80-fmrgin, y = 0.26-fmrgin},
    D3  = {x = 2.45-fmrgin, y = 0.26-fmrgin},
    
}

fields.m1 = fields.M1
fields.m2 = fields.M2
fields.m3 = fields.M3
fields.m4 = fields.M4
fields.m5 = fields.M5
fields.m6 = fields.M6
fields.m7 = fields.M7
fields.m8 = fields.M8
fields.m9 = fields.M9
fields.m10 = fields.M10

fields.TEST = fields.T
fields.Test = fields.T

fields.ExpressGoodInsertion = fields.EGI

fields.Delivery1 = fields.D1
fields.Delivery2 = fields.D2
fields.Delivery3 = fields.D3

