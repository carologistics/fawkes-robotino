-- public
local machine_pos = {}

-- Constants
local margin = 0.605
local field_margin = 0.6

--0.605 sind sicherheitsabstand etc zusammengerechnet
local machine_pos.delivery_goto = {
    M1  = {x = 1.68-margin, y = 1.68,        ori = 0}, 
    M2  = {x = 3.9,         y = 1.68+margin, ori = -math.pi/2}, 
    M3  = {x = 0.56,        y = 2.80-margin, ori =  math.pi/2},
    M4  = {x = 1.68-margin, y = 2.80,        ori = 0},
    M5  = {x = 2.80+margin, y = 2.80,        ori =  math.pi},
    M6  = {x = 3.92,        y = 2.80+margin, ori = -math.pi/2},
    M7  = {x = 5.04+margin, y = 2.80,        ori =  math.pi},
    M8  = {x = 1.68,        y = 3.92-margin, ori =  math.pi/2},
    M9  = {x = 3.92,        y = 3.92+margin, ori = -math.pi/2},
    M10 = {x = 5.04-margin, y = 3.92,        ori = 0},
    R1  = {x = 0.20+margin, y = 0.20+margin, ori = -math.pi*(3/4)},
    R2  = {x = 5.40-margin, y = 5.40-margin, ori = math.pi/4 },
    T   = {x = 5.40+margin, y = 0.20+margin, ori = -math.pi/4},
    EGI = {x = 3.60+margin, y = 5.35,        ori = math.pi},
    D1 =  {x = 3.15,        y = 0.26+margin, ori = -math.pi/2},
    D2 =  {x = 2.80,        y = 0.26+margin, ori = -math.pi/2},
    D3 =  {x = 2.45,        y = 0.26+margin, ori = -math.pi/2},
    
    m1 = M1, m2 = M2, m3 = M3, m4 = M4, m5 = M5, m6 = M6, m7 = M7, m8 = M8, m9 = M9,
    m10 = M10,
    
    TEST = T, Test = T,

    ExpressGoodInsertion = EGI,

    Delivery1 = D1, Delivery2 = D2, Delivery3 = D3
}

local machine_pos.fields = {
	size = 1.2
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
    
    m1 = M1, m2 = M2, m3 = M3, m4 = M4, m5 = M5, m6 = M6, m7 = M7, m8 = M8, m9 = M9,
    m10 = M10,
    
    TEST = T, Test = T,

    ExpressGoodInsertion = EGI,

    Delivery1 = D1, Delivery2 = D2, Delivery3 = D3
}


return machine_pos
