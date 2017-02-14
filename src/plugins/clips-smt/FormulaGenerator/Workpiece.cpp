/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Workpiece.cpp
 * Author: leonard
 * 
 * Created on January 31, 2017, 11:38 PM
 */

#include "Workpiece.h"

Workpiece::Workpiece(){
    
}

Workpiece::Workpiece(Color pBase, Color ring0, Color ring1, Color ring2, Color pCap) {
    base = pBase;
    std::vector<Color> ring(3);
    ring[0] = ring0;
    ring[1] = ring1;
    ring[2] = ring2;
    this->ring = ring;
    cap = pCap;
}

Workpiece::Workpiece(const Workpiece& orig) {
}

Workpiece::~Workpiece() {
}

