/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   RingStation.cpp
 * Author: leonard
 * 
 * Created on February 9, 2017, 3:37 PM
 */

#include "RingStation.h"

RingStation::RingStation(int id) : Station(id){
    setType("cs");
    vector<int> reqBases(Workpiece::LAST_ENTRY);
    for ( auto &i : reqBases ){
        i = -1;
    }
    this->reqBases = reqBases;
}

RingStation::~RingStation() {
}



