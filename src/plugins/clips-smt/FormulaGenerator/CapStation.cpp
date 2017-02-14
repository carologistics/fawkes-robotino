/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   CapStation.cpp
 * Author: leonard
 * 
 * Created on February 7, 2017, 3:56 PM
 */

#include "CapStation.h"

CapStation::CapStation(int id) : Station(id){
    setType("cs");
}

CapStation::~CapStation() {
}

