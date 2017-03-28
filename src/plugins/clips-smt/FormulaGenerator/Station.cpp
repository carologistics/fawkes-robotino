/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Station.cpp
 * Author: leonard
 * 
 * Created on February 18, 2017, 2:01 AM
 */

#include "Station.h"

Station::Station(int id, std::string type) : Machine(type, id) {
}

Station::~Station() {
}

void Station::setOccupiedUntil(int time){
    this->occupiedUntil = time;
}
int Station::getOccupiedUntil() const{
    return this->occupiedUntil;
}