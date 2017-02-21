/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   BaseStation.cpp
 * Author: leonard
 * 
 * Created on February 18, 2017, 2:02 AM
 */

#include "BaseStation.h"

BaseStation::BaseStation(int id) : Station(id) {
    this->setType("bs");
}

BaseStation::~BaseStation() {
}

int BaseStation::getDispenseBaseTime() const {
    return this->dispenseBaseTime;
}

void BaseStation::setPossibleBaseColors(std::set<Workpiece::Color> possibleBaseColors){
     this->possibleBaseColors = possibleBaseColors;
}
void BaseStation::addPossibleBaseColor(Workpiece::Color color){
    this->possibleBaseColors.insert(color);
}

void BaseStation::setDispenseBaseTime(int time) {
    this->dispenseBaseTime = time;
}

std::set<Workpiece::Color> BaseStation::getPossibleBaseColors() const{
        return this->possibleBaseColors;
}

bool BaseStation::isPossibleBaseColor(Workpiece::Color Color) const{
    return possibleBaseColors.find(Color) != possibleBaseColors.end();
}