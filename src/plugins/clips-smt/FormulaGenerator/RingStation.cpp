/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   RingStation.cpp
 * Author: leonard
 * 
 * Created on February 18, 2017, 2:02 AM
 */

#include <assert.h> 

#include "RingStation.h"

RingStation::RingStation(int id) : Station(id) {
    this->setType("rs");
}

RingStation::~RingStation() {

}

void RingStation::setFeedBaseTime(int time) {
    this->feedBaseTime = time;
}

void RingStation::setMountRingTime(int time) {
    this->mountRingTime = time;
}

void RingStation::setPossibleRingColors(std::map<Workpiece::Color, int> possibleRingColors) {
    this->possibleRingColors = possibleRingColors;
}

void RingStation::addPossibleRingColor(Workpiece::Color color, int additionalBases) {
    this->possibleRingColors[color] = additionalBases;
}

void RingStation::setAdditinalBasesFed(int amount) {
    this->additinalBasesFed = amount;
}

void RingStation::setRingColorSetup(Workpiece::Color color) {
    //possibleRingColors has to be set before 
    assert(isPossibleRingColor(color));
    this->ringColorSetup = color;
}

int RingStation::getFeedBaseTime() const {
    return this->feedBaseTime;
}

int RingStation::getMountRingTime() const {
    return this->mountRingTime;
}

std::map<Workpiece::Color, int> RingStation::getPossibleRingColors() const {
    return this->possibleRingColors;
}

bool RingStation::isPossibleRingColor(Workpiece::Color Color) const {
    return possibleRingColors.find(Color) != possibleRingColors.end();
}

int RingStation::getAdditionalBasesFed() const {
    return this->additinalBasesFed;
}

Workpiece::Color RingStation::getRingColorSetup() const {
    return this->ringColorSetup;
}

int RingStation::getNeededAdditinalBases(Workpiece::Color color) {
    if(color == Workpiece::NONE)
        return 0;
    return this->possibleRingColors[color];
}

int RingStation::getReqBases(){
    assert(getNeededAdditinalBases(getRingColorSetup()) - getAdditionalBasesFed() >= 0);
    return getNeededAdditinalBases(getRingColorSetup()) - getAdditionalBasesFed();
}

bool RingStation::readyToMountRing(){
    return getNeededAdditinalBases(getRingColorSetup()) == getAdditionalBasesFed();
}