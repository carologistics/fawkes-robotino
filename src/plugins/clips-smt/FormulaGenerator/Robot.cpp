/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Robot.cpp
 * Author: leonard
 * 
 * Created on February 18, 2017, 12:33 AM
 */

#include "Robot.h"
#include <assert.h>

Robot::Robot(int id) : Machine("r", id) {
}

Robot::~Robot() {
}

Time Robot::getFeedWorkpieceTime() const {
    return feedWorkpieceTime;
}

void Robot::setFeedWorkpieceTime(Time feedWorkpieceTime) {
    this->feedWorkpieceTime = feedWorkpieceTime;
}

Time Robot::getTakeWorkpieceTime() const {
    return takeWorkpieceTime;
}

void Robot::setTakeWorkpieceTime(Time takeWorkpiece) {
    this->takeWorkpieceTime = takeWorkpiece;
}

std::string Robot::toString(){
    std::string result;
    result += this->Machine::toString();
    result += "; Feed: " + std::to_string(getFeedWorkpieceTime());
    result += "; Take: " + std::to_string(getTakeWorkpieceTime());
    return result;
}
