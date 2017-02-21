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

Robot::Robot(int id) : Machine(id) {
    this->setType("r");
}

Robot::~Robot() {
}

