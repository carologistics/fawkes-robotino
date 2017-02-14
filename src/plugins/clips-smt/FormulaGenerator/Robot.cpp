/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Robot.cpp
 * Author: leonard
 * 
 * Created on January 31, 2017, 11:38 PM
 */

#include "Robot.h"

Robot::Robot(int id) : Machine(id){
    setType("r");
}

Robot::~Robot() {
}

