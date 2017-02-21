/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   DeliveryStation.cpp
 * Author: leonard
 * 
 * Created on February 18, 2017, 2:02 AM
 */

#include "DeliveryStation.h"

DeliveryStation::DeliveryStation(int id) : Station(id) {
    this->setType("ds");
}

DeliveryStation::~DeliveryStation() {
}

