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

DeliveryStation::DeliveryStation(int id) : Station(id, "ds") {
}

DeliveryStation::~DeliveryStation() {
}

Time DeliveryStation::getDeliverProductTime() const {
    return deliverProductTime;
}

void DeliveryStation::setDeliverProductTime(Time deliverProductTime) {
    this->deliverProductTime = deliverProductTime;
}


std::string DeliveryStation::toString() {
    std::string result;
    result += this->Station::toString();
    result += "; Deliver: " + std::to_string(getDeliverProductTime());

    return result;
}