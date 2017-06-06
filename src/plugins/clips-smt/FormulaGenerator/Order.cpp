/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Order.cpp
 * Author: leonard
 * 
 * Created on February 18, 2017, 2:00 AM
 */

#include "Order.h"
#include "RingStation.h"

Order::Order(int id, Workpiece product, Time deadline) {
    setId(id);
    setProduct(product);
    setDeadline(deadline);
}

Order::~Order() {
}

int Order::getId() const {
    return this->id;
}

Time Order::getDeadline() const {
    return this->deadline;
}

Workpiece Order::getProduct() const {
    return this->product;
}

std::string Order::getVarIdentifier() {
    return "o" + std::to_string(getId());
}

Workpiece::Color Order::getBaseColorReq() const {
    return getProduct().getBaseColor();
}

Workpiece::Color Order::getRingColorReq(int number) const {
    return getProduct().getRingColor(number);
}

Workpiece::Color Order::getCapColorReq() const {
    return getProduct().getCapColor();
}

std::vector<Workpiece::Color> Order::getRingColorReq() const {
    return getProduct().getRings();
}

void Order::setId(int id) {
    this->id = id;
}

void Order::setDeadline(Time deadline) {
    this->deadline = deadline;
}

void Order::setProduct(Workpiece& product) {
    this->product = product;
}

void Order::setBaseColorReq(Workpiece::Color color) {
    getProduct().setBaseColor(color);
}

void Order::setRingColorReq(Workpiece::Color color, int number) {
    getProduct().setRingColor(color, number);
}

void Order::setCapColorReq(Workpiece::Color color) {
    getProduct().setCapColor(color);
}

Order::State Order::getSetupRingId(int ringPosition) const {
    State setupRingId = NOTDEFINED;
    switch (ringPosition) {
        case 0: setupRingId = SETUP_RING0;
            break;
        case 1: setupRingId = SETUP_RING1;
            break;
        case 2: setupRingId = SETUP_RING2;
            break;
    }
    return setupRingId;
}

Order::State Order::getRingId(int ringPosition) const {
    State ringId = NOTDEFINED;
    switch (ringPosition) {
        case 0: ringId = RING0;
            break;
        case 1: ringId = RING1;
            break;
        case 2: ringId = RING2;
            break;
    }
    return ringId;
}

Order::State Order::getFedBaseId(int ringPosition, int addBaseNumber) const {
    State fedBaseId = NOTDEFINED;
    switch (ringPosition) {
        case 0:
            switch (addBaseNumber) {
                case 0: fedBaseId = FED_BASE0_RING0;
                    break;
                case 1: fedBaseId = FED_BASE1_RING0;
                    break;
            }
            break;
        case 1:
            switch (addBaseNumber) {
                case 0: fedBaseId = FED_BASE0_RING1;
                    break;
                case 1: fedBaseId = FED_BASE1_RING1;
                    break;
            }
            break;
        case 2:
            switch (addBaseNumber) {
                case 0: fedBaseId = FED_BASE0_RING2;
                    break;
                case 1: fedBaseId = FED_BASE1_RING2;
                    break;
            }
            break;
    }
    return fedBaseId;
}

Order::State Order::getCollectBaseId(int ringPosition, int addBaseNumber) const {
    State collectBaseId = NOTDEFINED;
    switch (ringPosition) {
        case 0:
            switch (addBaseNumber) {
                case 0: collectBaseId = COLLECT_BASE0_RING0;
                    break;
                case 1: collectBaseId = COLLECT_BASE1_RING0;
                    break;
            }
            break;
        case 1:
            switch (addBaseNumber) {
                case 0: collectBaseId = COLLECT_BASE0_RING1;
                    break;
                case 1: collectBaseId = COLLECT_BASE1_RING1;
                    break;
            }
            break;
        case 2:
            switch (addBaseNumber) {
                case 0: collectBaseId = COLLECT_BASE0_RING2;
                    break;
                case 1: collectBaseId = COLLECT_BASE1_RING2;
                    break;
            }
            break;
    }
    return collectBaseId;
}

int Order::getRingCount() const{
    int amount = 0;
    for(auto r: getRingColorReq()){
        if(r == Workpiece::NONE) return amount;
        amount++;
    }
    return amount;
}

Order::State Order::intToState(int state) {
    switch (state) {
        case Order::NONE:                   return Order::NONE;
        case Order::BASE:                   return Order::BASE;
        case Order::SETUP_RING0:            return Order::SETUP_RING0;
        case Order::COLLECT_BASE0_RING0:    return Order::COLLECT_BASE0_RING0;
        case Order::FED_BASE0_RING0:        return Order::FED_BASE0_RING0;
        case Order::COLLECT_BASE1_RING0:    return Order::COLLECT_BASE1_RING0;
        case Order::FED_BASE1_RING0:        return Order::FED_BASE1_RING0;
        case Order::RING0:                  return Order::RING0;
        case Order::SETUP_RING1:            return Order::SETUP_RING1;
        case Order::COLLECT_BASE0_RING1:    return Order::COLLECT_BASE0_RING1;
        case Order::FED_BASE0_RING1:        return Order::FED_BASE0_RING1;
        case Order::COLLECT_BASE1_RING1:    return Order::COLLECT_BASE1_RING1;
        case Order::FED_BASE1_RING1:        return Order::FED_BASE1_RING1;
        case Order::RING1:                  return Order::RING1;
        case Order::SETUP_RING2:            return Order::SETUP_RING2;
        case Order::COLLECT_BASE0_RING2:    return Order::COLLECT_BASE0_RING2;
        case Order::FED_BASE0_RING2:        return Order::FED_BASE0_RING2;
        case Order::COLLECT_BASE1_RING2:    return Order::COLLECT_BASE1_RING2;
        case Order::FED_BASE1_RING2:        return Order::FED_BASE1_RING2;
        case Order::RING2:                  return Order::RING2;
        case Order::FEDCAP:                 return Order::FEDCAP;
        case Order::CAP:                    return Order::CAP;
        case Order::DELIVERED:              return Order::DELIVERED;
        case Order::NOTDELIVERED:           return Order::NOTDELIVERED;
        default:                            return Order::NOTDEFINED;
    }
}

std::string Order::toString() {
    std::string result;
    result += "Order: " + std::to_string(getId());
    result += "; " + getProduct().toString();
    result += "; DL: " + std::to_string(getDeadline());

    return result;
}

bool Order::operator<(const Order& rhs) const {
    return this->getId() < rhs.getId();
}

bool Order::operator==(const Order& rhs) const {
    return this->getId() == rhs.getId();
}

bool Order::operator!=(const Order& rhs) const {
    return !(*this == rhs);
}

std::string Order::toString(State s) {
    return stateNames.at(s);
}

const std::map<Order::State, std::string> Order::stateNames = {
    {Order::NONE, "NONE"},
    {Order::BASE, "BASE"},
    {Order::SETUP_RING0, "SETUP_RING0"},
    {Order::COLLECT_BASE0_RING0, "COLLECT_BASE0_RING0"},
    {Order::FED_BASE0_RING0, "FED_BASE0_RING0"},
    {Order::COLLECT_BASE1_RING0, "COLLECT_BASE1_RING0"},
    {Order::FED_BASE1_RING0, "FED_BASE1_RING0"},
    {Order::RING0, "RING0"},
    {Order::SETUP_RING1, "SETUP_RING1"},
    {Order::COLLECT_BASE0_RING1, "COLLECT_BASE0_RING1"},
    {Order::FED_BASE0_RING1, "FED_BASE0_RING1"},
    {Order::COLLECT_BASE1_RING1, "COLLECT_BASE1_RING1"},
    {Order::FED_BASE1_RING1, "FED_BASE1_RING1"},
    {Order::RING1, "RING1"},
    {Order::SETUP_RING2, "SETUP_RING2"},
    {Order::COLLECT_BASE0_RING2, "COLLECT_BASE0_RING2"},
    {Order::FED_BASE0_RING2, "FED_BASE0_RING2"},
    {Order::COLLECT_BASE1_RING2, "COLLECT_BASE1_RING2"},
    {Order::FED_BASE1_RING2, "FED_BASE1_RING2"},
    {Order::RING2, "RING2"},
    {Order::FEDCAP, "FEDCAP"},
    {Order::CAP, "CAP"},
    {Order::DELIVERED, "DELIVERED"},
    {Order::NOTDELIVERED, "NOTDELIVERED"},
    {Order::NOTDEFINED, "NOTDEFINED"},
};