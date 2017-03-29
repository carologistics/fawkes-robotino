/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Reward.cpp
 * Author: leonard
 * 
 * Created on March 5, 2017, 11:52 AM
 */

#include "Reward.h"

Reward::Reward() {
}

Reward::~Reward() {
}

int Reward::getDelayedDelivery() const {
    return delayedDelivery;
}

int Reward::getDelivery() const {
    return delivery;
}

int Reward::getMountCap() const {
    return mountCap;
}

int Reward::getFinishCXPreCap(int x) const {
    return finishCXPreCap.at(x);
}

int Reward::getFinishCCXStep(int x) const {
    return finishCCXStep.at(x);
}

int Reward::getAdditionalBase() const {
    return additionalBase;
}

