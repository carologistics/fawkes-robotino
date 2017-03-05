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

int Reward::getDelayeDelivery() const {
    return delayeDelivery;
}

int Reward::getDelivery() const {
    return delivery;
}

int Reward::getMountCap() const {
    return mountCap;
}

int Reward::getFinishC3PreCap() const {
    return finishC3PreCap;
}

int Reward::getFinishC2PreCap() const {
    return finishC2PreCap;
}

int Reward::getFinishC1PreCap() const {
    return finishC1PreCap;
}

int Reward::getFinishCCStep(int i) const {
    int result;
    switch (i) {
        case 0:
            result = finishCC0Step;
            break;
        case 1:
            result = finishCC1Step;
            break;
        case 2:
            result = finishCC2Step;
            break;
    }
}

int Reward::getAdditionalBase() const {
    return additionalBase;
}

