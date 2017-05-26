/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Action.cpp
 * Author: leonard
 * 
 * Created on May 22, 2017, 6:50 PM
 */

#include "Action.h"


Action::~Action() {
}

bool Action::isNotDefined() const {
    return getActionType() == ActionType::NOTDEFINED;
}

bool Action::isNone() const {
    return getActionType() == ActionType::NONE;
}

bool Action::isCollectBase() const {
    return getActionType() == ActionType::COLLECTBASE;
}

bool Action::isFeedCap() const {
    return getActionType() == ActionType::FEEDCAP;
}

bool Action::isMountCap() const {
    return getActionType() == ActionType::MOUNTCAP;
}

bool Action::isPickWorkpiece() const {
    return getActionType() == ActionType::PICKWORKPIECE;
}

bool Action::isDeliverProduct() const {
    return getActionType() == ActionType::DELIVERPRODUCT;
}

Action::ActionType Action::getActionType() const {
    return actionType;
}

Workpiece::Color Action::getColor() const {
    return color;
}

robot_ptr Action::getRobot() const {
    return robot;
}

station_ptr Action::getStation() const {
    return station;
}

string Action::toString(const Action::ActionType actionType) {
    return actionTypeNames.at(actionType);
}

string Action::toString() const {
    string output = "(";
    if (getActionType() == ActionType::NOTDEFINED) {
        output = Action::toString(getActionType());
    } else if(getActionType() == ActionType::NONE) {
        output = Action::toString(getActionType());
    }
    else {
        output += Action::toString(getActionType()) + ", ";
        output += getRobot()->getVarIdentifier() + ", ";
        output += getStation()->getVarIdentifier() + ", ";
        output += Workpiece::toString(getColor()) + ")";
    }
    return output;
}

const std::map<Action::ActionType, std::string> Action::actionTypeNames = {
    {Action::NOTDEFINED, "NOTDEFINED"},
    {Action::NONE, "NONE"},
    {Action::COLLECTBASE, "COLLECTBASE"},
    {Action::FEEDCAP, "FEEDCAP"},
    {Action::MOUNTCAP, "MOUNTCAP"},
    {Action::PICKWORKPIECE, "PICKWORKPIECE"},
    {Action::DELIVERPRODUCT, "DELIVERPRODUCT"},
};