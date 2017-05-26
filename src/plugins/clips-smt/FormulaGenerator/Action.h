/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Action.h
 * Author: leonard
 *
 * Created on May 22, 2017, 6:50 PM
 */

#ifndef ACTION_H
#define ACTION_H

#include "StepFormula.h"

class Action {
public:

    enum ActionType {
        NOTDEFINED= -1, NONE, COLLECTBASE, FEEDCAP, MOUNTCAP, PICKWORKPIECE, DELIVERPRODUCT
    };

    Action(ActionType actionType) : actionType(actionType){
    }
    
    Action(ActionType actionType, robot_ptr robot, station_ptr station, Workpiece::Color color) :
    actionType(actionType), robot(robot), station(station), color(color) {
    }

    virtual ~Action();
    
    bool isNotDefined() const;
    bool isNone() const;
    bool isCollectBase() const;
    bool isFeedCap() const;
    bool isMountCap() const;
    bool isPickWorkpiece() const;
    bool isDeliverProduct() const;
    

    ActionType getActionType() const;
    Workpiece::Color getColor() const;
    robot_ptr getRobot() const;
    station_ptr getStation() const;

    static string toString(const Action::ActionType actionType);
    string toString() const;
private:

    ActionType actionType;
    robot_ptr robot = nullptr;
    station_ptr station = nullptr;
    Workpiece::Color color;
    
    static const std::map<Action::ActionType, std::string> actionTypeNames;

};

#endif /* ACTION_H */

