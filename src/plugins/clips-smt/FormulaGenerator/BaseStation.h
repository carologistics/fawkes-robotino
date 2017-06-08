/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   BaseStation.h
 * Author: leonard
 *
 * Created on February 18, 2017, 2:02 AM
 */

#ifndef BASESTATION_H
#define BASESTATION_H

#include <set>

#include "Station.h"
#include "Workpiece.h"

class BaseStation;
typedef std::shared_ptr<BaseStation> baseStation_ptr;

class BaseStation : public Station {
public:
    BaseStation(int id);
    virtual ~BaseStation();

    Time getDispenseBaseTime() const;
    void setPossibleBaseColors(std::set<Workpiece::Color> possibleBaseColors);
    void addPossibleBaseColor(Workpiece::Color color);

    void setDispenseBaseTime(Time time);
    std::set<Workpiece::Color> getPossibleBaseColors() const;
    bool isPossibleBaseColor(Workpiece::Color Color) const;
    void setColorForRingStation(Workpiece::Color colorForRingStation);
    Workpiece::Color getColorForRingStation() const; 
    
    std::string toString();

private:
    Time dispenseBaseTime = 0;

    std::set<Workpiece::Color> possibleBaseColors;
    //which color should a robot choose if he needs a base for the additional base requirement of a ring station 
    Workpiece::Color colorForRingStation; //@todo initialize

};

#endif /* BASESTATION_H */

