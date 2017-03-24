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


#include "Station.h"
#include "Workpiece.h"
#include <set>


class BaseStation;
typedef std::shared_ptr<BaseStation> baseStation_ptr;

class BaseStation : public Station {
public:
    BaseStation(int id);
    virtual ~BaseStation();

    int getDispenseBaseTime() const;
    void setPossibleBaseColors(std::set<Workpiece::Color> possibleBaseColors);
    void addPossibleBaseColor(Workpiece::Color color);

    void setDispenseBaseTime(int time);
    std::set<Workpiece::Color> getPossibleBaseColors() const;
    bool isPossibleBaseColor(Workpiece::Color Color) const;

private:
    int dispenseBaseTime;

    std::set<Workpiece::Color> possibleBaseColors;

};

#endif /* BASESTATION_H */

