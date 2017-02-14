/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   RingStation.h
 * Author: leonard
 *
 * Created on February 9, 2017, 3:37 PM
 */

#ifndef RINGSTATION_H
#define RINGSTATION_H

#include "Station.h"
#include "Workpiece.h"

class RingStation : public Station{
public:
    RingStation(int id);
    virtual ~RingStation();
    
    int getReqBases(Workpiece::Color c){return reqBases[c];}
    void setReqBases(Workpiece::Color c, int i){reqBases[c] = i;}
private:
    vector<int> reqBases;
};

#endif /* RINGSTATION_H */

