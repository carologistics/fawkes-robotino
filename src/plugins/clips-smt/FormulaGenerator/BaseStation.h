/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   BaseStation.h
 * Author: leonard
 *
 * Created on February 14, 2017, 12:20 PM
 */

#ifndef BASESTATION_H
#define BASESTATION_H

#include "Station.h"

class BaseStation : public Station {
public:
    BaseStation(int id);

    virtual ~BaseStation();
private:

};

#endif /* BASESTATION_H */

