/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Station.h
 * Author: leonard
 *
 * Created on February 9, 2017, 3:36 PM
 */

#ifndef STATION_H
#define STATION_H

#include "Machine.h"

class Station : public Machine{
public:
    Station(int id);
    virtual ~Station();
private:

};

#endif /* STATION_H */

