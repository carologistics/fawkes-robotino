/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   DeliveryStation.h
 * Author: leonard
 *
 * Created on February 18, 2017, 2:02 AM
 */

#ifndef DELIVERYSTATION_H
#define DELIVERYSTATION_H

#include "Station.h"
class DeliveryStation;
typedef std::shared_ptr<DeliveryStation> deliveryStation_ptr;

class DeliveryStation : public Station{
public:
    DeliveryStation(int id);
    virtual ~DeliveryStation();
private:

};

#endif /* DELIVERYSTATION_H */

