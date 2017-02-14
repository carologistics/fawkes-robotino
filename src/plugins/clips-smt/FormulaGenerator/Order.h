/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Order.h
 * Author: leonard
 *
 * Created on February 6, 2017, 7:45 PM
 */

#ifndef ORDER_H
#define ORDER_H
#include "Workpiece.h"

class Order {
public:
    Order(int pId, Workpiece &pProduct);
    Order(const Order& orig);
    virtual ~Order();
    
    int getId(){return id;}
    
    Workpiece::Color getBaseColorReq(){return product.getBaseColor();}
    Workpiece::Color getRingColorReq(int i){return product.getRingColor(i);}
    Workpiece::Color getCapColorReq(){return product.getCapColor();}
    
    enum ComponentState {
        NONE,
        NOTFINISHED,
        FINISHED,
        SETUP,
        DELIVERED,
    };
    
private:
    int id = -1;
    //Workpiece product;
    Workpiece product;
    
    ComponentState Base;
    ComponentState Ring1;
    ComponentState Ring2;
    ComponentState Ring3;
    ComponentState Cap;


};

#endif /* ORDER_H */

