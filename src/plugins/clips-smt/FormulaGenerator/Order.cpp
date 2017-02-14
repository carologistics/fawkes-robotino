/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Order.cpp
 * Author: leonard
 * 
 * Created on February 6, 2017, 7:45 PM
 */

#include "Order.h"

#include <iostream>

Order::Order(int pId, Workpiece &pProduct) {
    id = pId;
    product = pProduct;
}

Order::Order(const Order& orig) {

}

Order::~Order() {
}

