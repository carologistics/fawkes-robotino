/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Order.cpp
 * Author: leonard
 * 
 * Created on February 18, 2017, 2:00 AM
 */

#include "Order.h"

Order::Order(int id, Workpiece product, int deadline) {
    setId(id);
    setProduct(product);
    setDeadline(deadline);
}

Order::~Order() {
}

int Order::getId() const {
    return this->id;
}

int Order::getDeadline() const {
    return this->deadline;
}

Workpiece Order::getProduct() const {
    return this->product;
}

std::string Order::getVarIdentifier(){
    return "o" + std::to_string(getId());
}

Workpiece::Color Order::getBaseColorReq() const {
    return getProduct().getBaseColor();
}

Workpiece::Color Order::getRingColorReq(int number) const {
    return getProduct().getRingColor(number);
}

Workpiece::Color Order::getCapColorReq() const {
    return getProduct().getCapColor();
}

std::vector<Workpiece::Color> Order::getRingColorReq() const {
    return getProduct().getRings();
}

void Order::setId(int id) {
    this->id = id;
}

void Order::setDeadline(int deadline) {
    this->deadline = deadline;
}

void Order::setProduct(Workpiece& product) {
    this->product = product;
}

void Order::setBaseColorReq(Workpiece::Color color) {
    getProduct().setBaseColor(color);
}

void Order::setRingColorReq(Workpiece::Color color, int number) {
    getProduct().setRingColor(color, number);
}

void Order::setCapColorReq(Workpiece::Color color) {
    getProduct().setCapColor(color);
}



std::string Order::toString() {
    std::string result;
    result += "ID: " + std::to_string(getId());
    result += "\nDeadline: " + std::to_string(getDeadline());
    result += "\nRequirements:\n";
    result += getProduct().toString();

    return result;
}