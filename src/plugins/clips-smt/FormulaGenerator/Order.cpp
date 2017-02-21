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

Order::ComponentState Order::getBaseState() const {
    return this->base;
}

Order::ComponentState Order::getRingState(int number) const {
    return this->rings[number];
}

Order::ComponentState Order::getCapState() const {
    return this->cap;
}

std::vector<Order::ComponentState> Order::getRingState() const {
    return rings;
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

void Order::setBaseState(ComponentState state) {
    this->base = state;
}

void Order::setRingState(ComponentState state, int number) {
    this->rings[number] = state;
}

void Order::setCapState(ComponentState state) {
    this->cap = state;
}

std::string Order::toString() {
    std::string result;
    result += "ID: " + std::to_string(getId());
    result += "\nDeadline: " + std::to_string(getDeadline());
    result += "\nRequirements:\n";
    result += getProduct().toString();
    result += "\nState:\n";
    result += "Base: " + ComponentStateNames[getBaseState()];
    result += "\nRings: ";
    for (auto i = 0; i < getRingState().size(); i++) {
        result += ComponentStateNames[getRingState(i)] + "  ";
    }
    result += "\nBase: " + ComponentStateNames[getCapState()];

    return result;
}