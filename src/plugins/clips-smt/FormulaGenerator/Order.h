/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Order.h
 * Author: leonard
 *
 * Created on February 18, 2017, 2:00 AM
 */

#ifndef ORDER_H
#define ORDER_H

#include <memory>
#include <string>

#include "Workpiece.h"

class Order;
typedef std::shared_ptr<Order> order_ptr;

class Order {
public:

    enum ComponentState {
        NONE, NOTFINISHED, FINISHED, SETUP, DELIVERED, LAST_ENTRY = DELIVERED
    };
    Order(int id, Workpiece product, int deadline);
    virtual ~Order();

    int getId() const;
    int getDeadline() const;
    Workpiece getProduct() const;
    std::string getVarIdentifier();

    Workpiece::Color getBaseColorReq() const;
    Workpiece::Color getRingColorReq(int number) const;
    Workpiece::Color getCapColorReq() const;

    std::vector<Workpiece::Color> getRingColorReq() const;

    ComponentState getBaseState() const;
    ComponentState getRingState(int number) const;
    ComponentState getCapState() const;

    std::vector<ComponentState> getRingState() const;

    void setId(int id);
    void setDeadline(int deadline);
    void setProduct(Workpiece& w);

    void setBaseColorReq(Workpiece::Color color);
    void setRingColorReq(Workpiece::Color color, int number);
    void setCapColorReq(Workpiece::Color color);

    void setBaseState(ComponentState state);
    void setRingState(ComponentState state, int number);
    void setCapState(ComponentState state);

    std::string toString();

private:
    int id = -1;
    int deadline = -1;
    Workpiece product;

    /* fill this right before the 0th step formula is created, 
     * or get this information on the fly out of the workpieces in the machines
     * while formula generation */
    ComponentState base = NONE;
    std::vector<ComponentState> rings
            = std::vector<ComponentState>(Workpiece::getMaxRingNumber(), ComponentState(NONE));
    ComponentState cap = NONE;

    //for toString
    std::map<ComponentState, std::string> ComponentStateNames = {
        {NONE, "NONE"},
        {NOTFINISHED, "NOTFINISHED"},
        {FINISHED, "FINISHED"},
        {SETUP, "SETUP"},
        {DELIVERED, "DELIVERED"}
    };
};

#endif /* ORDER_H */

