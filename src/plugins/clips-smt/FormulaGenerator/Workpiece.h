/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Workpiece.h
 * Author: leonard
 *
 * Created on January 31, 2017, 11:38 PM
 */

#ifndef WORKPIECE_H
#define WORKPIECE_H
#include <vector>

using namespace std;

class Workpiece {
public:
    enum Color {
        NONE,
        RED,
        BLACK,
        SILVER,
        TRANSPARENT,
        BLUE,
        GREEN,
        YELLOW,
        ORANGE,
        GREY,
        LAST_ENTRY = GREY
    };
    Workpiece();
    Workpiece(Color base, Color ring0, Color ring1, Color ring2, Color Cap);
    Workpiece(const Workpiece& orig);
    virtual ~Workpiece();

    Color getBaseColor() {
        return base;
    }

    Color getRingColor(int i) {
        return ring[i];
    }
    
    Color getCapColor() {
        return cap;
    }

private:

    Color base = NONE;
    vector<Color> ring;
    Color cap = NONE;

};

#endif /* WORKPIECE_H */

