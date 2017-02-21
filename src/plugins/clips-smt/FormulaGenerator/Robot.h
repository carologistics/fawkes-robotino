/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Robot.h
 * Author: leonard
 *
 * Created on February 18, 2017, 12:33 AM
 */

#ifndef ROBOT_H
#define ROBOT_H

#include "Machine.h"

class Robot;
typedef std::shared_ptr<Robot> robot_ptr;

class Robot : public Machine{
public:
    Robot(int id);
    virtual ~Robot();
private:

};

#endif /* ROBOT_H */

