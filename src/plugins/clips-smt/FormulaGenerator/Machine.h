/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Machine.h
 * Author: leonard
 *
 * Created on February 6, 2017, 5:08 PM
 */

#ifndef MACHINE_H
#define MACHINE_H

#include <string>

using namespace std;

class Machine {
public:
    Machine();
    Machine(int pId);
    Machine(const Machine& orig);
    virtual ~Machine();
    
    int getId(){return id;}
    string getType(){return type;}
    void setType(string pType){type = pType;}
    void setId(int pId){id = pId;}
private:
    int id = -1;
    string type = "";

};

#endif /* MACHINE_H */

