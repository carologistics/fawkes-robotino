/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Reward.h
 * Author: leonard
 *
 * Created on March 5, 2017, 11:52 AM
 */

#ifndef REWARD_H
#define REWARD_H

class Reward {
public:
    Reward();
    virtual ~Reward();
    
    int getDelayeDelivery() const;
    int getDelivery() const;
    int getMountCap() const;
    int getFinishC3PreCap() const;
    int getFinishC2PreCap() const;
    int getFinishC1PreCap() const;
    int getFinishCCStep(int i) const;
    int getAdditionalBase() const;
    
private:

    int additionalBase = 2;
    int finishCC0Step = 5;
    int finishCC1Step = 10;
    int finishCC2Step = 20;
    int finishC1PreCap = 10;
    int finishC2PreCap = 30;
    int finishC3PreCap = 80;
    int mountCap = 10;
    int delivery = 20;
    int delayeDelivery = 1;
};

#endif /* REWARD_H */

