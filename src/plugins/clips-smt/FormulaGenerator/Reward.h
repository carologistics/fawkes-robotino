#ifndef REWARD_H
#define REWARD_H

#include <vector>

class Reward {
public:
    Reward();
    virtual ~Reward();
    
    int getDelayedDelivery() const;
    int getDelivery() const;
    int getMountCap() const;
    int getFinishCXPreCap(int x) const;
    int getFinishCCXStep(int x) const;
    int getAdditionalBase() const;
    
private:

    int additionalBase = 2;
    std::vector<int> finishCCXStep = {5, 10, 20};
    std::vector<int> finishCXPreCap = {10, 30, 80};
    int mountCap = 10;
    int delivery = 20;
    int delayedDelivery = 1;
};

#endif /* REWARD_H */

