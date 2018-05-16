#ifndef ROBOT_H
#define ROBOT_H

#include "Machine.h"

class Robot;
typedef std::shared_ptr<Robot> robot_ptr;

class Robot : public Machine{
public:
    Robot(int id);
    virtual ~Robot();
    
    Time getFeedWorkpieceTime() const;
    void setFeedWorkpieceTime(Time feedWorkpieceTime);
    Time getTakeWorkpieceTime() const;
    void setTakeWorkpieceTime(Time takeWorkpiece);
    std::string toString();
    
private:
    Time feedWorkpieceTime = 1;
    Time takeWorkpieceTime = 1;
};

#endif /* ROBOT_H */

