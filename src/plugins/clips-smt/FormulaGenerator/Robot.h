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
    
private:
    Time feedWorkpieceTime;
    Time takeWorkpieceTime;
};

#endif /* ROBOT_H */

