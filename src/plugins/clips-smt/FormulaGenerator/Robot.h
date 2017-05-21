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

