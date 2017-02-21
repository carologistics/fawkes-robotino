#ifndef STATION_H
#define STATION_H

#include "Machine.h"

class Station : public Machine{
public:
    Station(int id);
    virtual ~Station();
    
    void setOccupiedUntil(int time);
    int getOccupiedUntil() const;
    
private:
    /* If brocken do not add to GameData, or if broken and the time the machine is 
     * available again is known, set occupiedUntil to it.
     * */
    int occupiedUntil;
};

#endif /* STATION_H */

