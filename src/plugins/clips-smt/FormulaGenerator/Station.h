#ifndef STATION_H
#define STATION_H

#include "Machine.h"

class Station;
typedef std::shared_ptr<Station> station_ptr;

class Station : public Machine {
public:
    Station(int id, std::string type);
    virtual ~Station();

    void setOccupiedUntil(Time time);
    Time getOccupiedUntil() const;

    std::string toString();

private:
    /* If brocken do not add to GameData, or if broken and the time the machine is 
     * available again is known, set occupiedUntil to it.
     * */
    Time occupiedUntil;
};

#endif /* STATION_H */

