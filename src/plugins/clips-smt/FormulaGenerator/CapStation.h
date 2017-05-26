#ifndef CAPSTATION_H
#define CAPSTATION_H

#include <set>
#include "Station.h"
#include "Workpiece.h"

class CapStation;
typedef std::shared_ptr<CapStation> capStation_ptr;

class CapStation : public Station {
public:
    CapStation(int id);
    virtual ~CapStation();
    
    void setFeedCapTime(Time time);
    void setMountCapTime(Time time);
    void setPossibleCapColors(std::set<Workpiece::Color> possibleCapColors);
    void addPossibleCapColor(Workpiece::Color color);
    void setFedCapColor(Workpiece::Color color);
    
    Time getFeedCapTime() const;
    Time getMountCapTime() const;
    std::set<Workpiece::Color> getPossibleCapColors() const;
    bool isPossibleCapColor(Workpiece::Color Color) const;
    Workpiece::Color getFedCapColor() const;
    bool readyToMountRing();
    
private:
    
    //are this times identical?
    Time feedCapTime;
    Time mountCapTime;  
    
    //the color of the cap the station is able to mount
    std::set<Workpiece::Color> possibleCapColors;
    
    // the color of the cap the station is fed with
    Workpiece::Color fedCapColor = Workpiece::NONE;
};

#endif /* CAPSTATION_H */

