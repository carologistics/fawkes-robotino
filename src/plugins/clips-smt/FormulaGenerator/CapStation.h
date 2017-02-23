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
    CapStation(int id, int feedCapTime);
    virtual ~CapStation();
    
    void setFeedCapTime(int time);
    void setMountCapTime(int time);
    void setPossibleCapColors(std::set<Workpiece::Color> possibleCapColors);
    void addPossibleCapColor(Workpiece::Color color);
    void setFedCapColor(Workpiece::Color color);
    
    int getFeedCapTime() const;
    int getMountCapTime() const;
    std::set<Workpiece::Color> getPossibleCapColors() const;
    bool isPossibleCapColor(Workpiece::Color Color) const;
    Workpiece::Color getFedCapColor() const;
    bool readyToMountRing();
    
private:
    
    //are this times identical?
    int feedCapTime;
    int mountCapTime;  
    
    //the color of the cap the station is able to mount
    std::set<Workpiece::Color> possibleCapColors;
    
    /* setup color and feed cap is encoded as monolithic action, how to react if initial 
     * state is only the setup of color without fed cap?*/
    // the color of the cap the station is fed with
    Workpiece::Color fedCapColor = Workpiece::NONE;
};

#endif /* CAPSTATION_H */

