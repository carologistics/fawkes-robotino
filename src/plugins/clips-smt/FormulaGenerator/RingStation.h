#ifndef RINGSTATION_H
#define RINGSTATION_H

#include <map>

#include "Station.h"
#include "Workpiece.h"

class RingStation;
typedef std::shared_ptr<RingStation> ringStation_ptr;

class RingStation : public Station {
public:
    RingStation(int id);
    virtual ~RingStation();

    void setFeedBaseTime(Time time);
    void setMountRingTime(Time time);
    void setPossibleRingColors(std::map<Workpiece::Color, int> possibleRingColors);
    void addPossibleRingColor(Workpiece::Color color, int additionalBases);
    void setAdditinalBasesFed(int amount);
    void setRingColorSetup(Workpiece::Color color);

    Time getFeedBaseTime() const;
    Time getMountRingTime() const;
    std::map<Workpiece::Color, int> getPossibleRingColors() const;
    bool isPossibleRingColor(Workpiece::Color Color) const;
    int getAdditionalBasesFed() const;
    Workpiece::Color getRingColorSetup() const;
    int getNeededAdditinalBases(Workpiece::Color color) const;
    int getReqBases() const;
    bool readyToMountRing() const;
   
private:
    int feedBaseTime;
    int mountRingTime;

    // possible ring colors key = color, value = needed additional bases for this color
    std::map<Workpiece::Color, int> possibleRingColors;
    
    //additional bases on ring station 
    int additionalBasesFed = 0;
    
    //color the ring station is set up for
    Workpiece::Color ringColorSetup = Workpiece::NONE;
};

#endif /* RINGSTATION_H */

