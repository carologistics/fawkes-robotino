
#include <assert.h>
#include "CapStation.h"

CapStation::CapStation(int id) : Station(id, "cs") {
}

CapStation::~CapStation() {
}

void CapStation::setFeedCapTime(Time time) {
    this->feedCapTime = time;
}

void CapStation::setMountCapTime(Time time) {
    this->mountCapTime = time;
}

void CapStation::setPossibleCapColors(std::set<Workpiece::Color> possibleCapColors) {
    this->possibleCapColors = possibleCapColors;
}

void CapStation::addPossibleCapColor(Workpiece::Color color) {
    this->possibleCapColors.insert(color);
}

void CapStation::setFedCapColor(Workpiece::Color color) {
    //PossibleCapColors has to be set before
    assert(isPossibleCapColor(color));
    this->fedCapColor = color;
}

Time CapStation::getFeedCapTime() const {
    return this->feedCapTime;
}

Time CapStation::getMountCapTime() const {
    return this->mountCapTime;
}

std::set<Workpiece::Color> CapStation::getPossibleCapColors() const {
    return this->possibleCapColors;
}

bool CapStation::isPossibleCapColor(Workpiece::Color Color) const {
    return possibleCapColors.find(Color) != possibleCapColors.end();
}

Workpiece::Color CapStation::getFedCapColor() const {
    return this->fedCapColor;
}

bool CapStation::readyToMountRing() {
    return getFedCapColor() != Workpiece::NONE;
}

std::string CapStation::toString() {
    std::string result;
    result += this->Station::toString();
    result += "; Feed: " + std::to_string(getFeedCapTime());
    result += "; Mount: " + std::to_string(getMountCapTime());

    for (auto const& c : getPossibleCapColors()) {
        result += +"; " + Workpiece::toString(c);
    }
    return result;
}