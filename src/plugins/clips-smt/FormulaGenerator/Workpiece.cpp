#include "Workpiece.h"
#include "BaseStation.h"
#include<iostream>

int Workpiece::getMaxRingNumber(){
    return maxRingNumber;
}

Workpiece::Workpiece() {
}

Workpiece::Workpiece(Color base, std::vector<Color> rings, Color cap){
    setBaseColor(base);
    setRings(rings);
    setCapColor(cap);
}

Workpiece::Workpiece(Color base, std::vector<Color> rings){
    setBaseColor(base);
    setRings(rings);
}

Workpiece::Workpiece(Color base){
    setBaseColor(base);
}

Workpiece::~Workpiece() {
}

Workpiece::Color Workpiece::getBaseColor() const {
    return this->base;
}

Workpiece::Color Workpiece::getRingColor(int r) const {
    return this->rings[r];
}

Workpiece::Color Workpiece::getCapColor() const {
    return this->cap;
}

std::vector<Workpiece::Color> Workpiece::getRings() const {
    return this->rings;
}

void Workpiece::setBaseColor(Color c) {
    this->base = c;
}

void Workpiece::setRingColor(Color c, int i) {
    this->rings[i] = c;

}

void Workpiece::setCapColor(Color c) {
    this->cap = c;
}

void Workpiece::setRings(std::vector<Color> rings) {
    if(this->rings.size() < rings.size()) {
        std::cerr <<  "Workpiece::setRings(std::vector<Color> rings): GameData::maxRingNumber " << this->rings.size() << " < rings.size() " << rings.size() << std::endl;
	}

	int rings_size = rings.size();
    for (auto i = 0; i < rings_size; i++) {
        this->rings[i] = rings[i];
    }
}

std::string Workpiece::toString() {
    std::string result;
    result += "Base: " + ColorNames[getBaseColor()];
    result += "\nRings: ";
    for (auto const& r : getRings()) {
        result += ColorNames[r] + "  ";
    }
    result += "\nCap: " + ColorNames[getCapColor()];

    return result;
}
