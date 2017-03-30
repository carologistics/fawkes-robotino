#include "Workpiece.h"
#include "BaseStation.h"
#include<iostream>

int Workpiece::getMaxRingNumber(){
    return maxRingNumber;
}

Workpiece::Workpiece() {
    base = NONE;
    rings = std::vector<Color>(getMaxRingNumber(), Color(NONE));
    cap = NONE;
    init();
}

Workpiece::Workpiece(Color base, std::vector<Color> rings, Color cap){
    setBaseColor(base);
    setRings(rings);
    setCapColor(cap);
    init();
}

Workpiece::Workpiece(Color base, std::vector<Color> rings){
    setBaseColor(base);
    setRings(rings);
    cap = NONE;
    init();
}

Workpiece::Workpiece(Color base){
    setBaseColor(base);
    rings = std::vector<Color>(getMaxRingNumber(), Color(NONE));
    cap = NONE;
    init();
}

Workpiece::~Workpiece() {
}

void Workpiece::init() {
    ColorNames = {
        {Color::NONE, "NONE"},
        {Color::RED, "RED"},
        {Color::BLACK, "BLACK"},
        {Color::SILVER, "SILVER"},
        {Color::TRANSPARENT, "TRANSPARENT"},
        {Color::BLUE, "BLUE"},
        {Color::GREEN, "GREEN"},
        {Color::YELLOW, "YELLOW"},
        {Color::ORANGE, "ORANGE"},
        {Color::GREY, "GREY"}
    };
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
    if(this->rings.size() < rings.size())
        std::cerr <<  "Workpiece::setRings(std::vector<Color> rings): GameData::maxRingNumber < rings.size()";

    for (uint i = 0; i < rings.size(); i++) {
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
