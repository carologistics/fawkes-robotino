#include "Workpiece.h"
#include "BaseStation.h"
#include<iostream>

int Workpiece::getMaxRingNumber() {
    return maxRingNumber;
}

Workpiece::Workpiece() {
}

Workpiece::Workpiece(Color base, std::vector<Color> rings, Color cap) {
    setBaseColor(base);
    setRings(rings);
    setCapColor(cap);
}

Workpiece::Workpiece(Color base, std::vector<Color> rings) {
    setBaseColor(base);
    setRings(rings);
}

Workpiece::Workpiece(Color base) {
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
    if (this->rings.size() < rings.size())
        std::cerr << "Workpiece::setRings(std::vector<Color> rings): GameData::maxRingNumber < rings.size()";

    for (uint i = 0; i < rings.size(); i++) {
        this->rings[i] = rings[i];
    }
}

Workpiece::Color Workpiece::intToColor(int color) {
    switch (color) {
        case 0:
            return Workpiece::NONE;
        case 1:
            return Workpiece::RED;
        case 2:
            return Workpiece::BLACK;
        case 3:
            return Workpiece::SILVER;
        case 4:
            return Workpiece::TRANSPARENT;
        case 5:
            return Workpiece::BLUE;
        case 6:
            return Workpiece::GREEN;
        case 7:
            return Workpiece::YELLOW;
        case 8:
            return Workpiece::ORANGE;
        case 9:
            return Workpiece::GREY;
        default:
            return Workpiece::NOTDEFINED;
    }
}

std::string Workpiece::toString(const Workpiece::Color color) {
    return colorNames.at(color);
}

std::string Workpiece::toString() {
    std::string result;
    result += "(" + toString(getBaseColor()) + ", ";
    for (auto const& r : getRings()) {
        result += toString(r) + ", ";
    }
    result += toString(getCapColor()) + ")";

    return result;
}

const std::map<Workpiece::Color, std::string> Workpiece::colorNames = {
    {Workpiece::NOTDEFINED, "NOTDEFINED"},
    {Workpiece::NONE, "NONE"},
    {Workpiece::RED, "RED"},
    {Workpiece::BLACK, "BLACK"},
    {Workpiece::SILVER, "SILVER"},
    {Workpiece::TRANSPARENT, "TRANSPARENT"},
    {Workpiece::BLUE, "BLUE"},
    {Workpiece::GREEN, "GREEN"},
    {Workpiece::YELLOW, "YELLOW"},
    {Workpiece::ORANGE, "ORANGE"},
    {Workpiece::GREY, "GREY"}
};


