#include "Machine.h"
#include <iostream>

Machine::Machine(std::string type, int id) {
    this->setId(id);
    this->setType(type);
    this->addMovingTime(*this, 0);
}

Machine::~Machine() {
}

void Machine::addMovingTime(Machine& m1, Machine& m2, Time time) {
    m1.addMovingTime(m2, time);
    m2.addMovingTime(m1, time);
}

void Machine::setId(int id) {
    this->id = id;
}

void Machine::setType(std::string type) {
    this->type = type;
}

void Machine::addMovingTime(const Machine& m, Time time) {
    this->movingTimes.emplace(m, time);
}

void Machine::setMovingTimes(const std::map<Machine, Time>& movingTimes) {
    this->movingTimes = movingTimes;
    this->movingTimes.emplace(*this, 0);
}

void Machine::setWorkpiece(Workpiece& workpiece) {
    this->workpiece = workpiece;
}

int Machine::getId() const {
    return this->id;
}

std::string const Machine::getType() const {
    return this->type;
}

std::map<Machine, Time> Machine::getMovingTimes() const {
    return movingTimes;
}

Time Machine::getMovingTime(Machine const& m) const {
    Time time = 0;
    try {
        time = movingTimes.at(m);
    } catch (const std::out_of_range& oor) {
        std::cerr << "getMovingTime: "
                + m.getVarIdentifier()
                + " not in MovingTimes of "
                + this->getVarIdentifier() + "\n";
    }
    return time;
}

Workpiece Machine::getWorkpiece() const {
    return this->workpiece;
}

Workpiece::Color Machine::getBaseColor() const {
    return getWorkpiece().getBaseColor();
}

Workpiece::Color Machine::getRingColor(int number) const {
    return getWorkpiece().getRingColor(number);
}

Workpiece::Color Machine::getCapColor() const {
    return getWorkpiece().getCapColor();
}

bool Machine::isBaseStation() const{
    return this->getType() == "bs";
}

bool Machine::isRingStation() const {
    return this->getType() == "rs";
}

bool Machine::isCapStation() const {
    return this->getType() == "cs";
}

bool Machine::isDeliveryStation() const {
    return this->getType() == "ds";
}

bool Machine::isRobot() const {
    return this->getType() == "r";
}

std::string Machine::getVarIdentifier() const {
    return this->getType() + std::to_string(this->getId());
}

std::string Machine::toString() {
    std::string result;
    result += "\nIdentifier: " + getVarIdentifier();
    result += "\nWorkpiece:\n" + getWorkpiece().toString();
    result += "\nMovingTimes:\n";
    for (auto const& m : getMovingTimes()) {
        result += m.first.getVarIdentifier() + ": " + std::to_string(m.second) + "\n";
    }
    return result;
}

bool Machine::operator<(const Machine& rhs) const {
    return this->getVarIdentifier() < rhs.getVarIdentifier();
}

bool Machine::operator==(const Machine& rhs) const {
    return this->getVarIdentifier() == rhs.getVarIdentifier();
}

bool Machine::operator!=(const Machine& rhs) const {
    return !(*this == rhs);
}