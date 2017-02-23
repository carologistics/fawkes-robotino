#include "Machine.h"

Machine::Machine(int id) {
    this->setId(id);
}

Machine::~Machine() {
}

void Machine::addMovingTime(Machine& m1, Machine& m2, int time){
    m1.addMovingTime(m2, time);
    m2.addMovingTime(m1, time);
}

void Machine::setId(int id) {
    this->id = id;
}

void Machine::setType(std::string type) {
    this->type = type;
}

void Machine::addMovingTime(Machine m, int time) {
    this->movingTimes[m] = time;
}

void Machine::setMovingTimes(std::map<Machine, int> movingTimes) {
    this->movingTimes = movingTimes;
}

void Machine::setWorkpiece(Workpiece& workpiece){
    this->workpiece = workpiece;
}

int Machine::getId() const {
    return this->id;
}

std::string Machine::getType() const {
    return this->type;
}

std::map<Machine, int> Machine::getMovingTimes() {
    return movingTimes;
}

int Machine::getMovingTime(Machine m) {
    return movingTimes[m];
}

Workpiece Machine::getWorkpiece() const{
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

std::string Machine::getVarIdentifier() const {
    return this->getType() + std::to_string(this->getId());
}

std::string Machine::toString() {
    std::string result;
    result += "\nIdentifier: " + getVarIdentifier();
    result += "\nWorkpiece:\n" + getWorkpiece().toString();
    result += "\nMovingTimes:\n" ;
    for (auto const& m : getMovingTimes()) {
        result += m.first.getVarIdentifier() + ": " + std::to_string(m.second) +"\n";
    }
    return result;
}

bool Machine::operator<(const Machine& rhs) const {
    return this->getVarIdentifier() < rhs.getVarIdentifier();
}
