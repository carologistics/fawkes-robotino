#include "Machine.h"

Machine::Machine(int id) {
    this->setId(id);
}

Machine::~Machine() {
}

void Machine::setId(int id) {
    this->id = id;
}

void Machine::setType(std::string type) {
    this->type = type;
}

void Machine::setMovingTime(Machine m, int time) {
    this->movingTimes[m] = time;
}

void Machine::setMovingTimes(std::map<Machine, int> movingTimes) {
    this->movingTimes = movingTimes;
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

std::string Machine::getVarIdentifier() const {
    return this->getType() + std::to_string(this->getId());
}

std::string Machine::toString() {
    std::string result = "Robot: " + std::to_string(this->getId());
    result += "\nIdentifier: " + getVarIdentifier();
    result += "\nMovingTimes:\n" ;
    for (auto const& m : getMovingTimes()) {
        result += m.first.getVarIdentifier() + ": " + std::to_string(m.second) +"\n";
    }
    return result;
}

bool Machine::operator<(const Machine& rhs) const {
    return this->getVarIdentifier() < rhs.getVarIdentifier();
}
