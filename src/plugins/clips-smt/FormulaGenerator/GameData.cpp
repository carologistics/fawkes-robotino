#include "GameData.h"

GameData::GameData() {
}

GameData::~GameData() {
}

std::vector<robot_ptr> const GameData::getRobots() const {
    return this->robots;
}

void const GameData::fillStations() {
    stations.reserve(baseStations.size() + ringStations.size()
            + capStations.size() + deliveryStations.size()); // preallocate memory
    stations.insert(stations.end(), baseStations.begin(), baseStations.end());
    stations.insert(stations.end(), ringStations.begin(), ringStations.end());
    stations.insert(stations.end(), capStations.begin(), capStations.end());
    stations.insert(stations.end(), deliveryStations.begin(), deliveryStations.end());
}

std::vector<station_ptr> const GameData::getStations() {
    return this->stations;
}

std::vector<baseStation_ptr> const GameData::getBaseStations() const {
    return this->baseStations;
}

std::vector<ringStation_ptr> const GameData::getRingStations() const {
    return this->ringStations;
}

std::vector<capStation_ptr> const GameData::getCapStations() const {
    return this->capStations;
}

std::vector<deliveryStation_ptr> const GameData::getDeliveryStations() const {
    return this->deliveryStations;
}

std::vector<order_ptr> const GameData::getOrders() const {
    return this->orders;
}

/*std::map<machine_machine, int> const GameData::getMovingTimes() const {
    return this->movingTimes;
}*/

void GameData::setMachines(std::vector<robot_ptr> const robots) {
    this->robots = robots;
}

void GameData::setMachines(std::vector<baseStation_ptr> const baseStations) {
    this->baseStations = baseStations;
}

void GameData::setMachines(std::vector<ringStation_ptr> const ringStations) {
    this->ringStations = ringStations;
}

void GameData::setMachines(std::vector<capStation_ptr> const capStations) {
    this->capStations = capStations;
}

void GameData::setMachines(std::vector<deliveryStation_ptr> const deliveryStations) {
    this->deliveryStations = deliveryStations;
}

void GameData::setOrders(std::vector<order_ptr> const orders) {
    this->orders = orders;
}

/*void GameData::setMovingTimes(std::map<machine_machine, int> const movingTimes) {
    this->movingTimes = movingTimes;
}*/

void GameData::addMachine(robot_ptr const robot) {
    this->robots.push_back(robot);
}

void GameData::addMachine(baseStation_ptr const baseStation) {
    this->baseStations.push_back(baseStation);
}

void GameData::addMachine(ringStation_ptr const ringStation) {
    this->ringStations.push_back(ringStation);
}

void GameData::addMachine(capStation_ptr const capStation) {
    this->capStations.push_back(capStation);
}

void GameData::addMachine(deliveryStation_ptr const deliveryStation) {
    this->deliveryStations.push_back(deliveryStation);
}

void GameData::addOrder(order_ptr const order) {
    this->orders.push_back(order);
}

bool GameData::existsOrderWithBaseReq(Workpiece::Color c) const {
    for (auto const& o : getOrders()) {
        if (o->getBaseColorReq() == c)
            return true;
    }
    return false;
}

bool GameData::existsOrderWithRingReq(int i, Workpiece::Color c) const {
    for (auto const& o : getOrders()) {
        if (o->getRingColorReq(i) == c)
            return true;
    }
    return false;
}

bool GameData::existsOrderWithRingReq(Workpiece::Color c) const {
    for (int i = 0; i <= Workpiece::getMaxRingNumber(); i++) {
        if (existsOrderWithRingReq(i, c)) return true;
    }
    return false;
}

bool GameData::existsOrderWithCapReq(Workpiece::Color c) const {
    for (auto const& o : getOrders()) {
        if (o->getCapColorReq() == c)
            return true;
    }
    return false;
}

std::vector<Order> GameData::getOrdersWithBaseReq(Workpiece::Color c) const {
    std::vector<Order> ordersReq;
    for (auto const& o : getOrders()) {
        if (o->getBaseColorReq() == c)
            ordersReq.push_back(*o);
    }
    return ordersReq;
}

std::vector<Order> GameData::getOrdersWithRingReq(Workpiece::Color c) const {
    std::vector<Order> ordersReq;
    for (auto const& o : getOrders()) {
        for (int i = 0; i <= Workpiece::getMaxRingNumber(); i++) {
            if (o->getRingColorReq(i) == c) {
                ordersReq.push_back(*o);
                break;
            }
        }
    }
    return ordersReq;
}

std::vector<Order> GameData::getOrdersWithCapReq(Workpiece::Color c) const {
    std::vector<Order> ordersReq;
    for (auto const& o : getOrders()) {
        if (o->getCapColorReq() == c)
            ordersReq.push_back(*o);
    }
    return ordersReq;
}

std::string GameData::toString() const {
    std::string result;
    result += "Robots:            ";
    for (auto const& r : getRobots()) {
        result += r->getVarIdentifier() + "  ";
    }
    result += "\nBaseStations:     ";
    for (auto const& bs : getBaseStations()) {
        result += bs->getVarIdentifier() + " ";
    }
    result += "\nRingStations:     ";
    for (auto const& rs : getRingStations()) {
        result += rs->getVarIdentifier() + " ";
    }
    result += "\nCapStations:      ";
    for (auto const& cs : getCapStations()) {
        result += cs->getVarIdentifier() + " ";
    }
    result += "\nDeliveryStations: ";
    for (auto const& ds : getDeliveryStations()) {
        result += ds->getVarIdentifier() + " ";
    }
    result += "\nOrders: \n";
    for (auto const& ds : getOrders()) {
        result += ds->toString() + "\n ";
    }
    /*result += "\nMovingTimes: \n";
    for (auto const& mt : getMovingTimes()) {
        Machine m1 = (mt.first.first);
        Machine m2 = (mt.first.second);
        std::string time = std::to_string(mt.second);
        result += m1.toString() + "->" + m2.toString() + ": " + time + "\n";
    }*/
    return result;
}

Reward GameData::getReward() const {
    return reward;
}
