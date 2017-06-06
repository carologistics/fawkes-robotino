
#include <algorithm> 

#include "GameData.h"

GameData::GameData() {
}

GameData::~GameData() {
}

std::vector<robot_ptr> const GameData::getRobots() const {
    return this->robots;
}

std::vector<station_ptr> const GameData::getStations() const {
    return stations;
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

/*void GameData::setMachines(std::vector<robot_ptr> const robots) {
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
}*/

void GameData::setOrders(std::vector<order_ptr> const orders) {
    this->orders = orders;
}

void GameData::addStation(station_ptr const station) {
    this->stations.push_back(station);
}

void GameData::addMachine(robot_ptr const robot) {
    this->robots.push_back(robot);
}

void GameData::addMachine(baseStation_ptr const baseStation) {
    this->baseStations.push_back(baseStation);
    addStation(baseStation);
}

void GameData::addMachine(ringStation_ptr const ringStation) {
    this->ringStations.push_back(ringStation);
    addStation(ringStation);
}

void GameData::addMachine(capStation_ptr const capStation) {
    this->capStations.push_back(capStation);
    addStation(capStation);
}

void GameData::addMachine(deliveryStation_ptr const deliveryStation) {
    this->deliveryStations.push_back(deliveryStation);
    addStation(deliveryStation);
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

std::vector<Order> GameData::getOrdersWithAddBaseReq() const {
    std::vector<Order> ordersReq;
    for (auto const& o : getOrders()) {
        for (auto c : o->getRingColorReq())
            if (c != Workpiece::NONE && getNeededAdditionalBases(c) > 0) {
                ordersReq.push_back(*o);
            }
    }
    return ordersReq;
}

int GameData::getNeededAdditionalBases(Workpiece::Color color) const {
    
    for (auto rs : getRingStations()) {
        if (rs->isPossibleRingColor(color)) {
            return rs->getNeededAdditionalBases(color);
        }
    }
    throw "There does not exist a Ring Station with produces" + Workpiece::toString(color) + "\n";
}

std::vector<Order::State> GameData::getProductionStepsRing(const Order& o, int ringPosition) {
    std::vector<Order::State> steps;
    int addBases = getNeededAdditionalBases(o.getRingColorReq(ringPosition));
    for (int j = 0; j < addBases; j++) {
        steps.push_back(o.getCollectBaseId(ringPosition, j));
        steps.push_back(o.getFedBaseId(ringPosition, j));
    }
    return steps;
}

/*std::vector<Order::State> GameData::getProductionSteps(const Order& o) {
    std::vector<Order::State> steps;
    steps.push_back(Order::BASE);
    std::vector<Workpiece::Color> rings = o.getProduct().getRings();
    for (int i = 0; i < rings.size(); i++) {
        if (rings[i] != Workpiece::NONE) {
            steps.push_back(o.getSetupRingId(i));
            std::vector<Order::State> rings = getProductionStepsRing(o, i);
            steps.insert(steps.end(), rings.begin(), rings.end());
            steps.push_back(o.getRingId(i));
        }
    }
    steps.push_back(Order::FEDCAP);
    steps.push_back(Order::CAP);
    steps.push_back(Order::DELIVERED);
    return steps;
}*/

Order::State GameData::getPrevProductionStepMain(const Order& o, Order::State state) {
    Order::State result = Order::NOTDEFINED;
    std::vector<Workpiece::Color> rings = o.getRingColorReq();
    if (isMainProd(o, state)) {
        switch (state) {
            case Order::BASE: result = Order::NONE;
                break;
            case Order::SETUP_RING0: result = Order::BASE;
                break;
            case Order::RING0: result = Order::SETUP_RING0;
                break;
            case Order::SETUP_RING1: result = Order::RING0;
                break;
            case Order::RING1: result = Order::SETUP_RING1;
                break;
            case Order::SETUP_RING2: result = Order::RING1;
                break;
            case Order::RING2: result = Order::SETUP_RING2;
                break;
            case Order::CAP:
                if (rings[0] == Workpiece::NONE) {
                    result = Order::BASE;
                } else if (rings[1] == Workpiece::NONE) {
                    result = Order::RING0;
                } else if (rings[2] == Workpiece::NONE) {
                    result = Order::RING1;
                } else {
                    result = Order::RING2;
                }
                break;
            case Order::DELIVERED: result = Order::CAP;
                break;
            default: ;
        }
    }
    return result;
}

Order::State GameData::getPrevProductionStepCap(const Order& o, Order::State state) {
    Order::State result = Order::NOTDEFINED;
    if (isCapProd(state)) {
        result = Order::NONE;
    }
    return result;
}

Order::State GameData::getPrevProductionStepRing(const Order& o, Order::State state) {
    Order::State result = Order::NOTDEFINED;
    std::vector<Workpiece::Color> rings = o.getRingColorReq();
    if (isRingProd(o, state)) {
        for (int i = 0; i < o.getRingCount(); i++) {
            if (state == o.getCollectBaseId(i, 0) || state == o.getCollectBaseId(i, 1)) {
                result = Order::NONE;
            }
            if (state == o.getFedBaseId(i, 0)) {
                result = o.getCollectBaseId(i, 0);
            }
            if (state == o.getFedBaseId(i, 1)) {
                result = o.getCollectBaseId(i, 1);
            }
        }
    }
    return result;
}

bool GameData::isMainProd(Order o, Order::State state) const {
    bool result = false;

    std::vector<Workpiece::Color> rings = o.getRingColorReq();

    switch (state) {
        case Order::BASE: result = true;
            break;
        case Order::SETUP_RING0:
            if (rings[0] != Workpiece::NONE) result = true;
            break;
        case Order::RING0:
            if (rings[0] != Workpiece::NONE) result = true;
            break;
        case Order::SETUP_RING1:
            if (rings[1] != Workpiece::NONE) result = true;
            break;
        case Order::RING1:
            if (rings[1] != Workpiece::NONE) result = true;
            break;
        case Order::SETUP_RING2:
            if (rings[2] != Workpiece::NONE) result = true;
            break;
        case Order::RING2:
            if (rings[2] != Workpiece::NONE) result = true;
            break;
        case Order::CAP: result = true;
            break;
        case Order::DELIVERED: result = true;
            break;
        case Order::NOTDELIVERED: result = true;
            break;
        default: ;
    }
    return result;
}

bool GameData::isCapProd(Order::State state) const {
    return Order::FEDCAP == state;
}

bool GameData::isRingProd(Order o, Order::State state) const {
    bool result = false;
    std::vector<Workpiece::Color> rings = o.getRingColorReq();

    switch (state) {
        case Order::COLLECT_BASE0_RING0:
            if (rings[0] != Workpiece::NONE && getNeededAdditionalBases(rings[0]) > 0) result = true;
            break;
        case Order::FED_BASE0_RING0:
            if (rings[0] != Workpiece::NONE && getNeededAdditionalBases(rings[0]) > 0) result = true;
            break;
        case Order::COLLECT_BASE1_RING0:
            if (rings[0] != Workpiece::NONE && getNeededAdditionalBases(rings[0]) > 1) result = true;
            break;
        case Order::FED_BASE1_RING0:
            if (rings[0] != Workpiece::NONE && getNeededAdditionalBases(rings[0]) > 1) result = true;
            break;
        case Order::COLLECT_BASE0_RING1:
            if (rings[1] != Workpiece::NONE && getNeededAdditionalBases(rings[1]) > 0) result = true;
            break;
        case Order::FED_BASE0_RING1:
            if (rings[1] != Workpiece::NONE && getNeededAdditionalBases(rings[1]) > 0) result = true;
            break;
        case Order::COLLECT_BASE1_RING1:
            if (rings[1] != Workpiece::NONE && getNeededAdditionalBases(rings[1]) > 1) result = true;
            break;
        case Order::FED_BASE1_RING1:
            if (rings[1] != Workpiece::NONE && getNeededAdditionalBases(rings[1]) > 1) result = true;
            break;
        case Order::COLLECT_BASE0_RING2:
            if (rings[2] != Workpiece::NONE && getNeededAdditionalBases(rings[2]) > 0) result = true;
            break;
        case Order::FED_BASE0_RING2:
            if (rings[2] != Workpiece::NONE && getNeededAdditionalBases(rings[2]) > 0) result = true;
            break;
        case Order::COLLECT_BASE1_RING2:
            if (rings[2] != Workpiece::NONE && getNeededAdditionalBases(rings[2]) > 1) result = true;
            break;
        case Order::FED_BASE1_RING2:
            if (rings[2] != Workpiece::NONE && getNeededAdditionalBases(rings[2]) > 1) result = true;
            break;
        default : ;
    }
    return result;
}

std::string GameData::toString() const {
    std::string result;
    for (auto const& r : getRobots()) {
        result += r->toString() + "\n";
    }

    for (auto const& s : getBaseStations()) {
        result += s->toString() + "\n";
    }
    
    for (auto const& s : getRingStations()) {
        result += s->toString() + "\n";
    }
    
    for (auto const& s : getCapStations()) {
        result += s->toString() + "\n";
    }
    
    for (auto const& s : getDeliveryStations()) {
        result += s->toString() + "\n";
    }

    for (auto const& ds : getOrders()) {
        result += ds->toString() + "\n";
    }

    return result;
}

Reward GameData::getReward() const {
    return reward;
}
