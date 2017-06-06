#ifndef GAMEDATA_H
#define GAMEDATA_H

#include <vector>
#include <map>
#include <utility> 
#include <memory>

#include "Order.h"
#include "Robot.h"
#include "BaseStation.h"
#include "RingStation.h"
#include "CapStation.h"
#include "DeliveryStation.h"

#include "Reward.h"

/* Contains all relevant Information about the current world state.
 * 
 * We represent the world state as if all robots had finished their current
 * actions - even if it is not the case. For a given robot r, the moving time
 * for each station s is set to: r->x = r->d + d->s, 
 * d denotes the station on which the robot performs its action
 */

class GameData {
public:
    GameData();
    virtual ~GameData();

    std::vector<robot_ptr> const getRobots() const;
    std::vector<station_ptr> const getStations() const;
    std::vector<baseStation_ptr> const getBaseStations() const;
    std::vector<ringStation_ptr> const getRingStations() const;
    std::vector<capStation_ptr> const getCapStations() const;
    std::vector<deliveryStation_ptr> const getDeliveryStations() const;
    std::vector<order_ptr> const getOrders() const;

    void setOrders(std::vector<order_ptr> const orders);

    void addMachine(robot_ptr const robot);
    void addMachine(baseStation_ptr const baseStation);
    void addMachine(ringStation_ptr const ringStation);
    void addMachine(capStation_ptr const capStation);
    void addMachine(deliveryStation_ptr const deliveryStation);
    void addOrder(order_ptr const order);

    bool existsOrderWithBaseReq(Workpiece::Color c) const;
    bool existsOrderWithRingReq(int i, Workpiece::Color c) const;
    bool existsOrderWithRingReq(Workpiece::Color c) const;
    bool existsOrderWithCapReq(Workpiece::Color c) const;

    std::vector<Order> getOrdersWithBaseReq(Workpiece::Color c) const;
    std::vector<Order> getOrdersWithRingReq(Workpiece::Color c) const;
    std::vector<Order> getOrdersWithCapReq(Workpiece::Color c) const;
    std::vector<Order> getOrdersWithAddBaseReq() const;

    int getNeededAdditionalBases(Workpiece::Color) const;
    std::vector<Order::State> getProductionStepsRing(const Order& o, int ringPosition);

    Order::State getPrevProductionStepMain(const Order& o, Order::State state);
    Order::State getPrevProductionStepCap(const Order& o, Order::State state);
    Order::State getPrevProductionStepRing(const Order& o, Order::State state);

    bool isMainProd(Order o, Order::State state) const;
    bool isCapProd(Order::State state) const;
    bool isRingProd(Order o, Order::State state) const;

    std::string toString() const;
    Reward getReward() const;

private:

    void addStation(station_ptr const station);

    std::vector<station_ptr> stations;
    std::vector<robot_ptr> robots;
    std::vector<baseStation_ptr> baseStations;
    std::vector<ringStation_ptr> ringStations;
    std::vector<capStation_ptr> capStations;
    std::vector<deliveryStation_ptr> deliveryStations;
    std::vector<order_ptr> orders;

    Reward reward;
};

#endif /* GAMEDATA_H */

