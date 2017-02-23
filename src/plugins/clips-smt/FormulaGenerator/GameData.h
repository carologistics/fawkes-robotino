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

//typedef std::pair<Machine, Machine> machine_machine;

/* We represent the world state as if all robots had finished their current
 * actions - even if it is not the case. For a given robot r, the moving time
 * for each station s is set to: r->x = r->d + d->s, 
 * d denotes the station on which the robot performs its action
 */

class GameData {
public:
    GameData();
    virtual ~GameData();

    std::vector<robot_ptr> const getRobots() const;
    std::vector<baseStation_ptr> const getBaseStations() const;
    std::vector<ringStation_ptr> const getRingStations() const;
    std::vector<capStation_ptr> const getCapStations() const;
    std::vector<deliveryStation_ptr> const getDeliveryStations() const;
    std::vector<order_ptr> const getOrders() const;
    //std::map<machine_machine, int> const getMovingTimes() const;

    void setMachines(std::vector<robot_ptr> const robots);
    void setMachines(std::vector<baseStation_ptr> const baseStations);
    void setMachines(std::vector<ringStation_ptr> const ringStations);
    void setMachines(std::vector<capStation_ptr> const capStations);
    void setMachines(std::vector<deliveryStation_ptr> const deliveryStations);
    void setOrders(std::vector<order_ptr> const orders);

    //void setMovingTimes(std::map<machine_machine, int> const movingTimes);

    void addMachine(robot_ptr const robot);
    void addMachine(baseStation_ptr const baseStation);
    void addMachine(ringStation_ptr const ringStation);
    void addMachine(capStation_ptr const capStation);
    void addMachine(deliveryStation_ptr const deliveryStation);
    void addOrder(order_ptr const order);
    
    //assumes the symmetry of the moving times, and automatically adds m2->m1, too.
    //void addMovingTime(Machine const m1, Machine const m2, int movingTime);

    std::string toString() const;

private:
    std::vector<robot_ptr> robots;
    std::vector<baseStation_ptr> baseStations;
    std::vector<ringStation_ptr> ringStations;
    std::vector<capStation_ptr> capStations;
    std::vector<deliveryStation_ptr> deliveryStations;
    std::vector<order_ptr> orders;
    //replaced this with map for times in each machine
    //std::map<machine_machine, int> movingTimes;
};

#endif /* GAMEDATA_H */

