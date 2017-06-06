#ifndef ORDER_H
#define ORDER_H

#include <memory>
#include <string>

#include "Workpiece.h"
#include "Machine.h"

class Order;
typedef std::shared_ptr<Order> order_ptr;

class Order {
public:

    enum State {
        NONE,
        BASE,
        SETUP_RING0, COLLECT_BASE0_RING0, FED_BASE0_RING0, COLLECT_BASE1_RING0, FED_BASE1_RING0, RING0,
        SETUP_RING1, COLLECT_BASE0_RING1, FED_BASE0_RING1, COLLECT_BASE1_RING1, FED_BASE1_RING1, RING1,
        SETUP_RING2, COLLECT_BASE0_RING2, FED_BASE0_RING2, COLLECT_BASE1_RING2, FED_BASE1_RING2, RING2,
        FEDCAP, CAP,
        DELIVERED, NOTDELIVERED, NOTDEFINED
    };

    Order(int id, Workpiece product, Time deadline);
    virtual ~Order();

    int getId() const;
    Time getDeadline() const;
    Workpiece getProduct() const;
    std::string getVarIdentifier();

    Workpiece::Color getBaseColorReq() const;
    Workpiece::Color getRingColorReq(int number) const;
    Workpiece::Color getCapColorReq() const;

    std::vector<Workpiece::Color> getRingColorReq() const;

    void setId(int id);
    void setDeadline(Time deadline);
    void setProduct(Workpiece& w);

    void setBaseColorReq(Workpiece::Color color);
    void setRingColorReq(Workpiece::Color color, int number);
    void setCapColorReq(Workpiece::Color color);

    State getSetupRingId(int ringPosition) const;
    State getRingId(int ringPosition) const;
    State getCollectBaseId(int ringPosition, int addBaseNumber) const;
    State getFedBaseId(int ringPosition, int addBaseNumber) const;
    
    int getRingCount() const;

    static State intToState(int state);
    
    std::string toString();
    static std::string toString(State);
    bool operator<(const Order& rhs) const;
    bool operator==(const Order& rhs) const;
    bool operator!=(const Order& rhs) const;

private:
    int id = -1;
    Time deadline = 0;
    Workpiece product;
    static const std::map<State, std::string> stateNames;
};

#endif /* ORDER_H */

