#ifndef ORDER_H
#define ORDER_H

#include <memory>
#include <string>

#include "Workpiece.h"

class Order;
typedef std::shared_ptr<Order> order_ptr;

class Order {
public:
    enum State {NOTDELIVERED, DELIVERED};
    
    Order(int id, Workpiece product, int deadline);
    virtual ~Order();

    int getId() const;
    int getDeadline() const;
    Workpiece getProduct() const;
    std::string getVarIdentifier();

    Workpiece::Color getBaseColorReq() const;
    Workpiece::Color getRingColorReq(int number) const;
    Workpiece::Color getCapColorReq() const;

    std::vector<Workpiece::Color> getRingColorReq() const;

    void setId(int id);
    void setDeadline(int deadline);
    void setProduct(Workpiece& w);

    void setBaseColorReq(Workpiece::Color color);
    void setRingColorReq(Workpiece::Color color, int number);
    void setCapColorReq(Workpiece::Color color);

    std::string toString();

private:
    int id = -1;
    int deadline = -1;
    Workpiece product;
};

#endif /* ORDER_H */

