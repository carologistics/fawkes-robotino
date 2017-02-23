#ifndef MACHINE_H
#define MACHINE_H

#include <string>
#include <map>
#include <memory>

#include "Workpiece.h"

class Machine;

typedef std::shared_ptr<Machine> machine_ptr;

class Machine {
public:
    Machine(int id);
    virtual ~Machine();
    
    static void addMovingTime(Machine& m1, Machine& m2, int time);

    void setId(int id);
    void setType(std::string type);
    void addMovingTime(Machine const m, int time);
    void setMovingTimes(std::map<Machine, int> movingTimes);
    void setWorkpiece(Workpiece& workpiece);

    int getId() const;
    std::string getType() const;
    std::map<Machine, int> getMovingTimes();
    int getMovingTime(Machine m);
    Workpiece getWorkpiece() const;
    Workpiece::Color getBaseColor() const;
    Workpiece::Color getRingColor(int number) const;
    Workpiece::Color getCapColor() const;

    
    std::string getVarIdentifier() const;

    std::string toString();
    
    /* provides an order on machines under the assumption the pair (type, id) is unique
     * (i.e. all machines of a special type have different IDs)
     */
    bool operator<(const Machine& rhs) const;

private:
    int id;
    //is used to create formulas, has to be unique for each machine type
    std::string type = "";
    std::map<Machine, int> movingTimes;
    
    Workpiece workpiece;
};

#endif /* MACHINE_H */

