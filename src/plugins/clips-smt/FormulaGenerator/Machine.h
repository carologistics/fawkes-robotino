#ifndef MACHINE_H
#define MACHINE_H

#include <string>
#include <map>
#include <memory>

#include "Workpiece.h"

/*
 * A Machine is either a robot or a station. 
 * Stores information about the current world state and is used for variable creation
 */

class Machine;

typedef std::shared_ptr<Machine> machine_ptr;

class Machine {
public:
    Machine(std::string type, int id);
    virtual ~Machine();
    
    static void addMovingTime(Machine& m1, Machine& m2, int time);

    void setId(int id);
    void setType(std::string type);
    void addMovingTime(const Machine&  m, int time);
    void setMovingTimes(const std::map<Machine, int>& movingTimes);
    void setWorkpiece(Workpiece& workpiece);

    int getId() const;
    std::string const getType() const;
    std::map<Machine, int> getMovingTimes() const;
    int getMovingTime(const Machine& m) const;
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
    
    bool operator==(const Machine& rhs) const;
    bool operator!=(const Machine& rhs) const;

private:
    int id;
    //is used to create formulas, has to be unique for each machine type
    std::string type = "";
    std::map<Machine, int> movingTimes;
    //determines which workpiece a robot holds or a station has in its output
    Workpiece workpiece;
};

#endif /* MACHINE_H */

