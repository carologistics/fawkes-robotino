#ifndef MACHINE_H
#define MACHINE_H

#include <string>
#include <map>
#include <memory>

class Machine;

typedef std::shared_ptr<Machine> machine_ptr;

class Machine {
public:
    Machine(int id);
    virtual ~Machine();

    void setId(int id);
    void setType(std::string type);
    void setMovingTime(Machine const m, int time);
    void setMovingTimes(std::map<Machine, int> movingTimes);

    int getId() const;
    std::string getType() const;
    std::map<Machine, int> getMovingTimes();
    int getMovingTime(Machine m);
    
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
};

#endif /* MACHINE_H */

