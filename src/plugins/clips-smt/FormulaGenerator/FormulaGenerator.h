#ifndef FORMULAGENERATOR_H
#define FORMULAGENERATOR_H
#include <vector>
#include <iostream>

#include "StepFormula.h"

class FormulaGenerator {
public:
    FormulaGenerator(int amount, GameData& gameData);
    virtual ~FormulaGenerator();

    stepFormula_ptr getStep(int i);
    void setSteps(int amount, GameData& gameData) ;
private:
    std::vector<stepFormula_ptr> steps;
};




#endif /* FORMULAGENERATOR_H */

