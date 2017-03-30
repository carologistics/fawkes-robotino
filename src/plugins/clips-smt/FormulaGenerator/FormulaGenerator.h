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
    std::vector<stepFormula_ptr> getSteps();
    Formula getFormula();

    void generateSteps(int amount, GameData& gameData) ;
    Formula createFormula();

private:
    std::vector<stepFormula_ptr> steps;
    Formula formula;
};




#endif /* FORMULAGENERATOR_H */
