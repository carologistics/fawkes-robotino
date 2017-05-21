#ifndef FORMULAGENERATOR_H
#define FORMULAGENERATOR_H
#include <vector>
#include <iostream>

#include "StepFormula.h"

/*
 * provides methods to generate a formula for the wished amount of steps, 
 * a step denotes the possible interactions of a robot with a station at a certain world state.
 * A variable assignment is a solution of the formula for k steps if the variables for a certain step 
 * encodes the world state after exactly one valid action of an robot based on the world state in the step before.
 * The 0th step represents the current world state and its variables are initially assigned with values from gameData.
 */

class FormulaGenerator {
public:
    FormulaGenerator(int amount, GameData& gameData);
    virtual ~FormulaGenerator();

    stepFormula_ptr getStep(int i);
    std::vector<stepFormula_ptr> getSteps();
    Formula getFormula();
    
    /* generates the needed amount of StepFormula objects 
     * based on the current world state encoded in gameData*/
    void generateSteps(int amount, GameData& gameData) ;
    
    /*creates and returns the formula, the formula is saved in the formula attribute*/
    Formula createFormula();
    
private:
    std::vector<stepFormula_ptr> steps;
    Formula formula;
};




#endif /* FORMULAGENERATOR_H */

