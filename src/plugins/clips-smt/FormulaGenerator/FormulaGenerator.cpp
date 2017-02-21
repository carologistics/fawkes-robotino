#include <iostream>
#include "FormulaGenerator.h"

FormulaGenerator::FormulaGenerator(int amount, GameData& gameData) {
    setSteps(amount, gameData);
}

FormulaGenerator::~FormulaGenerator() {
}

stepFormula_ptr FormulaGenerator::getStep(int i){
    return this->steps[i];
}

void FormulaGenerator::setSteps(int amount, GameData& gameData) {
    std::vector<stepFormula_ptr> steps;
    std::shared_ptr<StepFormula> prevStep = nullptr;
    
    for (int number = 0; number <= amount; number++) {
        
        std::shared_ptr<StepFormula> step = std::make_shared<StepFormula>(prevStep, gameData);
        steps.push_back(step);
        prevStep = step;
    }
    this->steps = steps;
}
