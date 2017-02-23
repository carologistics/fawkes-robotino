#include <iostream>
#include "FormulaGenerator.h"

FormulaGenerator::FormulaGenerator(int amount, GameData& gameData) {
    generateSteps(amount, gameData);
}

FormulaGenerator::~FormulaGenerator() {
}

stepFormula_ptr FormulaGenerator::getStep(int i){
    return this->steps[i];
}

std::vector<stepFormula_ptr> FormulaGenerator::getSteps(){
    return this->steps;
}

void FormulaGenerator::generateSteps(int amount, GameData& gameData) {
    this->steps = StepFormula::generateSteps(amount, gameData);
}

Formula FormulaGenerator::createFormula(){
    
    std::vector<Formula> formulas;
    for(auto const& s : getSteps()) {
        formulas.push_back(s->create());
    }
    return Formula(carl::FormulaType::AND,formulas);
}